#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Bool
from pix_robobus_driver_msgs.msg import VaChassisWheelRpmFb
from geometry_msgs.msg import TwistWithCovarianceStamped

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import csv
import os
import sys
import threading
import logging
from datetime import datetime
import glob


DEFAULT_MCAP_DIR = "/media/ipc/AQLoopCloseData1/perception_data_20260129133840"  # 默认扫描目录

# ----------------- LocalizationCSVExporter -----------------
class LocalizationCSVExporter(Node):
    def __init__(self, outdir):
        super().__init__('localization_csv_exporter_mcap')
        self.outdir = outdir
        os.makedirs(self.outdir, exist_ok=True)

        # ================= timeout（ms） =================
        self.timeout_ms = {
            "fix": 50,
            "heading": 50,
            "imu": 30,
            "twist": 30,
            "fused_twist": 40,
            "wheel_rpm": 40,
        }

        # ================= 控制 =================
        self.save_enabled = True
        self.lock = threading.Lock()

        # ================= 初始化状态 =================
        self.reset_state()

        # ================= CSV =================
        self.csvfile = None
        self.writer = None

        # ================= 日志 =================
        log_name = f"localization_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        logging.basicConfig(
            filename=os.path.join(self.outdir, log_name),
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
            force=True
        )
        
        logging.info(f"Localization CSV Exporter initialized. Output directory: {outdir}")

    def reset_state(self):
        """重置所有状态变量，确保每个文件处理独立"""
        with self.lock:
            # ================= 数据缓存 =================
            self.fix = None
            self.heading = None
            self.imu = None
            self.twist = None
            self.fused_twist = None
            
            self.wheel_rpm_lf = 0.0
            self.wheel_rpm_rf = 0.0
            self.wheel_rpm_lr = 0.0
            self.wheel_rpm_rr = 0.0
            
            # ================= 时间戳 =================
            self.fix_time = None
            self.heading_time = None
            self.imu_time = None
            self.twist_time = None
            self.fused_twist_time = None
            self.wheel_rpm_time = None
            self.now_time = None
            
            # ================= 去重时间戳 =================
            self.last_ts = None  # 重要：重置去重时间戳
            
            # ================= 文件边界标记 =================
            self.file_start_time = None

    # ----------------- 工具 -----------------
    def is_valid(self, name, last_time, timeout_ms):
        if last_time is None or self.now_time is None:
            return False
        dt = (self.now_time - last_time).nanoseconds / 1e6
        return dt <= timeout_ms

    def v(self, ok, value):
        return value if ok else "*"

    # ----------------- 回调 -----------------
    def fix_cb(self, msg, t):
        self.fix = msg
        self.fix_time = t

    def heading_cb(self, msg, t):
        self.heading = msg.data
        self.heading_time = t

    def imu_cb(self, msg, t):
        self.imu = msg
        self.imu_time = t

    def twist_cb(self, msg, t):
        self.twist = msg
        self.twist_time = t

    def fused_twist_cb(self, msg, t):
        self.fused_twist = msg
        self.fused_twist_time = t

    def wheel_rpm_cb(self, msg, t):
        self.wheel_rpm_lf = msg.vcu_chassis_wheel_rpm_lf
        self.wheel_rpm_rf = msg.vcu_chassis_wheel_rpm_rf
        self.wheel_rpm_lr = msg.vcu_chassis_wheel_rpm_lr
        self.wheel_rpm_rr = msg.vcu_chassis_wheel_rpm_rr
        self.wheel_rpm_time = t

    # ----------------- CSV -----------------
    def init_csv(self, mcap_file_name):
        """初始化CSV文件，重置状态"""
        # 重置所有状态，确保文件处理独立
        self.reset_state()
        
        csv_name = f"{os.path.basename(mcap_file_name).replace('.mcap','')}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path = os.path.join(self.outdir, csv_name)
        self.csvfile = open(path, "w", newline="", buffering=1)
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow([
            "timestamp_s",
            "lat", "lon", "alt", "heading",
            "vx", "vy", "vz",
            "imu_wx", "imu_wy", "imu_wz",
            "ax", "ay", "az",
            "wheel_rpm_lf", "wheel_rpm_rf", "wheel_rpm_lr", "wheel_rpm_rr",
            "twist_wx", "twist_wy", "twist_wz",
            "fused_vx", "fused_vy", "fused_vz",
            "fused_wx", "fused_wy", "fused_wz"
        ])
        logging.info(f"Processing MCAP: {mcap_file_name} -> CSV: {csv_name}")
        print(f"  Output CSV: {csv_name}")

    def close_csv(self):
        if self.csvfile:
            self.csvfile.close()
            self.csvfile = None
            self.writer = None

    # ----------------- 写 CSV -----------------
    def timer_write_csv(self):
        """写CSV数据，添加文件边界检查"""
        if not self.fix or not self.writer:
            return

        fix_ok     = self.is_valid("fix", self.fix_time, self.timeout_ms["fix"])
        heading_ok = self.is_valid("heading", self.heading_time, self.timeout_ms["heading"])
        imu_ok     = self.is_valid("imu", self.imu_time, self.timeout_ms["imu"])
        twist_ok   = self.is_valid("twist", self.twist_time, self.timeout_ms["twist"])
        fused_ok   = self.is_valid("fused_twist", self.fused_twist_time, self.timeout_ms["fused_twist"])
        wheel_ok   = self.is_valid("wheel_rpm", self.wheel_rpm_time, self.timeout_ms["wheel_rpm"])

        t = self.fix.header.stamp
        ts = round(t.sec + t.nanosec * 1e-9, 4)
        
        # 检查时间戳连续性
        if self.last_ts is not None:
            time_diff = ts - self.last_ts
            
            # 去重检查
            if abs(time_diff) < 0.001:  # 1ms内的重复
                return
                
            # 记录异常时间跳跃
            if time_diff < -1.0:  # 时间倒流超过1秒
                logging.warning(f"Timestamp went backwards: {self.last_ts} -> {ts} (diff: {time_diff:.3f}s)")
            elif time_diff > 10.0:  # 时间跳跃超过10秒
                logging.info(f"Large time gap detected: {time_diff:.2f}s")
        
        # 更新最后时间戳
        self.last_ts = ts
        
        # 如果是文件的第一条数据，记录文件开始时间
        if self.file_start_time is None:
            self.file_start_time = ts
            logging.info(f"File start time: {ts}")

        lin = self.twist.twist.twist.linear if twist_ok else None
        ang = self.twist.twist.twist.angular if twist_ok else None
        w = self.imu.angular_velocity if imu_ok else None
        a = self.imu.linear_acceleration if imu_ok else None
        fl = self.fused_twist.twist.twist.linear if fused_ok else None
        fa = self.fused_twist.twist.twist.angular if fused_ok else None

        self.writer.writerow([
            ts,
            self.v(fix_ok, self.fix.latitude),
            self.v(fix_ok, self.fix.longitude),
            self.v(fix_ok, self.fix.altitude),
            self.v(heading_ok, self.heading),

            self.v(twist_ok, lin.x if lin else None),
            self.v(twist_ok, lin.y if lin else None),
            self.v(twist_ok, lin.z if lin else None),

            self.v(imu_ok, w.x if w else None),
            self.v(imu_ok, w.y if w else None),
            self.v(imu_ok, w.z if w else None),

            self.v(imu_ok, a.x if a else None),
            self.v(imu_ok, a.y if a else None),
            self.v(imu_ok, a.z if a else None),

            self.v(wheel_ok, self.wheel_rpm_lf),
            self.v(wheel_ok, self.wheel_rpm_rf),
            self.v(wheel_ok, self.wheel_rpm_lr),
            self.v(wheel_ok, self.wheel_rpm_rr),

            self.v(twist_ok, ang.x if ang else None),
            self.v(twist_ok, ang.y if ang else None),
            self.v(twist_ok, ang.z if ang else None),

            self.v(fused_ok, fl.x if fl else None),
            self.v(fused_ok, fl.y if fl else None),
            self.v(fused_ok, fl.z if fl else None),
            self.v(fused_ok, fa.x if fa else None),
            self.v(fused_ok, fa.y if fa else None),
            self.v(fused_ok, fa.z if fa else None),
        ])


# ----------------- 主程序 -----------------
def process_mcap_file(node, mcap_file):
    """处理单个MCAP文件"""
    try:
        print(f"  Opening MCAP: {os.path.basename(mcap_file)}")
        
        # 初始化MCAP读取器
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=mcap_file, storage_id="mcap"),
            rosbag2_py.ConverterOptions("", "")
        )

        # 获取topic类型信息
        topic_types = {}
        try:
            topics = reader.get_all_topics_and_types()
            topic_types = {
                t.name: get_message(t.type)
                for t in topics
            }
        except Exception as e:
            print(f"  Warning: Failed to get topic types: {e}")
            logging.warning(f"Failed to get topic types for {mcap_file}: {e}")

        # 初始化CSV（内部会重置所有状态）
        node.init_csv(mcap_file)

        # 处理消息
        msg_count = 0
        fix_count = 0
        
        while reader.has_next():
            try:
                topic, data, t_ns = reader.read_next()
                msg = deserialize_message(data, topic_types.get(topic))
                t = Time(nanoseconds=t_ns)
                node.now_time = t

                # 分发消息到对应的回调函数
                if topic == "/sensing/gnss/fix":
                    node.fix_cb(msg, t)
                    node.timer_write_csv()  # 只在有fix消息时写CSV
                    fix_count += 1
                elif topic == "/sensing/gnss/heading":
                    node.heading_cb(msg, t)
                elif topic == "/sensing/gnss/imu":
                    node.imu_cb(msg, t)
                elif topic == "/pix_robobus/va_chassis_wheel_rpm_fb":
                    node.wheel_rpm_cb(msg, t)
                elif topic == "/localization/twist_estimator/twist_with_covariance":
                    node.twist_cb(msg, t)
                elif topic == "/sensing/vehicle_velocity_converter/twist_with_covariance":
                    node.fused_twist_cb(msg, t)
                
                msg_count += 1
                
                # 进度显示
                if msg_count % 10000 == 0:
                    print(f"    Processed {msg_count} messages...")
                    
            except Exception as e:
                logging.error(f"Error processing message {msg_count} in {mcap_file}: {e}")
                continue

        # 关闭当前文件的CSV
        node.close_csv()
        
        print(f"  Finished: {msg_count} total messages, {fix_count} fix messages")
        logging.info(f"Finished processing {mcap_file}: {msg_count} total messages, {fix_count} fix messages")
        
    except Exception as e:
        print(f"  Error processing {mcap_file}: {e}")
        logging.error(f"Error processing {mcap_file}: {e}")
        # 确保关闭CSV文件
        try:
            node.close_csv()
        except:
            pass


def main():
    """主函数"""
    print("=" * 60)
    print("Localization CSV Exporter from MCAP files")
    print("=" * 60)
    
    # 初始化ROS2节点
    rclpy.init()
    
    # 创建输出目录
    output_dir = "/media/ipc/AQLoopCloseData1/perception_csv"
    node = LocalizationCSVExporter(outdir=output_dir)
    
    print(f"Output directory: {output_dir}")
    
    # ----------------- 参数解析 -----------------
    if len(sys.argv) > 1:
        # 支持传入多个 MCAP 文件或目录
        mcap_files = []
        for arg in sys.argv[1:]:
            if os.path.isdir(arg):
                # 递归查找所有MCAP文件
                found = glob.glob(os.path.join(arg, "**", "*.mcap"), recursive=True)
                if not found:
                    found = glob.glob(os.path.join(arg, "*.mcap"))
                if found:
                    mcap_files.extend(found)
                    print(f"Found {len(found)} MCAP files in directory: {arg}")
                else:
                    print(f"No MCAP files found in directory: {arg}")
            elif os.path.isfile(arg) and arg.endswith('.mcap'):
                mcap_files.append(arg)
                print(f"Added MCAP file: {arg}")
            else:
                print(f"Warning: Invalid argument (not a directory or .mcap file): {arg}")
    else:
        # 默认扫描目录
        print(f"Using default directory: {DEFAULT_MCAP_DIR}")
        mcap_files = glob.glob(os.path.join(DEFAULT_MCAP_DIR, "*.mcap"))
        if not mcap_files:
            # 尝试递归查找
            mcap_files = glob.glob(os.path.join(DEFAULT_MCAP_DIR, "**", "*.mcap"), recursive=True)

    # 检查是否有文件
    if not mcap_files:
        print("No MCAP files found.")
        print("Usage:")
        print("  python3 script.py [mcap_file1] [mcap_file2] ...")
        print("  python3 script.py [directory_containing_mcap_files]")
        print("  python3 script.py  (uses default directory)")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 按文件名排序（通常按时间顺序）
    mcap_files = sorted(mcap_files)
    print(f"\nFound {len(mcap_files)} MCAP files to process:")
    for i, f in enumerate(mcap_files, 1):
        print(f"  {i:3d}. {os.path.basename(f)}")
    print()

    # ----------------- 处理每个 MCAP -----------------
    total_files = len(mcap_files)
    for i, f in enumerate(mcap_files, 1):
        print(f"\n[{i}/{total_files}] Processing {os.path.basename(f)} ...")
        logging.info(f"=== Start processing {os.path.basename(f)} ({i}/{total_files}) ===")
        
        process_mcap_file(node, f)
        
        print(f"[{i}/{total_files}] Completed {os.path.basename(f)}")
        logging.info(f"=== Finished processing {os.path.basename(f)} ===")

    # 清理
    node.destroy_node()
    rclpy.shutdown()
    
    print("\n" + "=" * 60)
    print(f"All {total_files} files processed successfully!")
    print(f"CSV files saved to: {output_dir}")
    print("=" * 60)


if __name__ == "__main__":
    main()