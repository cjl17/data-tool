#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
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

DEFAULT_MCAP_DIR = "/media/ipc/AQLoopCloseData1/perception_data_20260129113410"  # 默认扫描目录

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
            "imu": 40,
            "twist": 40,
            "fused_twist": 40,
            "wheel_rpm": 40,
        }

        # ================= 控制 =================
        self.save_enabled = True
        self.lock = threading.Lock()

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

        # ================= CSV =================
        self.csvfile = None
        self.writer = None
        self.last_ts = None

        # ================= 日志 =================
        log_name = f"localization_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        logging.basicConfig(
            filename=os.path.join(self.outdir, log_name),
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
            force=True
        )

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
    def init_csv(self):
        csv_name = f"localization_all_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
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
        logging.info(f"CSV initialized: {path}")

    def close_csv(self):
        if self.csvfile:
            self.csvfile.close()
            self.csvfile = None
            self.writer = None

    # ----------------- 写 CSV -----------------
    def timer_write_csv(self, mcap_file):
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
        if ts == self.last_ts:
            return
        self.last_ts = ts

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
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=mcap_file, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", "")
    )

    topic_types = {
        t.name: get_message(t.type)
        for t in reader.get_all_topics_and_types()
    }

    logging.info(f"Processing MCAP: {mcap_file}")
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, topic_types[topic])
        t = Time(nanoseconds=t_ns)
        node.now_time = t

        if topic == "/sensing/gnss/fix":
            node.fix_cb(msg, t)
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

        node.timer_write_csv(mcap_file)


def main():
    rclpy.init()
    node = LocalizationCSVExporter(outdir="/media/ipc/AQLoopCloseData1/perception_csv")

    # 初始化 CSV
    node.init_csv()

    # ----------------- 参数解析 -----------------
    if len(sys.argv) > 1:
        # 支持传入多个 MCAP 文件
        mcap_files = []
        for arg in sys.argv[1:]:
            if os.path.isdir(arg):
                mcap_files.extend(glob.glob(os.path.join(arg, "*.mcap")))
            elif os.path.isfile(arg):
                mcap_files.append(arg)
    else:
        # 默认扫描目录
        mcap_files = glob.glob(os.path.join(DEFAULT_MCAP_DIR, "*.mcap"))

    if not mcap_files:
        print("No MCAP files found.")
        return

    # ----------------- 处理每个 MCAP -----------------
    for f in sorted(mcap_files):
        print(f"Processing {f} ...")
        process_mcap_file(node, f)

    node.close_csv()
    node.destroy_node()
    rclpy.shutdown()
    print("All done.")


if __name__ == "__main__":
    main()
