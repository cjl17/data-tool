#!/usr/bin/env python3
import sqlite3
import sys
import os
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml
import bisect
import math

# ---------------- 可修改参数 ----------------
DATA_PER_FOLDER = 200  # 每个文件夹包含多少条数据
# ------------------------------------------

def quaternion_to_yaw(q):
    """
    从四元数转换为航向角（yaw）
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def rotate_to_map(x_car, y_car, yaw):
    """
    将自车坐标系下的速度/加速度数据转换到 MAP 坐标系
    """
    x_map = math.cos(yaw) * x_car - math.sin(yaw) * y_car
    y_map = math.sin(yaw) * x_car + math.cos(yaw) * y_car
    return x_map, y_map

def main(db3_file):
    if not os.path.exists(db3_file):
        print(f"文件不存在: {db3_file}")
        sys.exit(1)

    # 输出路径固定为输入文件夹下 localization
    db3_path = Path(db3_file).parent
    outdir = db3_path / "localization"
    outdir.mkdir(exist_ok=True)

    rclpy.init()
    conn = sqlite3.connect(db3_file)
    cursor = conn.cursor()

    # 获取 topic id
    cursor.execute("SELECT id, name FROM topics")
    topics = {name: id for id, name in cursor.fetchall()}

    need_topics = {
        "pose_position": "/sensing/gnss/fix",  # 用于获取位置（经纬度）
        "pose": "/sensing/gnss/pose_with_covariance",  # 用于获取方向（四元数）
        "twist": "/localization/twist_estimator/twist_with_covariance",
        "imu": "/sensing/imu/imu_data"
    }

    for key in ["pose", "twist", "pose_position"]:
        if need_topics[key] not in topics:
            print(f"缺少topic: {need_topics[key]}")
            sys.exit(1)

    imu_exists = need_topics["imu"] in topics

    # 消息类型
    msg_types = {
        "pose_position": "sensor_msgs/msg/NavSatFix",  # 位置数据（经纬度）
        "pose": "geometry_msgs/msg/PoseWithCovarianceStamped",  # 方向数据（四元数）
        "twist": "geometry_msgs/msg/TwistWithCovarianceStamped",
        "imu": "sensor_msgs/msg/Imu"
    }
    msg_classes = {k: get_message(v) for k, v in msg_types.items()}

    # 读取并排序
    all_data = {"pose": [], "twist": [], "imu": [], "pose_position": []}
    for key in all_data.keys():
        if key not in need_topics or need_topics[key] not in topics:
            continue
        tid = topics[need_topics[key]]
        for row in cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={tid}"):
            ts = row[0] / 1e9
            msg = deserialize_message(row[1], msg_classes[key])
            all_data[key].append((ts, msg))
        all_data[key].sort(key=lambda x: x[0])

    # 二分查找最近时间戳
    def find_closest(data_list, target_ts):
        if not data_list:
            return None
        timestamps = [ts for ts, _ in data_list]
        pos = bisect.bisect_left(timestamps, target_ts)
        if pos == 0:
            return data_list[0][1]
        if pos >= len(data_list):
            return data_list[-1][1]
        before_ts, before_msg = data_list[pos-1]
        after_ts, after_msg = data_list[pos]
        if abs(before_ts - target_ts) <= abs(after_ts - target_ts):
            return before_msg
        else:
            return after_msg

    # 输出 YAML（10 Hz）
    if not all_data["pose"]:
        print("❌ pose 数据为空，无法生成 YAML 文件")
        rclpy.shutdown()
        sys.exit(1)

    start_ts = all_data["pose"][0][0]
    end_ts = all_data["pose"][-1][0]
    freq = 10.0  # Hz
    dt = 1.0 / freq

    count = 0
    folder_idx = 0
    ts = start_ts

    # 创建第一个文件夹
    current_folder = outdir / f"{folder_idx}"
    current_folder.mkdir(exist_ok=True)

    while ts <= end_ts:
        # 每 DATA_PER_FOLDER 条数据创建一个新文件夹
        if count > 0 and count % DATA_PER_FOLDER == 0:
            folder_idx += 1
            current_folder = outdir / f"{folder_idx}"
            current_folder.mkdir(exist_ok=True)

        # 获取GNSS数据
        position_msg = find_closest(all_data["pose_position"], ts)
        latitude = position_msg.latitude
        longitude = position_msg.longitude
        altitude = position_msg.altitude

        # 获取pose数据（方向）
        pose_msg = find_closest(all_data["pose"], ts)
        orientation = pose_msg.pose.pose.orientation

        # 获取IMU数据
        imu_msg = find_closest(all_data["imu"], ts) if imu_exists else None
        angularV = {"x": 0.0, "y": 0.0, "z": 0.0}
        if imu_msg:
            angularV = {
                "x": float(imu_msg.angular_velocity.x),
                "y": float(imu_msg.angular_velocity.y),
                "z": float(imu_msg.angular_velocity.z)
            }

        # Twist（速度与加速度）
        twist_msg = find_closest(all_data["twist"], ts)
        vel = twist_msg.twist.twist.linear if twist_msg else None
        acc = twist_msg.twist.twist.angular if twist_msg else None

        # 计算航向角（yaw）
        yaw = quaternion_to_yaw(orientation)

        # 将速度和加速度从自车坐标系转换到 MAP 坐标系
        vel_x_map, vel_y_map = rotate_to_map(vel.x if vel else 0.0, vel.y if vel else 0.0, yaw)
        acc_x_map, acc_y_map = rotate_to_map(acc.x if acc else 0.0, acc.y if acc else 0.0, yaw)

        # Covariance 转 float 列表
        posCov = [float(x) for x in pose_msg.pose.covariance] if hasattr(pose_msg.pose, "covariance") else [0.0]*36
        velCov = [float(x) for x in twist_msg.twist.covariance] if twist_msg and hasattr(twist_msg.twist, "covariance") else [0.0]*9

        entry = {
            "header": {"frameId": "vehicle-lidar", "timestampSec": ts},
            "pose": {
                "orientation": {"x": float(orientation.x), "y": float(orientation.y), "z": float(orientation.z), "w": float(orientation.w)},
                "position": {"x": float(longitude), "y": float(latitude), "z": float(altitude)}
            },
            "vel": {"x": vel_x_map, "y": vel_y_map, "z": float(vel.z) if vel else 0.0},
            "acc": {"x": acc_x_map, "y": acc_y_map, "z": float(acc.z) if acc else 0.0},
            "angularV": angularV,
            "consistencyToMap": 1.0,
            "mode": 1,
            "poseConfidence": 1.0,
            "status": 2,
            "worldFrame": "WGS84",
            "posCov": posCov,
            "velCov": velCov
        }

        with open(current_folder / f"{int(ts*1e2)}.yaml", "w") as f:
            yaml.dump(entry, f, sort_keys=False)

        count += 1
        ts += dt

    print(f"✅ 已生成 {count} 个 yaml 文件，分布在 {folder_idx+1} 个文件夹中，保存在 {outdir}/")
    rclpy.shutdown()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: ./123.py your.db3")
        sys.exit(2)
    main(sys.argv[1])

