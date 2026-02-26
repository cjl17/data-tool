#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import os
import sys
import math
import random
import argparse
from pathlib import Path
from typing import List

import cv2
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation as R
import yaml

try:
    import open3d as o3d
    _O3D_AVAILABLE = True
except Exception as _e:
    print("[WARN] Open3D import failed:", _e)
    _O3D_AVAILABLE = False


# ==============================
# 全局配置
# ==============================

# 时间补偿
dt = 0.0

# 是否生成所有帧（建议调试时设为 False）
all_gen = True

CAM_OPTIONS = [
    "front_3mm",
    "rear_3mm",
    "front_left",
    "front_right",
    "rear_left",
    "rear_right"
]

camera_name_map = {
    'front': 'camera1',
    'rear': 'camera2',
    'front_left': 'camera3',
    'front_right': 'camera4',
    'rear_left': 'camera5',
    'rear_right': 'camera6',
    'front_3mm': 'camera1',
    'rear_3mm': 'camera2',
}

cam_alias = {
    'CAM_FRONT': 'front',
    'CAM_BACK': 'rear',
    'CAM_FRONT_LEFT': 'front_left',
    'CAM_FRONT_RIGHT': 'front_right',
    'CAM_BACK_LEFT': 'rear_left',
    'CAM_BACK_RIGHT': 'rear_right',
}


# ==============================
# 工具函数
# ==============================

def read_pcd(pcd_file):
    if _O3D_AVAILABLE:
        try:
            pcd = o3d.t.io.read_point_cloud(pcd_file)
            pos = pcd.point['positions'].numpy()
            if 'intensity' in pcd.point:
                inten = pcd.point['intensity'].numpy()
            else:
                inten = np.linalg.norm(pos, axis=1, keepdims=True)
            return np.concatenate([pos, inten], axis=1)
        except:
            pass

    try:
        pcd = o3d.io.read_point_cloud(pcd_file)
        pts = np.asarray(pcd.points)
        if pts.size == 0:
            return np.zeros((0, 4))
        inten = np.linalg.norm(pts, axis=1, keepdims=True)
        return np.concatenate([pts, inten], axis=1)
    except:
        return np.zeros((0, 4))


def project(points, image, M1, M2):
    if len(points) == 0:
        return np.zeros((0, 4))

    coords = points[:, :3]
    ones = np.ones((len(coords), 1))
    coords = np.concatenate([coords, ones], axis=1)

    transform = M1 @ M2
    coords = coords @ transform.T

    valid = coords[:, 2] > 0
    coords = coords[valid]
    points = points[valid]

    coords[:, 0] /= coords[:, 2]
    coords[:, 1] /= coords[:, 2]

    h, w = image.shape[:2]
    mask = (
        (coords[:, 0] >= 0) &
        (coords[:, 0] < w) &
        (coords[:, 1] >= 0) &
        (coords[:, 1] < h)
    )

    coords = coords[mask]
    intensity = points[mask, -1]

    return np.concatenate([coords[:, :2], intensity.reshape(-1, 1)], axis=1)


def jet_color(v, vmin=1, vmax=255):
    t = (v - vmin) / (vmax - vmin)
    t = max(0, min(1, t))

    if t < 0.25:
        return (255, int(4*t*255), 0)
    elif t < 0.5:
        return (int((1-4*(t-0.25))*255), 255, 0)
    elif t < 0.75:
        return (0, 255, int(4*(t-0.5)*255))
    else:
        return (0, int((1-4*(t-0.75))*255), 255)


def draw_projection(image, coords, save_path):
    canvas = image.copy()
    canvas = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)

    for x, y, inten in coords:
        cv2.circle(canvas, (int(x), int(y)), 1, jet_color(inten), 1)

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)).save(save_path)


# ==============================
# 标定
# ==============================

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def resolve_calibration(data_root, camera_key, calib_root=None):
    candidates = []
    if calib_root:
        candidates.append(calib_root)
    candidates.append(os.path.join(data_root, "parameter"))

    for root in candidates:
        if not root or not os.path.isdir(root):
            continue

        base = os.path.join(root, "sensor_kit/robobus_sensor_kit_description")

        sensor_kit = os.path.join(base, "extrinsic_parameters/sensor_kit_calibration.yaml")
        sensor_base = os.path.join(base, "extrinsic_parameters/sensors_calibration.yaml")
        intrinsic = os.path.join(base, f"intrinsic_parameters/{camera_name_map[camera_key]}_params.yaml")

        if all(os.path.isfile(p) for p in [sensor_kit, sensor_base, intrinsic]):
            return sensor_kit, sensor_base, intrinsic

    return None, None, None


def get_M(sensor_kit_data, sensor_base_data, cam_data, camera_key):
    K = np.array(cam_data['camera_matrix']['data']).reshape(3, 3)
    M1 = np.eye(4)
    M1[:3, :3] = K

    frame = f"{camera_name_map[camera_key]}/camera_link"
    sk = sensor_kit_data['sensor_kit_base_link'][frame]
    rot = R.from_euler('xyz', [sk['roll'], sk['pitch'], sk['yaw']]).as_matrix()

    T1 = np.eye(4)
    T1[:3, :3] = rot
    T1[:3, 3] = [sk['x'], sk['y'], sk['z']]

    bs = sensor_base_data['base_link']['sensor_kit_base_link']
    rot2 = R.from_euler('xyz', [bs['roll'], bs['pitch'], bs['yaw']]).as_matrix()

    T2 = np.eye(4)
    T2[:3, :3] = rot2
    T2[:3, 3] = [bs['x'], bs['y'], bs['z']]

    M2 = np.linalg.inv(T1) @ np.linalg.inv(T2)
    return M1, M2


# ==============================
# 核心处理
# ==============================

def process_perception_dir(data_root, calib_root=None):
    ok_dir = os.path.join(data_root, "ok_data")
    if not os.path.isdir(ok_dir):
        print("[SKIP] no ok_data:", data_root)
        return

    seqs = sorted([d for d in os.listdir(ok_dir) if d.startswith("sequence")])

    for seq in seqs:
        lidar_dir = os.path.join(ok_dir, seq, "lidar")
        if not os.path.isdir(lidar_dir):
            continue

        lidar_files = sorted([f for f in os.listdir(lidar_dir) if f.endswith(".pcd")])
        if not lidar_files:
            continue

        for cam in CAM_OPTIONS:
            cam_dir = os.path.join(ok_dir, seq, cam)
            if not os.path.isdir(cam_dir):
                continue

            cam_key = cam_alias.get(cam, cam)
            if cam_key not in camera_name_map:
                continue

            sk, sb, intr = resolve_calibration(data_root, cam_key, calib_root)
            if sk is None:
                continue

            sensor_kit = load_yaml(sk)
            sensor_base = load_yaml(sb)
            cam_data = load_yaml(intr)

            M1, M2 = get_M(sensor_kit, sensor_base, cam_data, cam_key)

            img_files = sorted([f for f in os.listdir(cam_dir) if f.endswith(".jpg")])
            count = min(len(lidar_files), len(img_files))
            if not all_gen:
                count = min(count, 5)

            for i in range(count):
                lidar_path = os.path.join(lidar_dir, lidar_files[i])
                img_path = os.path.join(cam_dir, img_files[i])

                pts = read_pcd(lidar_path)
                img = np.array(Image.open(img_path).resize((1920, 1080)))

                coords = project(pts, img, M1, M2)

                save_path = os.path.join(
                    data_root,
                    "projection",
                    seq,
                    cam,
                    f"map{i:03d}.jpg"
                )
                draw_projection(img, coords, save_path)


# ==============================
# 目录工具
# ==============================

def find_first_dirs(root: Path) -> List[Path]:
    if root.name.startswith("first_"):
        return [root]
    return sorted([p for p in root.glob("first_*") if p.is_dir()])


# ==============================
# 主入口（带随机抽样）
# ==============================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root")
    parser.add_argument("--sample", type=int)
    parser.add_argument("--ratio", type=float)
    parser.add_argument("--seed", type=int)

    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)
        print("[INFO] seed =", args.seed)

    root = Path(args.root).resolve()

    # 单 perception_data
    if (root / "ok_data").is_dir():
        print("[MODE] single:", root)
        process_perception_dir(str(root))
        return

    # 批处理
    first_dirs = find_first_dirs(root)
    if not first_dirs:
        print("No first_* found")
        return

    for first_dir in first_dirs:
        calib_root = str(first_dir / "parameter")
        if not os.path.isdir(calib_root):
            calib_root = None

        print("\n====", first_dir, "====")

        perception_dirs = sorted(first_dir.glob("perception_data_*"))
        total = len(perception_dirs)

        if total == 0:
            continue

        # 随机抽样
        if args.sample:
            k = min(args.sample, total)
            perception_dirs = random.sample(perception_dirs, k)
            print(f"[SAMPLE] {k}/{total}")

        elif args.ratio:
            k = max(1, int(total * args.ratio))
            perception_dirs = random.sample(perception_dirs, k)
            print(f"[RATIO] {k}/{total}")

        else:
            print(f"[FULL] {total}")

        for p in perception_dirs:
            print("Processing:", p.name)
            process_perception_dir(str(p), calib_root)


if __name__ == "__main__":
    main()
