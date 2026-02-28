#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
from pathlib import Path
from collections import defaultdict
from ultralytics import YOLO
from tqdm import tqdm
import torch



print("CUDA available:", torch.cuda.is_available())
print("CUDA device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU")

# ================= 配置 =================
ROOT_DIR = "/media/pix/AQLoopCloseData"
OUTPUT_CSV = os.path.join(ROOT_DIR, "class_ratio_report.csv")
MODEL_PATH = "yolov8s.pt"
DEVICE = "cuda"
BATCH_SIZE = 120

# 只统计这些类别
TARGET_CLASSES = [
    "car",
    "truck",
    "bus",
    "person",
    "bicycle",
    "motorcycle"
]
# =======================================

# 加载模型
model = YOLO(MODEL_PATH)
CLASS_NAMES = model.names  # {0:'person',1:'bicycle',...}

# 生成 name -> id 映射
NAME_TO_ID = {v: k for k, v in CLASS_NAMES.items()}

# 目标类别ID集合（用于快速过滤）
TARGET_CLASS_IDS = {NAME_TO_ID[name] for name in TARGET_CLASSES if name in NAME_TO_ID}


def get_time_aligned_images(images_root):
    """
    返回按时间对齐的图片列表
    [
        [t0_cam1, t0_cam2, ...],
        [t1_cam1, t1_cam2, ...],
        ...
    ]
    """
    cam_dirs = [d for d in Path(images_root).iterdir() if d.is_dir()]
    if not cam_dirs:
        return []

    cam_dirs.sort()

    cam_images = []
    for cam in cam_dirs:
        imgs = sorted(cam.glob("*.jpg"))
        if len(imgs) == 0:
            return []
        cam_images.append(imgs)

    min_len = min(len(x) for x in cam_images)

    aligned = []
    for i in range(min_len):
        frame_imgs = [cam_images[c][i] for c in range(len(cam_images))]
        aligned.append(frame_imgs)

    return aligned


def process_perception(perception_path):
    images_root = Path(perception_path) / "raw_data/images"
    if not images_root.exists():
        return None

    aligned_frames = get_time_aligned_images(images_root)
    if not aligned_frames:
        return None

    total_frames = len(aligned_frames)

    # 每类出现帧计数（只统计目标类）
    class_frame_count = {name: 0 for name in TARGET_CLASSES}

    # 展平图片
    all_images = []
    frame_index_map = []

    for frame_idx, imgs in enumerate(aligned_frames):
        for img in imgs:
            all_images.append(img)
            frame_index_map.append(frame_idx)

    # 每帧检测到的类别（避免重复计数）
    frame_detected_classes = [set() for _ in range(total_frames)]

    # ================= 批量推理 =================
    for i in tqdm(range(0, len(all_images), BATCH_SIZE),
                  desc=f"{Path(perception_path).name}",
                  leave=False):

        batch_imgs = all_images[i:i+BATCH_SIZE]
        batch_frames = frame_index_map[i:i+BATCH_SIZE]

        results = model.predict(
            source=batch_imgs,
            device=DEVICE,
            verbose=False,
            stream=False
        )

        for res, frame_idx in zip(results, batch_frames):
            if res.boxes is None:
                continue

            classes = res.boxes.cls.tolist()

            for cls_id in classes:
                cls_id = int(cls_id)

                # 只保留目标类别
                if cls_id in TARGET_CLASS_IDS:
                    frame_detected_classes[frame_idx].add(cls_id)

    # ================= 统计帧占比 =================
    for frame_classes in frame_detected_classes:
        for cls_id in frame_classes:
            class_name = CLASS_NAMES[cls_id]
            class_frame_count[class_name] += 1

    # 转为比例
    class_ratios = {}
    for name in TARGET_CLASSES:
        class_ratios[name] = round(class_frame_count[name] / total_frames, 3)

    return total_frames, class_ratios


def main():
    rows = []

    first_dirs = [d for d in Path(ROOT_DIR).iterdir()
                  if d.is_dir() and d.name.startswith("first")]

    for first in first_dirs:
        perception_dirs = [d for d in first.iterdir()
                           if d.is_dir() and "perception_data" in d.name]

        for perception in perception_dirs:
            print(f"Processing {perception}")

            result = process_perception(perception)
            if result is None:
                continue

            total_frames, ratios = result

            row = {
                "first_folder": first.name,
                "perception_folder": perception.name,
                "total_frames": total_frames
            }
            row.update(ratios)
            rows.append(row)

    # ================= 写CSV =================
    fieldnames = [
        "first_folder",
        "perception_folder",
        "total_frames"
    ] + TARGET_CLASSES

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)

    print(f"\n完成！输出文件: {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
