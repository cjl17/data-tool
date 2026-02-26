#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
from pathlib import Path
from ultralytics import YOLO
from tqdm import tqdm

# ----------------- 配置 -----------------
ROOT_DIR = "/media/ipc/AQLoopCloseData/first_20260210100908"
OUTPUT_CSV = os.path.join(ROOT_DIR, "perception_low_quality_report.csv")
THRESHOLD_OBJECTS = 3  # 低物体帧判定阈值
TARGET_CLASSES = ["car", "truck", "bus", "person", "tree", "building", "traffic light", "fence", "stop sign"]  # YOLO默认类别对应名称
DEVICE = "cuda"  # 或 "cpu"
# ----------------------------------------

model = YOLO("yolov8n.pt")  # 默认模型

def scan_images(folder_path):
    """递归扫描文件夹下所有摄像头图片"""
    image_paths = []
    folder_path = Path(folder_path)
    for cam_dir in folder_path.iterdir():
        if cam_dir.is_dir():
            for img_file in cam_dir.glob("*.jpg"):
                # 记录相对摄像头路径
                rel_path = str(Path(cam_dir.name) / img_file.name)
                image_paths.append((img_file, rel_path))
    return image_paths

def process_perception_data(perception_path):
    """处理单个 perception_data 文件夹"""
    total_frames = 0
    low_object_frames = 0
    low_frame_images = []

    image_list = scan_images(Path(perception_path) / "raw_data/images")
    total_frames = len(image_list)

    batch_size = 32  # 可调，GPU更大可增大
    for i in tqdm(range(0, total_frames, batch_size), desc=f"Processing {Path(perception_path).name}"):
        batch = image_list[i:i+batch_size]
        img_paths = [p[0] for p in batch]
        rel_paths = [p[1] for p in batch]

        results = model.predict(source=img_paths, device=DEVICE, classes=None, verbose=False, stream=False)
        
        for res, rel_path in zip(results, rel_paths):
            # 统计目标类别数量
            count = 0
            if hasattr(res, 'boxes') and res.boxes is not None:
                for box in res.boxes:
                    cls_idx = int(box.cls[0].item()) if hasattr(box.cls[0], 'item') else int(box.cls[0])
                    cls_name = model.names.get(cls_idx, "")
                    if cls_name in TARGET_CLASSES:
                        count += 1
            if count < THRESHOLD_OBJECTS:
                low_object_frames += 1
                low_frame_images.append(rel_path)

    low_ratio = round(low_object_frames / total_frames, 2) if total_frames > 0 else 0.0
    return {
        "folder": Path(perception_path).name,
        "total_frames": total_frames,
        "low_object_frames": low_object_frames,
        "low_ratio": low_ratio,
        "low_frame_images": ";".join(low_frame_images)
    }

def main():
    perception_folders = [d for d in Path(ROOT_DIR).iterdir() if d.is_dir() and "perception_data" in d.name]
    report_data = []

    for folder in perception_folders:
        data = process_perception_data(folder)
        report_data.append(data)

    # 写 CSV
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["folder", "total_frames", "low_object_frames", "low_ratio", "low_frame_images"])
        writer.writeheader()
        for row in report_data:
            writer.writerow(row)

    print(f"报告生成完成：{OUTPUT_CSV}")

if __name__ == "__main__":
    main()
