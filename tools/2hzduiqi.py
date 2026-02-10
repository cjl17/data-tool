#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AQLoop batch raw_data 对齐 + sequence 导出 + 2Hz 抽帧
自动查找 first_* / perception_data_*
"""

import os
import glob
import argparse
import shutil
import logging
import re
from pathlib import Path
from typing import List, Optional


# ================= 单段处理器 =================

class AQLoopPreprocessor:

    def __init__(self, raw_data_path: Path, time_threshold: float = 0.005):
        self.base_path = raw_data_path
        self.time_threshold = time_threshold

        self.camera_folders = [
            "images/front_3mm_jpeg",
            "images/rear_3mm_jpeg",
            "images/front_left_jpeg",
            "images/front_right_jpeg",
            "images/rear_left_jpeg",
            "images/rear_right_jpeg",
        ]

        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s"
        )
        self.logger = logging.getLogger("AQLoopPreprocessor")

    def extract_timestamp(self, path: str) -> float:
        name = os.path.basename(path)
        m = re.findall(r"\d+\.\d+", name)
        return float(m[-1]) if m else 0.0

    def is_2hz(self, ts: float, tol=0.01) -> bool:
        frac = ts % 1.0
        return abs(frac - 0.0) < tol or abs(frac - 0.5) < tol

    def load_files(self):
        pcd_root = self.base_path / "pcd"
        subdirs = [d for d in pcd_root.iterdir() if d.is_dir()]
        if not subdirs:
            raise RuntimeError("pcd 下没有子目录")

        lidar_dir = subdirs[0]
        self.logger.info(f"使用 LiDAR 目录: {lidar_dir.name}")

        lidar_files = sorted(
            glob.glob(str(lidar_dir / "*.pcd")),
            key=self.extract_timestamp
        )

        camera_files = []
        for cam in self.camera_folders:
            cam_dir = self.base_path / cam
            files = sorted(
                glob.glob(str(cam_dir / "*.jpg")),
                key=self.extract_timestamp
            )
            camera_files.append(files)

        return lidar_files, camera_files

    def match(self, lidar_files, camera_files):
        idxs = [0] * len(camera_files)
        matched = []

        for lidar in lidar_files:
            ts = self.extract_timestamp(lidar)
            group = [lidar]

            for i, cams in enumerate(camera_files):
                idx = idxs[i]
                while idx < len(cams) and \
                        self.extract_timestamp(cams[idx]) < ts - self.time_threshold:
                    idx += 1

                if idx < len(cams) and \
                        abs(self.extract_timestamp(cams[idx]) - ts) <= self.time_threshold:
                    group.append(cams[idx])
                    idx += 1
                else:
                    group.append(None)

                idxs[i] = idx

            if all(x is not None for x in group[1:]):
                matched.append(group)

        return matched

    def export(self, matched, out_root: Path):
        out = out_root / "ok_data"
        out2 = out_root / "ok_data_2hz"
        out.mkdir(exist_ok=True)
        out2.mkdir(exist_ok=True)

        cam_names = [
            "front_3mm", "rear_3mm",
            "front_left", "front_right",
            "rear_left", "rear_right",
        ]

        for i, g in enumerate(matched):
            seq = out / "sequence00000"
            seq2 = out2 / "sequence00000"

            for d in cam_names + ["lidar"]:
                (seq / d).mkdir(parents=True, exist_ok=True)
                (seq2 / d).mkdir(parents=True, exist_ok=True)

            ts = self.extract_timestamp(g[0])

            def link(src, dst):
                if dst.exists():
                    dst.unlink()
                os.symlink(os.path.relpath(src, dst.parent), dst)

            for name, cam in zip(cam_names, g[1:]):
                src = Path(cam)
                link(src, seq / name / src.name)
                if self.is_2hz(ts):
                    link(src, seq2 / name / src.name)

            lidar = Path(g[0])
            link(lidar, seq / "lidar" / lidar.name)
            if self.is_2hz(ts):
                link(lidar, seq2 / "lidar" / lidar.name)

    def run(self, out_root: Path):
        lidar, cams = self.load_files()
        matched = self.match(lidar, cams)
        self.logger.info(f"匹配成功: {len(matched)} 帧")
        if matched:
            self.export(matched, out_root)


# ================= 批处理入口 =================

def main():
    parser = argparse.ArgumentParser("AQLoop first_* 批处理")
    parser.add_argument("root", help="AQLoopCloseData2 根目录")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    root = Path(args.root)
    first_dirs = sorted(root.glob("first_*"))
    if not first_dirs:
        raise RuntimeError("未找到 first_* 目录")

    first = first_dirs[0]
    logging.info(f"使用 first 目录: {first.name}")

    for p in sorted(first.glob("perception_data_*")):
        raw = p / "raw_data"
        if not raw.exists():
            continue

        logging.info(f"处理: {p.name}")
        proc = AQLoopPreprocessor(raw)
        proc.run(p)


if __name__ == "__main__":
    main()
