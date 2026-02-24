#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AQLoop batch raw_data 对齐 + sequence 导出 + 2Hz 抽帧
自动查找 first_* / perception_data_*
修复版：处理所有first_*目录
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
        """从文件名提取时间戳（毫秒或纳秒），转换为秒"""
        name = os.path.basename(path)
        # 先尝试匹配带小数点的格式（如 123.456）
        m = re.findall(r"\d+\.\d+", name)
        if m:
            return float(m[-1])
        
        # 如果没有小数点，提取所有数字（可能是毫秒或纳秒时间戳）
        # 文件名格式通常是：时间戳.扩展名，如 1770689760099.pcd
        m = re.findall(r"(\d+)", name)
        if m:
            ts_int = int(m[0])  # 取第一个数字串
            # 判断是毫秒还是纳秒：
            # - 13位数字通常是毫秒时间戳（如 1770689760099）
            # - 19位数字通常是纳秒时间戳
            ts_str = str(ts_int)
            if len(ts_str) >= 19:
                return ts_int / 1e9  # 纳秒转秒
            elif len(ts_str) >= 13:
                return ts_int / 1e3  # 毫秒转秒
            else:
                # 小于13位，可能是秒级时间戳，直接返回
                return float(ts_int)
        
        return 0.0

    def is_2hz(self, ts: float, base_ts: float, tol=0.01) -> bool:
        """
        判断时间戳是否在 2Hz 位置（每 0.5 秒一帧）
        基于相对时间戳（相对于第一个时间戳）
        """
        relative_ts = ts - base_ts
        # 2Hz = 每 0.5 秒一帧，判断相对时间是否是 0.5 的倍数（允许小误差）
        remainder = relative_ts % 0.5
        return remainder < tol or (0.5 - remainder) < tol

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

        # 计算基准时间戳（第一个匹配帧的时间戳）
        base_ts = self.extract_timestamp(matched[0][0]) if matched else 0.0

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
                if self.is_2hz(ts, base_ts):
                    link(src, seq2 / name / src.name)

            lidar = Path(g[0])
            link(lidar, seq / "lidar" / lidar.name)
            if self.is_2hz(ts, base_ts):
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

    # ============ 修复：遍历所有first_*目录，而不是只取第一个 ============
    for first_dir in first_dirs:
        logging.info(f"处理 first 目录: {first_dir.name}")
        
        for p in sorted(first_dir.glob("perception_data_*")):
            raw = p / "raw_data"
            if not raw.exists():
                continue

            logging.info(f"  处理 perception_data: {p.name}")
            try:
                proc = AQLoopPreprocessor(raw)
                proc.run(p)
                logging.info(f"  完成: {p.name}")
            except Exception as e:
                logging.error(f"  处理失败 {p.name}: {e}")


if __name__ == "__main__":
    main()