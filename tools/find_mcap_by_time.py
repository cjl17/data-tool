#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
按时间查找对应的 mcap 文件（仅定位，不提取数据）

功能：
- 输入时间字符串
- 自动建立时间索引（首次）
- 快速定位包含该时间的 mcap
- 支持前后扩展 N 个分片
"""

import argparse
import json
import re
import subprocess
from datetime import datetime
from pathlib import Path


INDEX_FILE = "mcap_time_index.json"


def parse_ros2_time(mcap: Path):
    """调用 ros2 bag info 获取时间范围（兼容纳秒精度）"""
    cmd = ["ros2", "bag", "info", str(mcap)]
    out = subprocess.check_output(cmd, text=True)

    start_match = re.search(r"Start:\s+(.*)\(", out)
    end_match = re.search(r"End:\s+(.*)\(", out)

    if not start_match:
        return None

    def fix_time_str(tstr):
        """
        把纳秒截断为微秒：
        2026 10:09:08.123456916
        -> 2026 10:09:08.123456
        """
        tstr = tstr.strip()
        if "." in tstr:
            head, frac = tstr.split(".")
            frac = frac[:6]  # 只保留微秒
            tstr = f"{head}.{frac}"
        return tstr

    fmt = "%b %d %Y %H:%M:%S.%f"

    start_str = fix_time_str(start_match.group(1))
    end_str = fix_time_str(end_match.group(1))

    start = datetime.strptime(start_str, fmt)
    end = datetime.strptime(end_str, fmt)

    return start.timestamp(), end.timestamp()



def build_index(dir_path: Path, index_path: Path):
    print("[INFO] Building index... (only first time)")

    index = []
    files = sorted(dir_path.glob("*.mcap"))

    for i, f in enumerate(files):
        print(f"  {i+1}/{len(files)} {f.name}")
        t = parse_ros2_time(f)
        if t:
            index.append({
                "file": f.name,
                "start": t[0],
                "end": t[1]
            })

    with open(index_path, "w") as fp:
        json.dump(index, fp, indent=2)

    print("[INFO] Index saved:", index_path)


def load_index(dir_path: Path):
    index_path = dir_path / INDEX_FILE
    if not index_path.exists():
        build_index(dir_path, index_path)

    with open(index_path) as fp:
        return json.load(fp)


def find_index(index, ts):
    for i, item in enumerate(index):
        if item["start"] <= ts <= item["end"]:
            return i
    return None


def main():
    parser = argparse.ArgumentParser(description="按时间查找 mcap")
    parser.add_argument("dir", type=Path, help="perception_data_* 目录")
    parser.add_argument("time", type=str, help="时间: 2026-02-10 09:57:13")
    parser.add_argument("--expand", type=int, default=0,
                        help="前后扩展分片数量")
    parser.add_argument("--print-full", action="store_true",
                        help="打印完整路径")

    args = parser.parse_args()

    dir_path = args.dir.resolve()
    if not dir_path.exists():
        print("[ERROR] directory not found")
        return

    # 时间转换
    query_dt = datetime.strptime(args.time, "%Y-%m-%d %H:%M:%S")
    query_ts = query_dt.timestamp()

    # 加载索引
    index = load_index(dir_path)

    # 查找
    idx = find_index(index, query_ts)
    if idx is None:
        print("[ERROR] No mcap contains this time")
        return

    # 扩展范围
    start_idx = max(0, idx - args.expand)
    end_idx = min(len(index) - 1, idx + args.expand)

    selected = index[start_idx:end_idx + 1]

    print("\n[RESULT]")
    for item in selected:
        path = dir_path / item["file"]
        if args.print_full:
            print(path)
        else:
            print(item["file"])


if __name__ == "__main__":
    main()
