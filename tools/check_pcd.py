#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


# =========================
# Filename parsing
# =========================

FNAME_RE_WITH_IDX = re.compile(r"^(?P<ts>\d+)_(?P<idx>\d+)\.(?P<ext>[A-Za-z0-9]+)$")
FNAME_RE_NO_IDX = re.compile(r"^(?P<ts>\d+)\.(?P<ext>[A-Za-z0-9]+)$")


@dataclass(frozen=True)
class Frame:
    ts_ms: int
    idx: Optional[int]
    path: Path


# =========================
# File iteration（未改）
# =========================

def _iter_files(export_dir: Path) -> Iterable[Path]:
    images = export_dir / "images"
    pcd = export_dir / "pcd"

    if images.is_dir():
        yield from images.glob("*/*")

    if pcd.is_dir():
        yield from pcd.glob("*/*.pcd")

    for subdir in export_dir.iterdir():
        if not subdir.is_dir():
            continue
        nested_images = subdir / "images"
        nested_pcd = subdir / "pcd"
        if nested_images.is_dir():
            yield from nested_images.glob("*/*")
        if nested_pcd.is_dir():
            yield from nested_pcd.glob("*/*.pcd")


def _parse_frame(p: Path) -> Optional[Frame]:
    m = FNAME_RE_WITH_IDX.match(p.name)
    if m:
        try:
            return Frame(int(m.group("ts")), int(m.group("idx")), p)
        except ValueError:
            return None

    m = FNAME_RE_NO_IDX.match(p.name)
    if m:
        try:
            return Frame(int(m.group("ts")), None, p)
        except ValueError:
            return None

    return None


def _median_int(values: list[int]) -> Optional[int]:
    if not values:
        return None
    return int(statistics.median(values))


# =========================
# Core analyze（完全未改）
# =========================

def analyze_folder(folder: Path, gap_factor: float, expected_dt_ms: Optional[int]) -> dict:
    frames: list[Frame] = []
    for p in sorted(folder.iterdir()):
        if not p.is_file():
            continue
        fr = _parse_frame(p)
        if fr:
            frames.append(fr)

    frames.sort(key=lambda f: (f.ts_ms, f.idx if f.idx is not None else 0))
    n = len(frames)

    if n < 2:
        return {
            "count": n,
            "estimated_dt_ms": None,
            "estimated_fps": None,
            "non_monotonic_ts": 0,
            "index_gaps": 0,
            "large_ts_gaps": 0,
        }

    non_mono = 0
    idx_gaps = 0
    dts = []

    for a, b in zip(frames, frames[1:]):
        dt = b.ts_ms - a.ts_ms
        dts.append(dt)
        if dt < 0:
            non_mono += 1

        if a.idx is not None and b.idx is not None:
            d_idx = b.idx - a.idx
            if d_idx > 1:
                idx_gaps += (d_idx - 1)

    med_dt = expected_dt_ms if expected_dt_ms else _median_int([dt for dt in dts if dt >= 0])
    est_fps = (1000.0 / med_dt) if med_dt and med_dt > 0 else None

    large_gaps = 0
    if med_dt and med_dt > 0:
        threshold = int(med_dt * gap_factor)
        for a, b in zip(frames, frames[1:]):
            if (b.ts_ms - a.ts_ms) > threshold:
                large_gaps += 1

    return {
        "count": n,
        "estimated_dt_ms": med_dt,
        "estimated_fps": est_fps,
        "non_monotonic_ts": non_mono,
        "index_gaps": idx_gaps,
        "large_ts_gaps": large_gaps,
    }


# =========================
# Find perception_data
# =========================

def find_raw_data_dirs(first_dir: Path) -> list[Path]:
    raw_dirs = []

    for p in sorted(first_dir.iterdir()):
        if p.is_dir() and p.name.startswith("perception_data_"):
            rd = p / "raw_data"
            if rd.is_dir():
                raw_dirs.append(rd)
            else:
                print(f"[WARN] no raw_data under {p}")

    if not raw_dirs:
        print(f"[ERROR] No perception_data_*/raw_data found under {first_dir}")
        sys.exit(2)

    return raw_dirs


# =========================
# Main（批量 + 异常过滤）
# =========================

def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Batch check perception_data under a first_xxx directory.")
    parser.add_argument("first_dir", type=Path)
    parser.add_argument("--gap-factor", type=float, default=3.0)
    parser.add_argument("--expected-dt-ms", type=int, default=None)

    args = parser.parse_args(argv)

    first_dir = args.first_dir.resolve()
    if not first_dir.exists():
        print(f"[ERROR] directory does not exist: {first_dir}")
        return 2

    raw_dirs = find_raw_data_dirs(first_dir)

    log_file = first_dir / "check_report.txt"
    log_fp = log_file.open("w", encoding="utf-8")

    total_count = 0
    good_count = 0
    bad_count = 0

    for raw_dir in raw_dirs:
        total_count += 1
        perception_name = raw_dir.parent.name

        groups: dict[Path, list[Path]] = {}
        for f in _iter_files(raw_dir):
            if f.is_file():
                groups.setdefault(f.parent, []).append(f)

        if not groups:
            msg = f"[WARN] {perception_name} no image/pcd files\n"
            print(msg.strip())
            log_fp.write(msg)
            bad_count += 1
            continue

        perception_has_issue = False
        summary_lines = []

        for folder in sorted(groups.keys()):
            r = analyze_folder(folder, args.gap_factor, args.expected_dt_ms)

            issues = (
                r["non_monotonic_ts"] > 0
                or r["index_gaps"] > 0
                or r["large_ts_gaps"] > 0
            )

            if issues:
                perception_has_issue = True

            fps_txt = f"{r['estimated_fps']:.2f}" if r["estimated_fps"] else "n/a"
            dt_txt = str(r["estimated_dt_ms"]) if r["estimated_dt_ms"] else "n/a"

            summary_lines.append(
                f"[FOLDER] {folder}\n"
                f"  count={r['count']}  median_dt_ms={dt_txt}  est_fps={fps_txt}\n"
                f"  non_monotonic_ts={r['non_monotonic_ts']}  "
                f"index_gaps={r['index_gaps']}  large_ts_gaps={r['large_ts_gaps']}\n"
            )

        # ===== 输出控制 =====
        if perception_has_issue:
            bad_count += 1
            print(f"\n===== BAD: {perception_name} =====")
            log_fp.write(f"\n===== BAD: {perception_name} =====\n")
            for line in summary_lines:
                print(line.strip())
                log_fp.write(line)
        else:
            good_count += 1
            log_fp.write(f"GOOD: {perception_name}\n")

    log_fp.close()

    print("\n========== SUMMARY ==========")
    print(f"Total : {total_count}")
    print(f"Good  : {good_count}")
    print(f"Bad   : {bad_count}")
    print(f"Report saved to: {log_file}")

    return 1 if bad_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
