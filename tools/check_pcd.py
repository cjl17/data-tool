#!/usr/bin/env python3
"""
Check exported JPG/PCD continuity (timestamp/index) from tools/extract_ros2_mcap_pcd_jpg.py output.

Assumes filenames like:
  <stamp_ms>_<idx>.jpg
  <stamp_ms>_<idx>.pcd

It scans:
  <export_dir>/images/*/*.(jpg|png|webp|tif|bin)
  <export_dir>/pcd/*/*.pcd

And reports per subfolder:
  - count
  - timestamp monotonicity violations
  - missing index gaps
  - large timestamp gaps (potential dropped frames) based on median dt
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


FNAME_RE = re.compile(r"^(?P<ts>\d+)_(?P<idx>\d+)\.(?P<ext>[A-Za-z0-9]+)$")


@dataclass(frozen=True)
class Frame:
    ts_ms: int
    idx: int
    path: Path


def _iter_files(export_dir: Path) -> Iterable[Path]:
    images = export_dir / "images"
    pcd = export_dir / "pcd"
    if images.is_dir():
        # images/<topic_short_or_long>/*.<ext>
        yield from images.glob("*/*")
    if pcd.is_dir():
        # pcd/<topic_dir>/*.pcd
        yield from pcd.glob("*/*.pcd")


def _parse_frame(p: Path) -> Optional[Frame]:
    m = FNAME_RE.match(p.name)
    if not m:
        return None
    try:
        ts_ms = int(m.group("ts"))
        idx = int(m.group("idx"))
    except ValueError:
        return None
    return Frame(ts_ms=ts_ms, idx=idx, path=p)


def _median_int(values: list[int]) -> Optional[int]:
    if not values:
        return None
    return int(statistics.median(values))


def analyze_folder(
    folder: Path,
    *,
    gap_factor: float,
    expected_dt_ms: Optional[int],
    max_examples: int,
) -> dict:
    frames: list[Frame] = []
    for p in sorted(folder.iterdir()):
        if not p.is_file():
            continue
        fr = _parse_frame(p)
        if fr:
            frames.append(fr)

    frames.sort(key=lambda f: (f.ts_ms, f.idx, f.path.name))
    n = len(frames)
    if n < 2:
        return {
            "folder": str(folder),
            "count": n,
            "estimated_dt_ms": None,
            "estimated_fps": None,
            "non_monotonic_ts": 0,
            "index_gaps": 0,
            "large_ts_gaps": 0,
            "examples": {},
        }

    # Check timestamp monotonicity + collect dt
    non_mono = 0
    non_mono_examples: list[str] = []
    dts: list[int] = []
    idx_gaps = 0
    idx_gap_examples: list[str] = []
    for a, b in zip(frames, frames[1:]):
        dt = b.ts_ms - a.ts_ms
        dts.append(dt)
        if dt < 0:
            non_mono += 1
            if len(non_mono_examples) < max_examples:
                non_mono_examples.append(f"{a.path.name} -> {b.path.name} (dt_ms={dt})")
        # Index gaps should be checked between consecutive frames (in time order),
        # since that's what "dropped frames" means operationally.
        d_idx = b.idx - a.idx
        if d_idx > 1:
            idx_gaps += (d_idx - 1)
            if len(idx_gap_examples) < max_examples:
                idx_gap_examples.append(f"missing {a.idx + 1}..{b.idx - 1} between {a.path.name} and {b.path.name}")

    med_dt = expected_dt_ms if expected_dt_ms is not None else _median_int([dt for dt in dts if dt >= 0])
    est_fps = (1000.0 / med_dt) if med_dt and med_dt > 0 else None

    # Large timestamp gaps (potential drops), based on median dt
    large_gaps = 0
    large_gap_examples: list[str] = []
    if med_dt and med_dt > 0:
        threshold = int(med_dt * gap_factor)
        for a, b in zip(frames, frames[1:]):
            dt = b.ts_ms - a.ts_ms
            if dt > threshold:
                large_gaps += 1
                if len(large_gap_examples) < max_examples:
                    large_gap_examples.append(f"{a.path.name} -> {b.path.name} (dt_ms={dt} > {threshold})")

    return {
        "folder": str(folder),
        "count": n,
        "estimated_dt_ms": med_dt,
        "estimated_fps": est_fps,
        "non_monotonic_ts": non_mono,
        "index_gaps": idx_gaps,
        "large_ts_gaps": large_gaps,
        "examples": {
            "non_monotonic_ts": non_mono_examples,
            "index_gaps": idx_gap_examples,
            "large_ts_gaps": large_gap_examples,
        },
    }


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Check exported JPG/PCD timestamp/index continuity.")
    p.add_argument(
        "export_dir",
        type=Path,
        help="Export directory (contains images/ and/or pcd/), e.g. .../export_pcd_jpg",
    )
    p.add_argument(
        "--gap-factor",
        type=float,
        default=3.0,
        help="Mark a timestamp gap as suspicious if dt > (median_dt * gap_factor). Default: 3.0",
    )
    p.add_argument(
        "--expected-dt-ms",
        type=int,
        default=None,
        help="If provided, use this dt (ms) instead of estimating from data.",
    )
    p.add_argument(
        "--max-examples",
        type=int,
        default=5,
        help="Max example lines to print per folder per check. Default: 5",
    )
    p.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Write full report to this JSON file.",
    )
    args = p.parse_args(argv)

    export_dir = args.export_dir.resolve()
    if not export_dir.exists():
        print(f"[ERROR] export_dir does not exist: {export_dir}", file=sys.stderr)
        return 2

    # Group by parent folder (topic folder)
    groups: dict[Path, list[Path]] = {}
    for f in _iter_files(export_dir):
        if f.is_file():
            groups.setdefault(f.parent, []).append(f)

    if not groups:
        print(f"[ERROR] No files found under {export_dir}/images/* or {export_dir}/pcd/*", file=sys.stderr)
        return 2

    report = {
        "export_dir": str(export_dir),
        "gap_factor": args.gap_factor,
        "expected_dt_ms": args.expected_dt_ms,
        "folders": [],
    }

    # Analyze each folder
    any_issues = False
    for folder in sorted(groups.keys()):
        r = analyze_folder(
            folder,
            gap_factor=args.gap_factor,
            expected_dt_ms=args.expected_dt_ms,
            max_examples=args.max_examples,
        )
        report["folders"].append(r)

        issues = (r["non_monotonic_ts"] > 0) or (r["index_gaps"] > 0) or (r["large_ts_gaps"] > 0)
        any_issues = any_issues or issues

        fps_txt = f"{r['estimated_fps']:.2f}" if r["estimated_fps"] else "n/a"
        dt_txt = str(r["estimated_dt_ms"]) if r["estimated_dt_ms"] is not None else "n/a"
        print(
            f"[FOLDER] {folder}\n"
            f"  count={r['count']}  median_dt_ms={dt_txt}  est_fps={fps_txt}\n"
            f"  non_monotonic_ts={r['non_monotonic_ts']}  index_gaps={r['index_gaps']}  large_ts_gaps={r['large_ts_gaps']}"
        )
        if issues:
            ex = r["examples"]
            for k in ("non_monotonic_ts", "index_gaps", "large_ts_gaps"):
                if ex.get(k):
                    for line in ex[k]:
                        print(f"    - {k}: {line}")

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[OK] JSON report written: {args.json_out}")

    return 1 if any_issues else 0


if __name__ == "__main__":
    raise SystemExit(main())


