#!/usr/bin/env python3
"""
Export ROS2 (rosbag2) MCAP data to:
- Per-topic image files (from sensor_msgs/msg/CompressedImage) as .jpg/.png/...
- Per-topic pointcloud frames (from sensor_msgs/msg/PointCloud2) as .pcd

Tested with rosbag2 metadata.yaml (storage_identifier: mcap) and serialization_format: cdr.
"""

from __future__ import annotations

import argparse
import json
import multiprocessing
import re
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import yaml


POINTFIELD_DT = {
    # sensor_msgs/msg/PointField datatype enum
    1: ("i1", 1),  # INT8
    2: ("u1", 1),  # UINT8
    3: ("i2", 2),  # INT16
    4: ("u2", 2),  # UINT16
    5: ("i4", 4),  # INT32
    6: ("u4", 4),  # UINT32
    7: ("f4", 4),  # FLOAT32
    8: ("f8", 8),  # FLOAT64
}


def _topic_dirname(topic: str) -> str:
    # /a/b/c -> a__b__c (portable and readable)
    t = topic.strip().strip("/")
    if not t:
        return "root"
    t = re.sub(r"[^A-Za-z0-9._/-]+", "_", t)
    return t.replace("/", "__")


def _sanitize_dir_component(name: str) -> str:
    # Safe single directory name component (no slashes)
    n = (name or "").strip()
    if not n:
        return "root"
    n = re.sub(r"[^A-Za-z0-9._-]+", "_", n)
    return n


def _short_image_dirname_from_topic(topic: str) -> Optional[str]:
    """
    Try to produce a short folder name for image topics.
    Example:
      /electronic_rearview_mirror/front_3mm/camera_image_jpeg -> front_3mm_jpeg
    """
    parts = [p for p in topic.strip().strip("/").split("/") if p]
    if len(parts) < 2:
        return None

    last = parts[-1]
    prev = parts[-2]

    # Common patterns we see in this dataset:
    # - camera_image_jpeg / camera_image_png / ...
    m = re.fullmatch(r"camera_image_([A-Za-z0-9]+)", last)
    if m:
        fmt = m.group(1).lower()
        return f"{prev}_{fmt}"

    # Fallback: image_jpeg / image_png / ...
    m = re.fullmatch(r"image_([A-Za-z0-9]+)", last)
    if m:
        fmt = m.group(1).lower()
        return f"{prev}_{fmt}"

    return None


def _image_topic_dirname_map(image_topics: Iterable[str]) -> dict[str, str]:
    """
    Map image topic -> output folder name.
    Uses short names when possible, and falls back to _topic_dirname on collisions.
    """
    topics = list(image_topics)
    short: dict[str, str] = {}
    for t in topics:
        candidate = _short_image_dirname_from_topic(t)
        if candidate:
            short[t] = _sanitize_dir_component(candidate)
        else:
            short[t] = _topic_dirname(t)

    # Collision detection: if multiple topics map to same folder, fall back to full names for those.
    inv: dict[str, list[str]] = {}
    for t, d in short.items():
        inv.setdefault(d, []).append(t)
    for d, ts in inv.items():
        if len(ts) > 1:
            for t in ts:
                short[t] = _topic_dirname(t)
    return short


def _safe_mkdir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def _write_rosbag2_metadata_yaml_from_single_mcap(
    dir_path: Path, mcap_path: Path, source_metadata_path: Optional[Path] = None
) -> None:
    """
    rosbags.rosbag2.Reader expects rosbag2-style metadata.yaml.
    Generate metadata for a single mcap file.
    
    If source_metadata_path is provided, read topics and message types from it.
    Otherwise, fall back to reading MCAP summary (channels/schemas).
    """
    from mcap.reader import make_reader

    # Try to read from source metadata.yaml first
    topics_with_count = []
    if source_metadata_path and source_metadata_path.exists():
        try:
            with source_metadata_path.open("r", encoding="utf-8") as f:
                source_meta = yaml.safe_load(f)
            bag_info = source_meta.get("rosbag2_bagfile_information", {})
            # Extract topics from source metadata
            source_topics = bag_info.get("topics_with_message_count", [])
            for topic_info in source_topics:
                topic_meta = topic_info.get("topic_metadata", {})
                topics_with_count.append(
                    {
                        "topic_metadata": {
                            "name": topic_meta.get("name", ""),
                            "type": topic_meta.get("type", ""),
                            "serialization_format": topic_meta.get("serialization_format", "cdr"),
                            "offered_qos_profiles": topic_meta.get("offered_qos_profiles", ""),
                        },
                        "message_count": 0,  # Will be updated from MCAP if available
                    }
                )
        except Exception:
            # If reading source metadata fails, fall back to MCAP summary
            topics_with_count = []

    # Read MCAP summary once (used for both topics and timing)
    with mcap_path.open("rb") as f:
        r = make_reader(f)
        summary = r.get_summary()

    # If we don't have topics from source metadata, read from MCAP summary
    if not topics_with_count:
        channels = getattr(summary, "channels", {}) or {}
        schemas = getattr(summary, "schemas", {}) or {}

        for ch in channels.values():
            topic = getattr(ch, "topic", None)
            schema_id = getattr(ch, "schema_id", None)
            if not topic or schema_id is None:
                continue
            sch = schemas.get(schema_id)
            msgtype = getattr(sch, "name", None) if sch else None
            if not msgtype:
                continue

            topics_with_count.append(
                {
                    "topic_metadata": {
                        "name": str(topic),
                        "type": str(msgtype),
                        "serialization_format": "cdr",
                        "offered_qos_profiles": "",
                    },
                    "message_count": 0,
                }
            )

    # mcap summary fields (best-effort; set safe defaults when absent)
    msg_count = int(getattr(getattr(summary, "statistics", None), "message_count", 0) or 0)
    start_ns = int(getattr(getattr(summary, "statistics", None), "message_start_time", 0) or 0)
    end_ns = int(getattr(getattr(summary, "statistics", None), "message_end_time", 0) or 0)
    duration_ns = max(0, end_ns - start_ns) if start_ns and end_ns else 0

    meta = {
        "rosbag2_bagfile_information": {
            "version": 5,
            "storage_identifier": "mcap",
            "duration": {"nanoseconds": int(duration_ns)},
            "starting_time": {"nanoseconds_since_epoch": int(start_ns or 0)},
            "message_count": int(msg_count),
            "topics_with_message_count": topics_with_count,
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [mcap_path.name],
            "files": [
                {
                    "path": mcap_path.name,
                    "starting_time": {"nanoseconds_since_epoch": int(start_ns or 0)},
                    "duration": {"nanoseconds": int(duration_ns)},
                    "message_count": int(msg_count),
                }
            ],
        }
    }

    (dir_path / "metadata.yaml").write_text(
        yaml.safe_dump(meta, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _guess_ext_from_format(fmt: str) -> str:
    f = (fmt or "").lower()
    if "jpeg" in f or f.strip() == "jpg":
        return ".jpg"
    if "png" in f:
        return ".png"
    if "webp" in f:
        return ".webp"
    if "tiff" in f or "tif" in f:
        return ".tif"
    # CompressedImage format is usually "jpeg" / "png"; fallback to .bin to avoid lying
    return ".bin"


def _stamp_to_ns(stamp) -> int:
    # builtin_interfaces/msg/Time: sec + nanosec
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _stamp_to_ms(stamp_ns: int) -> int:
    # Convert nanoseconds to milliseconds (divide by 1_000_000)
    return stamp_ns // 1_000_000


def _pcd_header(
    *,
    fields: list[str],
    sizes: list[int],
    types: list[str],
    counts: list[int],
    width: int,
    height: int,
    points: int,
) -> bytes:
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            f"FIELDS {' '.join(fields)}",
            f"SIZE {' '.join(str(x) for x in sizes)}",
            f"TYPE {' '.join(types)}",
            f"COUNT {' '.join(str(x) for x in counts)}",
            f"WIDTH {width}",
            f"HEIGHT {height}",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {points}",
            "DATA binary",
            "",
        ]
    )
    return header.encode("ascii")


def _pcd_type_char(np_code: str) -> str:
    # PCD TYPE: F/I/U
    if np_code.startswith("f"):
        return "F"
    if np_code.startswith("i"):
        return "I"
    if np_code.startswith("u"):
        return "U"
    # fallback
    return "U"

def _log(msg: str, quiet: bool = False) -> None:
    if not quiet:
        print(msg, flush=True)


@dataclass(frozen=True)
class TopicFilters:
    image_topic_regex: Optional[re.Pattern]
    pcd_topic_regex: Optional[re.Pattern]

    def want_images(self, topic: str) -> bool:
        return self.image_topic_regex is None or bool(self.image_topic_regex.search(topic))

    def want_pcd(self, topic: str) -> bool:
        return self.pcd_topic_regex is None or bool(self.pcd_topic_regex.search(topic))


def _is_already_processed(out_dir: Path, overwrite: bool) -> bool:
    """Check if output directory already exists and has been processed."""
    if overwrite:
        return False
    stats_file = out_dir / "export_stats.json"
    if stats_file.exists() and out_dir.exists():
        # Check if both images and pcd directories exist
        images_dir = out_dir / "images"
        pcd_dir = out_dir / "pcd"
        if images_dir.exists() or pcd_dir.exists():
            return True
    return False


def export_bag(
    bag_path: Path,
    out_dir: Path,
    filters: TopicFilters,
    *,
    overwrite: bool,
    every_n: int,
    max_total: Optional[int],
    max_per_topic: Optional[int],
    progress_seconds: float,
    quiet: bool = False,
) -> tuple[bool, Optional[str]]:
    """
    Export bag to output directory.
    Returns: (success: bool, error_message: Optional[str])
    """
    # Check if already processed
    if _is_already_processed(out_dir, overwrite):
        _log(f"[SKIP] Already processed: {out_dir}", quiet=quiet)
        return (True, None)

    try:
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        _safe_mkdir(out_dir)
        out_images = out_dir / "images"
        out_pcd = out_dir / "pcd"
        _safe_mkdir(out_images)
        _safe_mkdir(out_pcd)

        stats = {
            "bag_path": str(bag_path),
            "export_dir": str(out_dir),
            "images": {},
            "pcd": {},
        }

        t0 = time.monotonic()
        last_progress = t0
        scanned_total = 0
        _log(f"[INFO] bag_path: {bag_path}", quiet=quiet)
        _log(f"[INFO] out_dir: {out_dir}", quiet=quiet)
        _log(f"[INFO] overwrite={overwrite} every_n={every_n} max_total={max_total} max_per_topic={max_per_topic}", quiet=quiet)

        # Handle single .mcap file: rosbags Reader requires a directory with metadata.yaml.
        # Always create a temporary rosbag2 dir with its own metadata.yaml for each mcap file.
        # Try to read topics from source metadata.yaml if available.
        reader_cm = None
        if bag_path.is_file() and bag_path.suffix.lower() == ".mcap":
            _log("[INFO] Single .mcap file detected; creating temporary rosbag2 dir with independent metadata.yaml...", quiet=quiet)
            tmp = tempfile.TemporaryDirectory(prefix="single_mcap_rosbag2_")
            tmp_dir = Path(tmp.name)
            # Try to find source metadata.yaml in parent directory
            source_metadata = bag_path.parent / "metadata.yaml"
            if not source_metadata.exists():
                source_metadata = None
            _write_rosbag2_metadata_yaml_from_single_mcap(tmp_dir, bag_path, source_metadata)
            # Prefer symlink to avoid copy; fallback to copy if symlink not allowed.
            try:
                (tmp_dir / bag_path.name).symlink_to(bag_path)
            except OSError:
                import shutil

                shutil.copy2(bag_path, tmp_dir / bag_path.name)
            reader_cm = Reader(tmp_dir)
            # Keep tmp alive while reader is in use by attaching to reader_cm
            reader_cm._tmpdir = tmp  # type: ignore[attr-defined]
        else:
            reader_cm = Reader(bag_path)

        with reader_cm as reader:
            # Build quick topic -> msgtype map
            topic_types = {c.topic: c.msgtype for c in reader.connections}
            (out_dir / "topics.json").write_text(json.dumps(topic_types, indent=2, sort_keys=True), encoding="utf-8")

            # Pre-filter connections so we don't iterate unrelated topics.
            selected_conns = []
            for c in reader.connections:
                if c.msgtype == "sensor_msgs/msg/CompressedImage" and filters.want_images(c.topic):
                    selected_conns.append(c)
                elif c.msgtype == "sensor_msgs/msg/PointCloud2" and filters.want_pcd(c.topic):
                    selected_conns.append(c)

            if not selected_conns:
                raise RuntimeError("No matching topics found for export (check regex filters).")

            image_topics = [c.topic for c in selected_conns if c.msgtype == "sensor_msgs/msg/CompressedImage"]
            image_dirnames = _image_topic_dirname_map(image_topics)

            _log(f"[INFO] selected_topics: {len(selected_conns)}", quiet=quiet)
            if not quiet:
                for c in selected_conns:
                    _log(f"  - {c.topic} ({c.msgtype})")
            _log("[INFO] start reading messages...", quiet=quiet)

            per_topic_idx: dict[str, int] = {}
            exported_total = 0
            exported_per_topic: dict[str, int] = {}

            for conn, t_ns_epoch, raw in reader.messages(connections=selected_conns):
                scanned_total += 1
                idx = per_topic_idx.get(conn.topic, 0) + 1
                per_topic_idx[conn.topic] = idx
                if every_n > 1 and (idx % every_n) != 0:
                    continue
                if max_per_topic is not None and exported_per_topic.get(conn.topic, 0) >= max_per_topic:
                    continue
                if max_total is not None and exported_total >= max_total:
                    break

                if conn.msgtype == "sensor_msgs/msg/CompressedImage":
                    if not filters.want_images(conn.topic):
                        continue

                    msg = typestore.deserialize_cdr(raw, conn.msgtype)
                    ext = _guess_ext_from_format(getattr(msg, "format", ""))
                    topic_dir = out_images / image_dirnames.get(conn.topic, _topic_dirname(conn.topic))
                    _safe_mkdir(topic_dir)

                    stamp_ns = _stamp_to_ns(msg.header.stamp) if getattr(msg, "header", None) else int(t_ns_epoch)
                    stamp_ms = _stamp_to_ms(stamp_ns)
                    fn = f"{stamp_ms:013d}{ext}"
                    out_path = topic_dir / fn
                    if out_path.exists():
                        _log(f"[WARNING] Duplicate timestamp {stamp_ms:013d} for topic {conn.topic}: {out_path}", quiet=quiet)
                        if not overwrite:
                            continue
                        else:
                            _log(f"[INFO] Overwriting existing file: {out_path}", quiet=quiet)
                    # msg.data is bytes-like (often array('B')); avoid copy via memoryview.
                    with out_path.open("wb") as f:
                        f.write(memoryview(msg.data))

                    stats["images"][conn.topic] = stats["images"].get(conn.topic, 0) + 1
                    exported_total += 1
                    exported_per_topic[conn.topic] = exported_per_topic.get(conn.topic, 0) + 1

                elif conn.msgtype == "sensor_msgs/msg/PointCloud2":
                    if not filters.want_pcd(conn.topic):
                        continue

                    msg = typestore.deserialize_cdr(raw, conn.msgtype)
                    topic_dir = out_pcd / _topic_dirname(conn.topic)
                    _safe_mkdir(topic_dir)

                    stamp_ns = _stamp_to_ns(msg.header.stamp) if getattr(msg, "header", None) else int(t_ns_epoch)
                    stamp_ms = _stamp_to_ms(stamp_ns)
                    fn = f"{stamp_ms:013d}.pcd"
                    out_path = topic_dir / fn
                    if out_path.exists():
                        _log(f"[WARNING] Duplicate timestamp {stamp_ms:013d} for topic {conn.topic}: {out_path}", quiet=quiet)
                        if not overwrite:
                            continue
                        else:
                            _log(f"[INFO] Overwriting existing file: {out_path}", quiet=quiet)

                    # Build PCD schema from PointCloud2 fields
                    fields = []
                    sizes = []
                    types = []
                    counts = []

                    # Sort by offset so PCD field order matches memory layout (when possible)
                    pc_fields = sorted(list(msg.fields), key=lambda f: int(f.offset))
                    for f in pc_fields:
                        np_code, size = POINTFIELD_DT.get(int(f.datatype), ("u1", 1))
                        fields.append(str(f.name))
                        sizes.append(size)
                        types.append(_pcd_type_char(np_code))
                        counts.append(int(f.count))

                    total_points = int(msg.width) * int(msg.height)
                    header = _pcd_header(
                        fields=fields,
                        sizes=sizes,
                        types=types,
                        counts=counts,
                        width=int(msg.width),
                        height=int(msg.height),
                        points=total_points,
                    )

                    # Fast-path: if point_step equals packed size and offsets are contiguous, write raw bytes directly.
                    packed_size = sum(s * c for s, c in zip(sizes, counts))
                    contiguous = True
                    expected_off = 0
                    for f, s, c in zip(pc_fields, sizes, counts):
                        if int(f.offset) != expected_off:
                            contiguous = False
                            break
                        expected_off += s * c
                    can_dump_raw = contiguous and packed_size == int(msg.point_step)

                    with out_path.open("wb") as f:
                        f.write(header)
                        if can_dump_raw:
                            f.write(memoryview(msg.data))
                        else:
                            # Generic repack (handles padding / non-contiguous layouts)
                            buf = memoryview(msg.data)
                            step = int(msg.point_step)
                            for i in range(total_points):
                                base = i * step
                                for fld, s, c in zip(pc_fields, sizes, counts):
                                    off = int(fld.offset)
                                    f.write(buf[base + off : base + off + s * c])

                    stats["pcd"][conn.topic] = stats["pcd"].get(conn.topic, 0) + 1
                    exported_total += 1
                    exported_per_topic[conn.topic] = exported_per_topic.get(conn.topic, 0) + 1

                if progress_seconds > 0:
                    now = time.monotonic()
                    if now - last_progress >= progress_seconds:
                        dt = now - t0
                        rate = scanned_total / dt if dt > 0 else 0.0
                        _log(
                            f"[PROGRESS] scanned={scanned_total} exported={exported_total} "
                            f"elapsed={dt:.1f}s rate={rate:.1f} msgs/s last_topic={conn.topic}",
                            quiet=quiet,
                        )
                        last_progress = now

            (out_dir / "export_stats.json").write_text(json.dumps(stats, indent=2, sort_keys=True), encoding="utf-8")
            dt = time.monotonic() - t0
            _log(f"[DONE] scanned={scanned_total} exported={exported_total} elapsed={dt:.1f}s", quiet=quiet)
            return (True, None)
    except Exception as e:
        error_msg = f"Error processing {bag_path}: {e}"
        _log(f"[ERROR] {error_msg}", quiet=quiet)
        return (False, error_msg)


def _extract_timestamp_from_perception_dir(perception_dir_name: str) -> Optional[str]:
    """
    Extract timestamp from perception_data_* directory name.
    Example: perception_data_20260205100643 -> 20260205100643
    """
    if not perception_dir_name.startswith("perception_data_"):
        return None
    # Remove "perception_data_" prefix
    timestamp_part = perception_dir_name[len("perception_data_"):]
    # Extract the timestamp part (before any underscore suffix like _0, _1, etc.)
    if "_" in timestamp_part:
        timestamp_part = timestamp_part.split("_")[0]
    return timestamp_part


def _get_first_dir_name(perception_dir: Path) -> str:
    """
    Generate first_* directory name from perception_data_* directory.
    Example: perception_data_20260205100643 -> first_20260205100643
    """
    timestamp = _extract_timestamp_from_perception_dir(perception_dir.name)
    if timestamp:
        return f"first_{timestamp}"
    # Fallback: use directory name with first_ prefix
    return f"first_{perception_dir.name}"


def _is_perception_data_dir(path: Path) -> bool:
    """Check if a path is a perception_data_* directory (excluding _csv suffix)."""
    return path.name.startswith("perception_data_") and not path.name.endswith("_csv")


def _get_base_out_dir(bag_path: Path, out_arg: Optional[Path]) -> Path:
    """
    Determine base output directory based on bag_path and out_arg.
    Creates first_* directory for perception_data_* directories if out_arg is not provided.
    """
    if out_arg:
        return out_arg.resolve()
    
    if _is_perception_data_dir(bag_path):
        # Create first_* directory in parent directory
        first_dir_name = _get_first_dir_name(bag_path)
        first_dir = bag_path.parent / first_dir_name
        first_dir.mkdir(parents=True, exist_ok=True)
        return first_dir.resolve()
    
    # For non-perception_data_* directories, use mcap_extracted
    if bag_path.is_dir():
        return (bag_path / "mcap_extracted").resolve()
    
    # For single files, use parent/raw_data
    return (bag_path.parent / "raw_data").resolve()


def _find_perception_data_dirs(search_dir: Path) -> list[Path]:
    """Find all perception_data_* directories in the search directory."""
    perception_dirs = []
    for item in search_dir.iterdir():
        if item.is_dir() and item.name.startswith("perception_data_"):
            perception_dirs.append(item)
    return sorted(perception_dirs)


def _find_mcap_files_in_rosbag2_dir(rosbag2_dir: Path) -> list[Path]:
    """Find all .mcap files in a rosbag2 directory."""
    mcap_files = []
    # Check metadata.yaml for relative_file_paths
    metadata_path = rosbag2_dir / "metadata.yaml"
    if metadata_path.exists():
        try:
            with metadata_path.open("r", encoding="utf-8") as f:
                meta = yaml.safe_load(f)
            relative_paths = meta.get("rosbag2_bagfile_information", {}).get("relative_file_paths", [])
            for rel_path in relative_paths:
                mcap_path = rosbag2_dir / rel_path
                if mcap_path.exists() and mcap_path.suffix.lower() == ".mcap":
                    mcap_files.append(mcap_path)
        except Exception:
            pass
    
    # Fallback: find all .mcap files in the directory
    if not mcap_files:
        mcap_files = sorted(rosbag2_dir.glob("*.mcap"))
    
    return mcap_files


def _process_single_mcap(
    mcap_file: Path,
    base_out_dir: Path,
    filters: TopicFilters,
    overwrite: bool,
    every_n: int,
    max_total: Optional[int],
    max_per_topic: Optional[int],
    progress_seconds: float,
    quiet: bool,
    use_mcap_name_as_dir: bool = False,
) -> tuple[Path, bool, Optional[str]]:
    """
    Process a single mcap file. Returns (mcap_file, success, error_msg).
    
    Args:
        use_mcap_name_as_dir: If True, use mcap file name (without extension) as directory name,
                              and create raw_data subdirectory inside it. Otherwise, use mcap file stem directly.
    """
    if use_mcap_name_as_dir:
        # Use mcap file name (without extension) as directory name
        mcap_dir_name = _sanitize_dir_component(mcap_file.stem)
        mcap_dir = base_out_dir / mcap_dir_name
        # Create raw_data subdirectory inside mcap directory
        out_dir = mcap_dir / "raw_data"
        # Pre-create directories to avoid race conditions in parallel processing
        _safe_mkdir(out_dir)
    else:
        # Create subdirectory for each mcap file (original behavior)
        mcap_stem = _sanitize_dir_component(mcap_file.stem)
        out_dir = base_out_dir / mcap_stem
    
    if out_dir.exists() and not out_dir.is_dir():
        return (mcap_file, False, f"Output path exists but is not a directory: {out_dir}")
    
    success, error_msg = export_bag(
        mcap_file,
        out_dir,
        filters,
        overwrite=overwrite,
        every_n=every_n,
        max_total=max_total,
        max_per_topic=max_per_topic,
        progress_seconds=progress_seconds,
        quiet=quiet,
    )
    return (mcap_file, success, error_msg)


def _process_mcap_files(
    mcap_files: list[Path],
    base_out_dir: Path,
    filters: TopicFilters,
    args: argparse.Namespace,
    use_mcap_name_as_dir: bool = False,
) -> int:
    """
    Process a list of mcap files (parallel or sequential).
    Returns the number of successfully processed files.
    """
    if not mcap_files:
        return 0
    
    # Pre-create directories to reduce file system contention during parallel processing
    if use_mcap_name_as_dir:
        for mcap_file in mcap_files:
            mcap_dir_name = _sanitize_dir_component(mcap_file.stem)
            mcap_dir = base_out_dir / mcap_dir_name
            raw_data_dir = mcap_dir / "raw_data"
            _safe_mkdir(raw_data_dir)
    
    num_jobs = args.jobs if args.jobs is not None else multiprocessing.cpu_count()
    use_parallel = num_jobs > 1 and len(mcap_files) > 1
    
    if use_parallel:
        if not args.quiet:
            print(f"[INFO] Processing {len(mcap_files)} files with {num_jobs} parallel jobs...")
        with multiprocessing.Pool(processes=num_jobs) as pool:
            results = pool.starmap(
                _process_single_mcap,
                [
                    (
                        mcap_file,
                        base_out_dir,
                        filters,
                        args.overwrite,
                        args.every_n,
                        args.max_total,
                        args.max_per_topic,
                        args.progress_seconds,
                        args.quiet,
                        use_mcap_name_as_dir,
                    )
                    for mcap_file in sorted(mcap_files)
                ],
            )
        
        success_count = 0
        for mcap_file, success, error_msg in results:
            if success:
                if not args.quiet:
                    print(f"[OK] {mcap_file.name}")
                success_count += 1
            else:
                print(f"[ERROR] {mcap_file.name}: {error_msg}", file=sys.stderr)
        return success_count
    else:
        # Sequential processing
        success_count = 0
        for mcap_file in sorted(mcap_files):
            try:
                mcap_dir_name = _sanitize_dir_component(mcap_file.stem)
                if not args.quiet:
                    if use_mcap_name_as_dir:
                        print(f"[INFO] Processing: {mcap_file.name} -> {mcap_dir_name}/raw_data")
                    else:
                        print(f"[INFO] Processing: {mcap_file.name}")
                
                success, error_msg = _process_single_mcap(
                    mcap_file,
                    base_out_dir,
                    filters,
                    args.overwrite,
                    args.every_n,
                    args.max_total,
                    args.max_per_topic,
                    args.progress_seconds,
                    args.quiet,
                    use_mcap_name_as_dir,
                )
                if success:
                    if not args.quiet:
                        if use_mcap_name_as_dir:
                            mcap_dir = base_out_dir / mcap_dir_name
                            raw_data_dir = mcap_dir / "raw_data"
                            print(f"[OK] Export complete: {raw_data_dir}")
                        else:
                            out_dir = base_out_dir / mcap_dir_name
                            print(f"[OK] Export complete: {out_dir}")
                    success_count += 1
                else:
                    print(f"[ERROR] Failed: {error_msg}", file=sys.stderr)
            except Exception as e:
                print(f"[ERROR] Failed to process {mcap_file.name}: {e}", file=sys.stderr)
                continue
        return success_count


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Extract ROS2 MCAP (rosbag2 + CDR) images (CompressedImage) and pointclouds (PointCloud2) to JPG/PCD.",
    )
    p.add_argument(
        "bag_path",
        type=Path,
        nargs="?",
        default=None,
        help="Bag directory containing metadata.yaml and *.mcap segments (rosbag2 MCAP), or a single .mcap file. "
             "If not provided, automatically processes all perception_data_* directories in --search-dir.",
    )
    p.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output directory. Default: <bag_path>/raw_data or <mcap_file_dir>/raw_data",
    )
    p.add_argument(
        "--search-dir",
        type=Path,
        default=None,
        help="Directory to search for perception_data_* directories when bag_path is not provided. Default: current directory.",
    )
    p.add_argument(
        "--image-topic-regex",
        type=str,
        default=None,
        help="Only export CompressedImage topics matching this regex (default: all).",
    )
    p.add_argument(
        "--pcd-topic-regex",
        type=str,
        default=None,
        help="Only export PointCloud2 topics matching this regex (default: all).",
    )
    p.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing files (default: skip if exists).",
    )
    p.add_argument(
        "--every-n",
        type=int,
        default=1,
        help="Downsample: export every Nth message per topic (default: 1).",
    )
    p.add_argument(
        "--progress-seconds",
        type=float,
        default=5.0,
        help="Print progress every N seconds (0 disables). Default: 5.",
    )
    p.add_argument(
        "--max-total",
        type=int,
        default=None,
        help="Stop after exporting this many files total (across all topics).",
    )
    p.add_argument(
        "--max-per-topic",
        type=int,
        default=None,
        help="Stop exporting once each topic reaches this many files.",
    )
    p.add_argument(
        "--jobs",
        type=int,
        default=None,
        help="Number of parallel jobs for processing multiple mcap files. Default: number of CPU cores.",
    )
    p.add_argument(
        "--quiet",
        action="store_true",
        help="Reduce log output (useful for parallel processing).",
    )
    args = p.parse_args(argv)

    # Validate filter arguments first
    filters = TopicFilters(
        image_topic_regex=re.compile(args.image_topic_regex) if args.image_topic_regex else None,
        pcd_topic_regex=re.compile(args.pcd_topic_regex) if args.pcd_topic_regex else None,
    )

    if args.every_n < 1:
        print("[ERROR] --every-n must be >= 1", file=sys.stderr)
        return 2
    if args.max_total is not None and args.max_total < 1:
        print("[ERROR] --max-total must be >= 1", file=sys.stderr)
        return 2
    if args.max_per_topic is not None and args.max_per_topic < 1:
        print("[ERROR] --max-per-topic must be >= 1", file=sys.stderr)
        return 2
    if args.progress_seconds < 0:
        print("[ERROR] --progress-seconds must be >= 0", file=sys.stderr)
        return 2

    # Determine what to process
    if args.bag_path is None:
        # Auto-find all perception_data_* directories
        search_dir = (args.search_dir.resolve() if args.search_dir else Path.cwd()).resolve()
        if not search_dir.exists():
            print(f"[ERROR] search_dir does not exist: {search_dir}", file=sys.stderr)
            return 2
        
        perception_dirs = _find_perception_data_dirs(search_dir)
        if not perception_dirs:
            print(f"[ERROR] No perception_data_* directories found in: {search_dir}", file=sys.stderr)
            return 2
        
        print(f"[INFO] Found {len(perception_dirs)} perception_data_* directories:")
        for pd in perception_dirs:
            print(f"  - {pd}")
        print()
        
        # Process each directory: find all mcap files and process each separately
        total_success = 0
        total_mcap_files = 0
        for bag_path in perception_dirs:
            try:
                print(f"[INFO] Processing directory: {bag_path}")
                mcap_files = _find_mcap_files_in_rosbag2_dir(bag_path)
                if not mcap_files:
                    print(f"[WARNING] No .mcap files found in: {bag_path}")
                    continue
                
                print(f"  Found {len(mcap_files)} .mcap files:")
                for mf in mcap_files:
                    print(f"    - {mf.name}")
                
                # Determine base output directory
                base_out_dir = _get_base_out_dir(bag_path, args.out)
                if not args.out:
                    print(f"  [INFO] Output directory: {base_out_dir}")
                
                # Process each mcap file (parallel or sequential)
                dir_success = _process_mcap_files(
                    mcap_files,
                    base_out_dir,
                    filters,
                    args,
                    use_mcap_name_as_dir=True,
                )
                total_success += dir_success
                
                total_mcap_files += len(mcap_files)
                print(f"  [SUMMARY] {dir_success}/{len(mcap_files)} mcap files processed successfully in {bag_path}\n")
            except Exception as e:
                print(f"[ERROR] Failed to process directory {bag_path}: {e}", file=sys.stderr)
                continue
        
        print(f"\n[SUMMARY] Successfully processed {total_success}/{total_mcap_files} mcap files across {len(perception_dirs)} directories")
        return 0 if total_success == total_mcap_files and total_mcap_files > 0 else 1
    
    else:
        # Process single bag_path (file or directory)
        bag_path = args.bag_path.resolve()
        if not bag_path.exists():
            print(f"[ERROR] bag_path does not exist: {bag_path}", file=sys.stderr)
            return 2
        
        # Check if it's a single .mcap file or a rosbag2 directory
        is_single_mcap = bag_path.is_file() and bag_path.suffix.lower() == ".mcap"
        is_rosbag2_dir = bag_path.is_dir() and (bag_path / "metadata.yaml").exists()
        
        if is_single_mcap:
            # Process single .mcap file
            # Determine output directory
            if args.out:
                out_dir = args.out.resolve()
            else:
                # Check if mcap file is in a perception_data_* directory
                parent_dir = bag_path.parent
                if _is_perception_data_dir(parent_dir):
                    # Create first_* directory in parent's parent directory
                    base_out_dir = _get_base_out_dir(parent_dir, None)
                    # Create mcap_name/raw_data structure inside first_* directory
                    mcap_dir_name = _sanitize_dir_component(bag_path.stem)
                    mcap_dir = base_out_dir / mcap_dir_name
                    out_dir = mcap_dir / "raw_data"
                    print(f"[INFO] Output directory: {out_dir}")
                else:
                    # Fallback to original behavior
                    out_dir = (bag_path.parent / "raw_data" / bag_path.stem).resolve()
            if out_dir.exists() and not out_dir.is_dir():
                print(f"[ERROR] --out exists but is not a directory: {out_dir}", file=sys.stderr)
                return 2

            success, error_msg = export_bag(
                bag_path,
                out_dir,
                filters,
                overwrite=args.overwrite,
                every_n=args.every_n,
                max_total=args.max_total,
                max_per_topic=args.max_per_topic,
                progress_seconds=args.progress_seconds,
                quiet=args.quiet,
            )
            if success:
                print(f"[OK] Export complete: {out_dir}")
                return 0
            else:
                print(f"[ERROR] {error_msg}", file=sys.stderr)
                return 1
        
        elif is_rosbag2_dir:
            # Process rosbag2 directory: find all mcap files and process each separately
            mcap_files = _find_mcap_files_in_rosbag2_dir(bag_path)
            if not mcap_files:
                print(f"[ERROR] No .mcap files found in rosbag2 directory: {bag_path}", file=sys.stderr)
                return 2
            
            print(f"[INFO] Found {len(mcap_files)} .mcap files in rosbag2 directory:")
            for mf in mcap_files:
                print(f"  - {mf.name}")
            print()
            
            # Determine base output directory
            base_out_dir = _get_base_out_dir(bag_path, args.out)
            if not args.out:
                print(f"[INFO] Output directory: {base_out_dir}")
            
            # Check if we should use mcap name as directory (for perception_data_* directories)
            use_mcap_name = not args.out and _is_perception_data_dir(bag_path)
            
            # Process each mcap file (parallel or sequential)
            success_count = _process_mcap_files(
                mcap_files,
                base_out_dir,
                filters,
                args,
                use_mcap_name_as_dir=use_mcap_name,
            )
            
            print(f"\n[SUMMARY] Successfully processed {success_count}/{len(mcap_files)} mcap files")
            return 0 if success_count == len(mcap_files) else 1
        
        else:
            # Check if it's a directory with .mcap files but no metadata.yaml
            mcap_files = list(bag_path.glob("*.mcap"))
            if mcap_files:
                # Process each mcap file separately
                print(f"[INFO] Found {len(mcap_files)} .mcap files in directory (no metadata.yaml):")
                for mf in mcap_files:
                    print(f"  - {mf.name}")
                print()
                
                # Determine base output directory
                base_out_dir = _get_base_out_dir(bag_path, args.out)
                if not args.out:
                    print(f"[INFO] Output directory: {base_out_dir}")
                
                # Check if we should use mcap name as directory (for perception_data_* directories)
                use_mcap_name = not args.out and _is_perception_data_dir(bag_path)
                
                # Process each mcap file (parallel or sequential)
                success_count = _process_mcap_files(
                    sorted(mcap_files),
                    base_out_dir,
                    filters,
                    args,
                    use_mcap_name_as_dir=use_mcap_name,
                )
                
                print(f"\n[SUMMARY] Successfully processed {success_count}/{len(mcap_files)} mcap files")
                return 0 if success_count == len(mcap_files) else 1
            else:
                print(f"[ERROR] bag_path must be either a .mcap file or a rosbag2 directory with metadata.yaml: {bag_path}", file=sys.stderr)
                return 2


if __name__ == "__main__":
    raise SystemExit(main())


