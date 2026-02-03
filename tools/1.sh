#!/bin/bash
set -e

BASE_DIR="/media/ipc/AQLoopCloseData1"
OUT_ROOT="$BASE_DIR/packed_data"

mkdir -p "$OUT_ROOT"

echo "Unified output root:"
echo "  $OUT_ROOT"
echo "-----------------------------------"

# 查找所有 perception_data_*_csv
CSV_DIRS=$(find "$BASE_DIR" -maxdepth 1 -type d -name "perception_data_*_csv")

[ -z "$CSV_DIRS" ] && {
    echo "No CSV directories found"
    exit 1
}

for CSV_DIR in $CSV_DIRS; do
    BASE_NAME=$(basename "$CSV_DIR" | sed 's/_csv$//')
    RAW_ROOT="$BASE_DIR/$BASE_NAME/export_pcd_jpg"

    if [ ! -d "$RAW_ROOT" ]; then
        echo "Skip (no raw data): $BASE_NAME"
        continue
    fi

    # 🔑 perception_data_20260129113410 级别目录
    DATASET_OUT="$OUT_ROOT/$BASE_NAME"
    mkdir -p "$DATASET_OUT"

    echo "Processing dataset: $BASE_NAME"

    for csv_file in "$CSV_DIR"/"${BASE_NAME}"_*.csv; do
        [ -f "$csv_file" ] || continue

        csv_base=$(basename "$csv_file" .csv)
        prefix=$(echo "$csv_base" | awk -F'_' '{print $1"_"$2"_"$3"_"$4}')

        TARGET_DIR="$DATASET_OUT/$prefix"
        mkdir -p "$TARGET_DIR"

        cp "$csv_file" "$TARGET_DIR/"

        RAW_SUBDIR="$RAW_ROOT/$prefix"
        if [ -d "$RAW_SUBDIR" ]; then
            cp -r "$RAW_SUBDIR" "$TARGET_DIR/raw_data"
        else
            echo "  Warning: missing raw_data for $prefix"
        fi

        echo "  ✓ packed $BASE_NAME / $prefix"
    done

    echo "-----------------------------------"
done

echo "✅ All datasets packed into:"
echo "  $OUT_ROOT"
