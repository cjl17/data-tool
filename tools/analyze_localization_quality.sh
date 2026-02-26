#!/bin/bash

set -e

INPUT_PATH="$1"

if [ -z "$INPUT_PATH" ]; then
    echo "用法:"
    echo "./analyze_localization_quality.sh <csv文件或目录>"
    exit 1
fi

# =============================
# 1. 判断输入类型
# =============================
if [ -f "$INPUT_PATH" ]; then
    INPUT_DIR=$(dirname "$INPUT_PATH")
    CSV_FILES=("$INPUT_PATH")
elif [ -d "$INPUT_PATH" ]; then
    INPUT_DIR="$INPUT_PATH"
    mapfile -t CSV_FILES < <(find "$INPUT_DIR" -maxdepth 1 -name "localization*.csv")
else
    echo "路径不存在: $INPUT_PATH"
    exit 1
fi

CSV_COUNT=${#CSV_FILES[@]}

if [ "$CSV_COUNT" -eq 0 ]; then
    echo "未找到CSV文件"
    exit 1
fi

# =============================
# 2. 创建输出目录（修复关键点）
# =============================
OUTPUT_DIR="$INPUT_DIR/localization_analysis"
mkdir -p "$OUTPUT_DIR"

echo "=========================================="
echo "找到 $CSV_COUNT 个CSV文件"
echo "输出目录: $OUTPUT_DIR"
echo "=========================================="

# =============================
# 3. 调用Python分析
# =============================
python3 - <<PYTHON
import os
import pandas as pd
import numpy as np

csv_files = """${CSV_FILES[*]}""".split()
output_dir = "$OUTPUT_DIR"

print("CSV files:", csv_files)

summary_list = []

def analyze_file(csv_path):
    df = pd.read_csv(csv_path)

    if len(df) < 10:
        return None

    # ========= 时间分析 =========
    if "timestamp" in df.columns:
        dt = df["timestamp"].diff().dropna()
    elif "time" in df.columns:
        dt = df["time"].diff().dropna()
    else:
        return None

    dt_median = dt.median()

    # 修复浮点误判（允许1%误差）
    dt_tol = dt_median * 1.01

    slow_mask = dt > dt_tol

    # ========= 加速度异常 =========
    anomaly_rows = []

    if "ax" in df.columns:
        dax = df["ax"].diff().abs()
        ax_thresh = np.percentile(dax.dropna(), 99.5)
        ax_mask = dax > ax_thresh
    else:
        ax_mask = np.zeros(len(df), dtype=bool)

    for i in range(len(df)):
        reasons = []
        if i < len(slow_mask) and slow_mask.iloc[i]:
            reasons.append("采样过慢")

        if i < len(ax_mask) and ax_mask.iloc[i]:
            reasons.append("ax跳变")

        if reasons:
            anomaly_rows.append((i, ";".join(reasons)))

    base = os.path.basename(csv_path).replace(".csv", "")

    anomaly_df = pd.DataFrame(anomaly_rows, columns=["row", "reason"])
    anomaly_file = os.path.join(output_dir, base + "_anomaly_report.csv")
    anomaly_df.to_csv(anomaly_file, index=False)

    summary = {
        "file": base,
        "total_rows": len(df),
        "anomaly_count": len(anomaly_rows),
        "dt_median": dt_median
    }

    return summary


for csv_file in csv_files:
    s = analyze_file(csv_file)
    if s:
        summary_list.append(s)

summary_df = pd.DataFrame(summary_list)
summary_df.to_csv(os.path.join(output_dir, "localization_summary_report.csv"), index=False)

print("分析完成")
PYTHON

echo "完成"
