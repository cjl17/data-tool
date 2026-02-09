#!/bin/bash

BASE_DIR="/media/ipc/BevData/bev_data/20251203"
SWEEP_DIR="${BASE_DIR}/sweep"

# 1. 创建 sweep 目录
mkdir -p "${SWEEP_DIR}"

# 2. 遍历 sequence 目录
for seq_dir in "${BASE_DIR}"/2025*; do
    [ -d "${seq_dir}" ] || continue

    seq_name=$(basename "${seq_dir}")
    ok_data_dir="${seq_dir}/ok_data"
    target_dir="${SWEEP_DIR}/bev_date_${seq_name}"

    # 只处理存在 ok_data 的 sequence
    if [ -d "${ok_data_dir}" ]; then
        if [ -e "${target_dir}" ]; then
            echo "⚠️ 已存在，跳过: ${target_dir}"
            continue
        fi

        echo "处理: ${ok_data_dir}  ->  ${target_dir}"
        cp -a "${ok_data_dir}" "${target_dir}"
    fi
done

echo "✅ sweep 数据整理完成（已保留原 ok_data）"

