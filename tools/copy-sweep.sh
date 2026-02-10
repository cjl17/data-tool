#!/bin/bash
set -euo pipefail
shopt -s nullglob

########################################
# 路径解析
########################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(dirname "$(dirname "${SCRIPT_DIR}")")"

echo "脚本目录: ${SCRIPT_DIR}"
echo "基础目录: ${BASE_DIR}"
echo

########################################
# 单个 perception 处理函数
########################################
process_one() {
    ok_data_dir="$1"
    first_dir="$2"

    perception_dir="$(dirname "$ok_data_dir")"
    perception_name="$(basename "$perception_dir")"

    # 解析 perception_data_{timestamp}_{index}
    [[ "$perception_name" =~ perception_data_([0-9]+)_([0-9]+) ]] || return

    timestamp="${BASH_REMATCH[1]}"
    index="${BASH_REMATCH[2]}"
    sequence_name=$(printf "sequence%05d" "$index")

    target_dir="${first_dir}/sweep/bev_data_${timestamp}_${index}_${sequence_name}"

    if [ -e "$target_dir" ]; then
        echo "⚠️ 已存在，跳过: $target_dir"
        return
    fi

    mkdir -p "$target_dir"

    echo "➡️  开始拷贝: $(basename "$perception_dir")"

    # HDD 友好 rsync + 总体进度
    rsync -rL \
        --whole-file \
        --inplace \
        --no-perms --no-owner --no-group \
        --omit-dir-times \
        --info=progress2,stats1 \
        "${ok_data_dir}/" "${target_dir}/"

    # CSV 单独拷贝
    csv="${perception_dir}/localization_${index}.csv"
    if [ -f "$csv" ]; then
        cp "$csv" "$target_dir/"
    fi

    echo "✅ 完成: $(basename "$target_dir")"
    echo
}

export -f process_one

########################################
# 主循环：first*
########################################
for first_dir in "${BASE_DIR}"/first*; do
    [ -d "$first_dir" ] || continue

    first_name="$(basename "$first_dir")"
    echo "=============================="
    echo "处理 first 目录: $first_name"
    echo "=============================="

    mkdir -p "${first_dir}/sweep"

    # 查找所有 ok_data，并行 2 路（HDD 最优）
    find "$first_dir" -maxdepth 2 -path "*/perception_data_*/ok_data" -type d \
    | parallel --line-buffer -j 2 process_one {} "$first_dir"

    echo "🎉 ${first_name} sweep 数据整理完成"
    echo
done
