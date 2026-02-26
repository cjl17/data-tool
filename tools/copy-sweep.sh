#!/bin/bash
set -euo pipefail
shopt -s nullglob

########################################
# 用户配置：输入输出路径
########################################
# 输入 first 目录（包含 perception_data_*）
INPUT_FIRST_DIR="/media/ipc/AQLoopCloseData/first_20260210100908"

# 输出目录（生成 sweep/bev_data_*）
OUTPUT_DIR="/media/ipc/AQLoopCloseData/first_20260210100908/sweep"

mkdir -p "$OUTPUT_DIR"

echo "输入目录:  $INPUT_FIRST_DIR"
echo "输出目录:  $OUTPUT_DIR"
echo

########################################
# 单个 ok_data 处理函数
########################################
process_one() {
    ok_data_dir="$1"
    output_root="$2"

    perception_dir="$(dirname "$ok_data_dir")"
    perception_name="$(basename "$perception_dir")"

    # 解析 perception_data_{timestamp}_{index}
    [[ "$perception_name" =~ perception_data_([0-9]+)_([0-9]+) ]] || return

    timestamp="${BASH_REMATCH[1]}"
    index="${BASH_REMATCH[2]}"
    sequence_name=$(printf "sequence%05d" "$index")

    target_dir="${output_root}/bev_data_${timestamp}_${index}_${sequence_name}"

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

    # --- 平铺唯一 sequence* 子目录 ---
    seq_subdir=$(find "$target_dir" -maxdepth 1 -type d -name "sequence*" | head -n1)
    if [ -n "$seq_subdir" ] && [ "$seq_subdir" != "$target_dir" ]; then
        echo "    平铺 ${seq_subdir} -> ${target_dir}"
        shopt -s dotglob
        mv "$seq_subdir"/* "$target_dir"/
        shopt -u dotglob
        rmdir "$seq_subdir"
    fi

    # --- 拷贝 CSV 到 localization 文件夹 ---
    csv="${perception_dir}/localization_${index}.csv"
    if [ -f "$csv" ]; then
        loc_dir="${target_dir}/localization"
        mkdir -p "$loc_dir"
        cp "$csv" "$loc_dir/"
        echo "    拷贝 CSV -> ${loc_dir}/"
    fi

    echo "✅ 完成: $(basename "$target_dir")"
    echo
}

export -f process_one

########################################
# 主循环：处理所有 perception_data_* 的 ok_data
########################################
find "$INPUT_FIRST_DIR" -maxdepth 2 -path "*/perception_data_*/ok_data" -type d \
| parallel --line-buffer -j 2 process_one {} "$OUTPUT_DIR"

echo "🎉 所有 sweep 数据整理完成"

