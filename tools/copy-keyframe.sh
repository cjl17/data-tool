#!/bin/bash
set -euo pipefail
shopt -s nullglob

########################################
# 【用户只需要改这里】
########################################

INPUT_FIRST_DIR="/media/ipc/AQLoopCloseData2/first_20260205125341/0212check"
OUTPUT_DIR="/media/ipc/AQLoopCloseData2/first_20260205125341/pix-20260112checks-a"

########################################
# 安全检查
########################################
if [ ! -d "$INPUT_FIRST_DIR" ]; then
    echo "❌ 输入目录不存在: $INPUT_FIRST_DIR"
    exit 1
fi

mkdir -p "$OUTPUT_DIR"

echo "=============================="
echo "仅拷贝 ok_data_2hz"
echo "输入: $INPUT_FIRST_DIR"
echo "输出: $OUTPUT_DIR"
echo "=============================="
echo

########################################
# 单个 ok_data_2hz 处理函数
########################################
process_one_2hz() {
    ok_data_2hz_dir="$1"
    output_root="$2"

    # 父目录必须是 perception_data_xxx_x
    perception_dir="$(dirname "$ok_data_2hz_dir")"
    perception_name="$(basename "$perception_dir")"

    [[ "$perception_name" =~ ^perception_data_([0-9]+)_([0-9]+)$ ]] || {
        echo "⚠️ 非法目录，跳过: $ok_data_2hz_dir"
        return
    }

    timestamp="${BASH_REMATCH[1]}"
    index="${BASH_REMATCH[2]}"
    sequence_name=$(printf "sequence%05d" "$index")

    target_dir="${output_root}/bev_data_${timestamp}_${index}_${sequence_name}"

    if [ -e "$target_dir" ]; then
        echo "⚠️ 已存在，跳过: $target_dir"
        return
    fi

    mkdir -p "$target_dir"

    echo "➡️  [2Hz] $(basename "$perception_dir")"

    # 找到 ok_data_2hz 下唯一 sequence* 目录
    seq_subdir=("$ok_data_2hz_dir"/sequence*/)
    seq_subdir="${seq_subdir%/}"   # 去掉末尾斜杠

    if [ -d "$seq_subdir" ]; then
        # 拷贝 sequence 下的内容到 bev_data 目录，不保留 sequence 层
        rsync -rL \
            --whole-file \
            --inplace \
            --no-perms --no-owner --no-group \
            --omit-dir-times \
            --info=progress2,stats1 \
            "${seq_subdir}/" "${target_dir}/"
    else
        # 万一没有 sequence 目录，直接拷贝 ok_data_2hz 内容
        rsync -rL \
            --whole-file \
            --inplace \
            --no-perms --no-owner --no-group \
            --omit-dir-times \
            --info=progress2,stats1 \
            "${ok_data_2hz_dir}/" "${target_dir}/"
    fi

    echo "✅ 完成: $(basename "$target_dir")"
    echo
}

export -f process_one_2hz

########################################
# 主逻辑：严格只查找 ok_data_2hz
########################################
find "$INPUT_FIRST_DIR" \
    -maxdepth 2 \
    -type d \
    -path "*/perception_data_*/ok_data_2hz" \
| parallel --line-buffer -j 2 process_one_2hz {} "$OUTPUT_DIR"

echo "🎉 所有 ok_data_2hz 拷贝完成"

