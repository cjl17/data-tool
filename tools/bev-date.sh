#!/bin/bash
source $(dirname /home/*/pix)/.bashrc
source $(dirname /home/*/pix)/.humble.sh 
 
SCRIPT_DIR=$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")" && pwd)
ros2bag_path_data=$1

if [ -z "$ros2bag_path_data" ]; then
  echo "用法: $0 <data_root>"
  echo "  - data_root: 数据根目录（包含 ok_data 目录）"
  exit 1
fi

# 检查 ok_data 目录是否存在
if [ ! -d "$ros2bag_path_data/ok_data" ]; then
  echo "错误: 目录 $ros2bag_path_data/ok_data 不存在"
  exit 1
fi

# 自动列出所有存在的 sequence0000* 文件夹（在 ok_data 目录下）
SEQUENCE_OPTIONS=($(find "$ros2bag_path_data/ok_data" -maxdepth 1 -type d -name "sequence*" -exec basename {} \; | sort))

if [ ${#SEQUENCE_OPTIONS[@]} -eq 0 ]; then
  echo "错误: 在 $ros2bag_path_data/ok_data 下没有找到 sequence 目录"
  exit 1
fi

# 所有相机名称（新格式）
CAM_OPTIONS=("front_3mm" "rear_3mm" "front_left" "front_right" "rear_left" "rear_right")

# map_data_root 等于 data_root（输出在数据目录下）
map_data_root=$ros2bag_path_data

# 执行命令（所有组合执行一次）
for seq in "${SEQUENCE_OPTIONS[@]}"; do
  for cam_type in "${CAM_OPTIONS[@]}"; do
    echo "正在执行: sequence=${seq}, cam=${cam_type}"
    /usr/bin/python3 "$SCRIPT_DIR/pc_projection.py" "$ros2bag_path_data" "$seq" "$cam_type" "$map_data_root"
    if [ $? -ne 0 ]; then
      echo "错误: sequence=${seq}, cam=${cam_type} 处理失败"
    fi
  done
done

echo "所有处理完成！"

