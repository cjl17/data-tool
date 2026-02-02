#!/usr/bin/env python3
from pathlib import Path
import sys

# -----------------------------
# 获取文件夹路径
# -----------------------------
if len(sys.argv) < 2:
    print("用法: python3 rename_files.py <文件夹路径>")
    sys.exit(1)

folder = Path(sys.argv[1])

# -----------------------------
# 配置初始编号和步长
# -----------------------------
start_index = 175636611607   # 第一个文件编号
step = 10                     # 每个文件递增步长

# 获取文件夹名字，用于文件名前缀
folder_name = folder.name

# -----------------------------
# 获取文件列表并排序
# -----------------------------
files = sorted(folder.iterdir())

# -----------------------------
# 第一步：临时重命名，避免覆盖
# -----------------------------
tmp_files = []
for file in files:
    tmp_name = file.parent / f"{file.stem}_tmp{file.suffix}"
    file.rename(tmp_name)
    tmp_files.append(tmp_name)

# -----------------------------
# 第二步：正式重命名
# -----------------------------
for idx, tmp_file in enumerate(tmp_files):
    new_index = start_index + idx * step
    new_name = tmp_file.parent / f"{new_index}.jpg"
    tmp_file.rename(new_name)
    print(f"{tmp_file.name} -> {new_name.name}")

print("✅ 批量重命名完成！")

