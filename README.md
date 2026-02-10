# Data Tool 数据处理工具集

本仓库包含用于处理 ROS2 MCAP 数据、相机标定、定位数据导出和分析的完整工具集。

## 📁 目录结构

```
data-tool/
├── README.md                    # 本文件（总览）
├── tools/                       # 新版工具集（推荐使用）
│   ├── README.md               # 工具集详细说明
│   ├── extract_ros2_mcap_pcd_jpg1.py         # MCAP 数据导出工具
│   ├── 2hzduiqi.py                            # 点云和相机数据匹配与预处理
│   ├── export_localization_to_csv2.sh         # 定位数据导出工具
│   ├── analyze_localization_quality.sh        # 定位数据质量分析
│   ├── check_pcd.py                          # PCD/JPG 连续性检查
│   ├── pc_projection.py                      # 点云投影到相机图像
│   ├── copy-keyframe.sh                      # 拷贝关键帧数据（ok_data_2hz）
│   ├── copy-sweep.sh                         # 拷贝 sweep 数据（ok_data）
│   └── ...
└── old_tool/                    # 旧版工具集（兼容性保留）
    ├── README.md               # 旧工具详细说明
    ├── convert_to_tencent_format.py           # 相机标定格式转换
    ├── generate_extrinsics.py                 # 相机外参矩阵生成
    ├── db3_to_yaml.py                         # ROS2 Bag 转 YAML
    ├── rename_with_timestamp.py               # 批量文件重命名
    └── localization_exporter.py               # 实时定位数据导出
```

## 🚀 快速开始

### 新版工具集（推荐）

**主要功能**：
- ✅ 支持 MCAP 格式（ROS2 rosbag2）
- ✅ 并行处理，提高效率
- ✅ 每个 MCAP 文件独立处理
- ✅ 自动批量处理

**详细文档**：参见 [tools/README.md](./tools/README.md)

**快速使用**：
```bash
# 1. 导出图像和点云（从 MCAP 文件）
python3 tools/extract_ros2_mcap_pcd_jpg1.py /path/to/perception_data_20260129113410 --jobs 8

# 2. 匹配点云和相机数据，生成训练序列
python3 tools/2hzduiqi.py /media/ipc/AQLoopCloseData2

# 3. 导出定位数据（每个 MCAP 一个 CSV）
bash tools/export_localization_to_csv2.sh /path/to/perception_data_20260129113410

# 4. 分析定位数据质量
./tools/analyze_localization_quality.sh /media/ipc/AQLoopCloseData1/perception_csv

# 5. 点云投影到相机图像（可选）
python3 tools/pc_projection.py /media/ipc/AQLoopCloseData2

# 6. 拷贝关键帧数据（可选，需要修改脚本中的路径）
bash tools/copy-keyframe.sh

# 7. 拷贝 sweep 数据（可选）
bash tools/copy-sweep.sh
```

### 旧版工具集

**主要功能**：
- 相机标定数据处理
- ROS2 db3 bag 文件处理
- 实时定位数据导出

**详细文档**：参见 [old_tool/README.md](./old_tool/README.md)

**快速使用**：
```bash
# 相机标定格式转换
python3 old_tool/convert_to_tencent_format.py input.yaml output.json

# 生成相机外参矩阵
python3 old_tool/generate_extrinsics.py camera0 output.json

# ROS2 Bag 转 YAML
python3 old_tool/db3_to_yaml.py /path/to/rosbag.db3
```

---

## 📊 工具对比

### 新版 vs 旧版工具

| 功能 | 新版工具 | 旧版工具 | 推荐 |
|------|---------|---------|------|
| **MCAP 格式支持** | ✅ | ❌ | 新版 |
| **并行处理** | ✅ | ❌ | 新版 |
| **每个文件独立 CSV** | ✅ | ❌ | 新版 |
| **db3 格式支持** | ❌ | ✅ | 旧版 |
| **实时数据导出** | ❌ | ✅ | 旧版 |
| **相机标定处理** | ❌ | ✅ | 旧版 |

### 定位数据导出工具对比

| 特性 | `tools/export_localization_to_csv.py.sh` | `old_tool/localization_exporter.py` | `old_tool/db3_to_yaml.py` |
|------|----------------------------------------|-------------------------------------|---------------------------|
| **输入格式** | MCAP | ROS2 话题（实时） | db3 bag |
| **输出格式** | CSV（每个文件独立） | CSV（单个文件） | YAML（10 Hz） |
| **处理方式** | 离线批量处理 | 实时订阅 | 离线处理 |
| **推荐场景** | 批量处理 MCAP 文件 | 实时数据采集 | 处理旧版 db3 bag |

---

## 🛠️ 完整工作流程

### 场景 1：处理 MCAP 数据（推荐流程）

```bash
# 1. 从 MCAP 导出图像和点云
python3 tools/extract_ros2_mcap_pcd_jpg1.py \
  /path/to/perception_data_20260129113410 \
  --jobs 8
# 输出到: /path/to/perception_data_20260129113410/raw_data/

# 2. 匹配点云和相机数据，生成训练序列
python3 tools/2hzduiqi.py /media/ipc/AQLoopCloseData2
# 输出到: /media/ipc/AQLoopCloseData2/ok_data/ 和 ok_data_2hz/

# 3. 导出定位数据到 CSV（每个 MCAP 一个 CSV）
bash tools/export_localization_to_csv2.sh \
  /path/to/perception_data_20260129113410

# 4. 分析定位数据质量
./tools/analyze_localization_quality.sh \
  /media/ipc/AQLoopCloseData1/perception_csv

# 5. 检查导出的 PCD/JPG 连续性
python3 tools/check_pcd.py \
  /path/to/perception_data_20260129113410/raw_data

# 6. 点云投影到相机图像（可选）
python3 tools/pc_projection.py /media/ipc/AQLoopCloseData2

# 7. 拷贝关键帧数据（可选，需要修改脚本中的路径）
bash tools/copy-keyframe.sh

# 8. 拷贝 sweep 数据（可选）
bash tools/copy-sweep.sh
```

### 场景 2：相机标定数据处理

```bash
# 1. 转换相机标定文件格式
python3 old_tool/convert_to_tencent_format.py \
  camera_calibration.yaml output.json

# 2. 生成相机外参变换矩阵
python3 old_tool/generate_extrinsics.py \
  camera0 camera0_extrinsics.json
```

### 场景 3：处理旧版 db3 Bag 文件

```bash
# 从 db3 bag 文件提取定位数据
python3 old_tool/db3_to_yaml.py /path/to/rosbag.db3
# 输出到: /path/to/rosbag.db3 所在目录/localization/
```

### 场景 4：实时定位数据采集

```bash
# 启动实时导出节点
python3 old_tool/localization_exporter.py /tmp/localization_data

# 在另一个终端启用保存
ros2 service call /localization_exporter/set_enable_save \
  std_srvs/srv/SetBool "{data: true}"
```

### 场景 5：点云投影与数据整理

```bash
# 1. 点云投影到相机图像
# 注意：需要先修改脚本中的 dt 和 all_gen 变量来配置时间补偿和生成帧数
python3 tools/pc_projection.py /media/ipc/AQLoopCloseData2
# 输出：在 ok_data/sequence*/ 目录下生成各相机的投影图像

# 2. 拷贝关键帧数据（ok_data_2hz）
# 注意：需要先修改脚本中的 INPUT_FIRST_DIR 和 OUTPUT_DIR
bash tools/copy-keyframe.sh
# 输出：将 ok_data_2hz 数据拷贝到指定输出目录

# 3. 拷贝 sweep 数据（ok_data）
bash tools/copy-sweep.sh
# 输出：自动处理所有 first* 目录，将 ok_data 拷贝到 first*/sweep/ 目录
```

---

## 📦 依赖要求

### 通用依赖

```bash
# Python 基础依赖
pip install numpy pyyaml pandas matplotlib opencv-python scipy pillow

# 点云处理依赖（pc_projection.py 需要）
pip install open3d python-pcl  # 或使用 open3d 的 Tensor API
```

### ROS2 依赖（部分工具需要）

```bash
# 确保 ROS2 环境已配置
source /opt/ros/humble/setup.bash

# 安装 ROS2 Python 包
pip install rosbags rosbag2-py

# 如果需要定位导出工具，需要编译消息包
cd ~/ros2_ws
colcon build --packages-select pix_robobus_driver_msgs
source install/setup.bash
```

### 系统要求

- **Python**: 3.8+
- **ROS2**: Humble 或兼容版本（部分工具需要）
- **Bash**: 用于 `.sh` 脚本
- **GNU parallel**: 用于并行处理（`copy-keyframe.sh`、`copy-sweep.sh` 需要）
  - 安装：`sudo apt install parallel` 或 `brew install parallel`
- **rsync**: 用于文件拷贝（`copy-keyframe.sh`、`copy-sweep.sh` 需要）
  - 通常系统已预装

---

## 📚 详细文档

### 新版工具集
- 📖 [tools/README.md](./tools/README.md) - 新版工具集完整文档
- 📖 [tools/README_extract_ros2_mcap_pcd_jpg.md](./tools/README_extract_ros2_mcap_pcd_jpg.md) - MCAP 导出工具详细文档

### 旧版工具集
- 📖 [old_tool/README.md](./old_tool/README.md) - 旧版工具集完整文档

---

## 🔍 工具选择指南

### 我应该使用哪个工具？

**处理 MCAP 格式数据** → 使用 `tools/` 目录下的工具
- `extract_ros2_mcap_pcd_jpg1.py` - 导出图像和点云
- `export_localization_to_csv2.sh` - 导出定位数据
- `2hzduiqi.py` - 点云和相机数据匹配与预处理

**数据整理与拷贝** → 使用 `tools/` 目录下的工具
- `copy-keyframe.sh` - 拷贝关键帧数据（ok_data_2hz）
- `copy-sweep.sh` - 拷贝 sweep 数据（ok_data）

**点云处理** → 使用 `tools/` 目录下的工具
- `pc_projection.py` - 点云投影到相机图像
- `check_pcd.py` - PCD/JPG 连续性检查

**处理 db3 格式数据** → 使用 `old_tool/` 目录下的工具
- `db3_to_yaml.py` - 转换 db3 bag 到 YAML

**实时数据采集** → 使用 `old_tool/localization_exporter.py`
- 需要实时订阅 ROS2 话题

**相机标定处理** → 使用 `old_tool/` 目录下的工具
- `convert_to_tencent_format.py` - 格式转换
- `generate_extrinsics.py` - 外参矩阵生成

**点云投影** → 使用 `tools/pc_projection.py`
- 将点云投影到相机图像上
- 支持时间补偿配置
- 自动处理所有 sequence 目录和相机

**数据整理** → 使用 `tools/` 目录下的拷贝工具
- `copy-keyframe.sh` - 拷贝关键帧数据（ok_data_2hz），需要修改脚本中的路径
- `copy-sweep.sh` - 拷贝 sweep 数据（ok_data），自动处理所有 first* 目录

---

## ⚠️ 注意事项

### 通用注意事项
- 处理大量数据前建议先用小样本测试
- 确保有足够的磁盘空间
- 建议定期备份重要数据

### 新版工具注意事项
- 需要 ROS2 环境和相关消息包
- 并行处理时注意内存使用
- 每个 MCAP 文件会生成独立的输出

### 旧版工具注意事项
- 部分工具需要修改硬编码路径
- db3 格式工具需要确保 bag 文件包含必需话题
- 实时导出工具需要在有 ROS2 话题的环境下运行

---

## 🔄 版本说明

### 新版工具（tools/）
- ✅ 支持最新的 MCAP 格式
- ✅ 并行处理，性能更好
- ✅ 每个文件独立处理，更灵活
- ✅ 持续更新和维护

### 旧版工具（old_tool/）
- ⚠️ 保留用于兼容性
- ⚠️ 主要用于处理旧格式数据
- ⚠️ 建议迁移到新版工具

---

## 📝 更新日志

### 2025-02-XX
- ✅ 添加 `pc_projection.py` - 点云投影工具
- ✅ 添加 `copy-keyframe.sh` - 关键帧数据拷贝工具
- ✅ 添加 `copy-sweep.sh` - sweep 数据拷贝工具
- ✅ 更新工具列表和工作流程

### 2025-02-02
- ✅ 创建总 README 文档
- ✅ 更新 `old_tool/README.md`，添加 `rename_with_timestamp.py` 说明
- ✅ 添加工具对比和选择指南
- ✅ 添加完整的工作流程示例

### 2025-01-30
- ✅ 更新 `tools/README.md`，添加详细依赖说明
- ✅ 更新 `analyze_localization_quality.sh`，支持目录参数
- ✅ 更新 `extract_ros2_mcap_pcd_jpg.py`，支持并行处理

---

## 🤝 贡献与支持

如有问题或建议：
1. 查看各子目录的详细 README
2. 检查脚本代码注释
3. 联系维护人员

---

## 📄 许可证

请查看各工具脚本的许可证声明。

---

**推荐使用新版工具集（`tools/`）进行数据处理，旧版工具（`old_tool/`）保留用于兼容性。**

