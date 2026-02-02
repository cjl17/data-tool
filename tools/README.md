# Tools 工具集说明

本目录包含用于处理 ROS2 MCAP 数据的各种工具脚本。

## 快速索引

| 工具 | 功能 | 输入 | 输出 |
|------|------|------|------|
| `extract_ros2_mcap_pcd_jpg.py` | 导出图像和点云 | MCAP 文件/目录 | JPG 图片 + PCD 点云 |
| `export_localization_to_csv.py.sh` | 导出定位数据 | MCAP 文件/目录 | CSV 文件（每个 MCAP 一个） |
| `localization_exporter_all.py` | 批量导出定位数据 | MCAP 文件/目录 | 单个合并的 CSV 文件 |
| `analyze_localization_quality.sh` | 分析定位数据质量 | CSV 文件/目录 | 异常报告 + 可视化图表 |
| `check_pcd.py` | 检查导出文件连续性 | 导出目录 | 连续性检查报告 |

## 工具列表

### 1. `extract_ros2_mcap_pcd_jpg.py` - MCAP 数据导出工具

**功能**：从 ROS2 rosbag2 (MCAP 格式) 导出图像和点云数据

- 导出 `sensor_msgs/msg/CompressedImage` → JPG 图片
- 导出 `sensor_msgs/msg/PointCloud2` → PCD 点云文件
- 支持并行处理多个 MCAP 文件
- 支持自动批量处理所有 `perception_data_*` 目录
- 每个 MCAP 文件独立生成 metadata.yaml

**详细文档**：参见 [README_extract_ros2_mcap_pcd_jpg.md](./README_extract_ros2_mcap_pcd_jpg.md)

**快速使用**：
```bash
# 自动处理当前目录下所有 perception_data_* 目录
python3 tools/extract_ros2_mcap_pcd_jpg.py

# 处理单个目录
python3 tools/extract_ros2_mcap_pcd_jpg.py /path/to/perception_data_20260129113410

# 并行处理（使用 8 个进程）
python3 tools/extract_ros2_mcap_pcd_jpg.py --jobs 8
```

---

### 2. `export_localization_to_csv.py.sh` - 定位数据导出工具（推荐）

**功能**：从 MCAP 文件导出定位相关数据到 CSV 格式

- **每个 MCAP 文件生成一个独立的 CSV 文件**
- 导出数据包括：
  - GPS 定位：经纬度、海拔、航向角
  - 速度：定位速度 (vx, vy, vz)、融合速度 (fused_vx, fused_vy, fused_vz)
  - IMU：角速度 (wx, wy, wz)、加速度 (ax, ay, az)
  - 轮速：四个轮子的转速
  - 时间戳同步和数据有效性检查

**输出格式**：
- CSV 文件名：`{mcap文件名}_{时间戳}.csv`
- 输出目录：`/media/ipc/AQLoopCloseData1/perception_csv/`

**使用方法**：
```bash
# 使用默认目录
python3 tools/export_localization_to_csv.py.sh

# 指定 MCAP 文件或目录
python3 tools/export_localization_to_csv.py.sh /path/to/perception_data_20260129113410

# 指定多个文件或目录
python3 tools/export_localization_to_csv.py.sh file1.mcap file2.mcap /path/to/dir
```

**注意事项**：
- 需要 ROS2 环境已配置（`source /opt/ros/humble/setup.bash`）
- 需要 `pix_robobus_driver_msgs` 消息包（如果缺少会报错）
- 每个 MCAP 文件处理前会重置状态，确保数据独立

**特性**：
- 自动时间戳同步和数据有效性检查
- 支持超时阈值配置（默认 30-50ms）
- 自动去重（1ms 内的重复时间戳）
- 每个文件处理前自动重置状态，确保数据独立

---

### 3. `localization_exporter_all.py` - 批量定位数据导出工具（旧版）

**功能**：批量导出定位数据，将所有 MCAP 文件合并到一个 CSV

**注意**：此工具将所有 MCAP 文件的数据合并到一个 CSV 文件。如果需要每个 MCAP 文件独立的 CSV，请使用 `export_localization_to_csv.py.sh`。

**使用方法**：
```bash
python3 tools/localization_exporter_all.py [mcap_file1] [mcap_file2] ...
```

---

### 4. `analyze_localization_quality.sh` - 定位数据质量分析工具

**功能**：分析定位 CSV 数据的质量，检测异常并生成报告

**分析内容**：
- 时间戳异常检测（时间倒退、采样过快/过慢）
- 速度跳变检测（定位速度和融合速度）
- IMU 和加速度异常检测
- 数据完整性统计
- 生成可视化图表和异常报告

**输出文件**：
- `localization_anomaly_report.csv` - 异常点详细报告
- `localization_anomaly_plot.png` - 可视化图表

**使用方法**：
```bash
# 使用默认目录（自动查找并合并所有 CSV）
./tools/analyze_localization_quality.sh

# 指定 CSV 文件目录
./tools/analyze_localization_quality.sh /media/ipc/AQLoopCloseData1/perception_csv

# 指定单个 CSV 文件
./tools/analyze_localization_quality.sh -f /path/to/single.csv

# 指定输出目录
./tools/analyze_localization_quality.sh -d /path/to/csvs -o /path/to/output
```

**参数说明**：
- `-f, --file`: 指定单个 CSV 文件
- `-d, --dir`: 指定包含 CSV 文件的目录（默认：`/media/ipc/AQLoopCloseData1/perception_csv`）
- `-o, --outdir`: 输出目录（默认：`/media/ipc/AQLoopCloseData1/perception_csv/`）

**特性**：
- 自动查找并合并目录中的所有 CSV 文件
- 智能阈值计算（基于数据统计和车辆动力学）
- 多维度异常检测
- 生成详细的可视化报告

---

### 5. `check_pcd.py` - PCD/JPG 连续性检查工具

**功能**：检查导出的 PCD 和 JPG 文件的连续性和完整性

**检查内容**：
- 文件数量统计
- 时间戳单调性检查
- 序号缺失检测
- 时间戳间隔异常检测（可能的丢帧）

**使用方法**：
```bash
python3 tools/check_pcd.py <export_dir>
```

**示例**：
```bash
# 检查导出目录
python3 tools/check_pcd.py /path/to/export_pcd_jpg
```

**输出**：
- 每个 topic 子目录的统计信息
- 时间戳异常报告
- 序号缺失报告
- 时间间隔异常报告

---

## 工作流程示例

### 完整的数据处理流程

```bash
# 1. 从 MCAP 导出图像和点云
python3 tools/extract_ros2_mcap_pcd_jpg.py \
  /path/to/perception_data_20260129113410 \
  --jobs 8

# 2. 导出定位数据到 CSV（每个 MCAP 一个 CSV）
python3 tools/export_localization_to_csv.py.sh \
  /path/to/perception_data_20260129113410

# 3. 分析定位数据质量
./tools/analyze_localization_quality.sh \
  /media/ipc/AQLoopCloseData1/perception_csv

# 4. 检查导出的 PCD/JPG 连续性
python3 tools/check_pcd.py \
  /path/to/perception_data_20260129113410/export_pcd_jpg
```

### 批量处理多个目录

```bash
# 1. 批量导出所有 perception_data_* 目录的图像和点云
python3 tools/extract_ros2_mcap_pcd_jpg.py --jobs 8

# 2. 批量导出定位数据
for dir in /path/to/data/perception_data_*; do
    python3 tools/export_localization_to_csv.py.sh "$dir"
done

# 3. 分析所有 CSV 文件
./tools/analyze_localization_quality.sh /media/ipc/AQLoopCloseData1/perception_csv
```

---

## 依赖要求

### Python 依赖

#### 基础依赖
- `rosbags` - ROS2 bag 文件读取
- `rosbag2_py` - ROS2 bag2 API
- `rclpy` - ROS2 Python 客户端库
- `pandas` - 数据分析
- `matplotlib` - 数据可视化
- `numpy` - 数值计算
- `yaml` - YAML 文件解析

#### ROS2 消息包依赖
- `pix_robobus_driver_msgs` - 车辆驱动消息包（用于轮速数据）
  - 如果缺少此包，`export_localization_to_csv.py.sh` 和 `localization_exporter_all.py` 会报错
  - 需要从 ROS2 工作空间编译安装

### 系统要求
- ROS2 环境（Humble 或兼容版本）
- Python 3.8+
- Bash shell（用于 `.sh` 脚本）

### 安装依赖

```bash
# 安装 Python 基础依赖
pip3 install rosbags rosbag2-py pandas matplotlib numpy pyyaml

# 确保 ROS2 环境已配置
source /opt/ros/humble/setup.bash  # 或您的 ROS2 安装路径

# 如果使用定位导出工具，需要编译 pix_robobus_driver_msgs
# 在工作空间中：
cd ~/ros2_ws
colcon build --packages-select pix_robobus_driver_msgs
source install/setup.bash
```

---

## 文件结构说明

```
tools/
├── README.md                              # 本文件（工具集总览）
├── README_extract_ros2_mcap_pcd_jpg.md    # extract_ros2_mcap_pcd_jpg.py 详细文档
├── extract_ros2_mcap_pcd_jpg.py          # MCAP 数据导出工具
├── export_localization_to_csv.py.sh      # 定位数据导出工具（推荐）
├── localization_exporter_all.py          # 批量定位数据导出工具（旧版）
├── analyze_localization_quality.sh       # 定位数据质量分析工具
└── check_pcd.py                          # PCD/JPG 连续性检查工具
```

---

## 常见问题

### Q: 每个 MCAP 文件都会生成独立的 CSV 吗？

**A**: 是的。`export_localization_to_csv.py.sh` 会为每个 MCAP 文件生成一个独立的 CSV 文件，文件名格式为 `{mcap文件名}_{时间戳}.csv`。

### Q: 如何合并多个 CSV 文件进行分析？

**A**: `analyze_localization_quality.sh` 会自动查找并合并指定目录下的所有 CSV 文件进行分析。

### Q: 并行处理时 metadata.yaml 会冲突吗？

**A**: 不会。每个 MCAP 文件会在临时目录中生成独立的 metadata.yaml，互不干扰。

### Q: 如何跳过已处理的文件？

**A**: `extract_ros2_mcap_pcd_jpg.py` 会自动检查输出目录，如果已存在 `export_stats.json` 且未使用 `--overwrite`，会自动跳过。

### Q: 运行 `export_localization_to_csv.py.sh` 时出现 `ModuleNotFoundError: No module named 'pix_robobus_driver_msgs'` 错误？

**A**: 这是因为缺少 ROS2 消息包。解决方法：
1. 确保 ROS2 环境已正确配置：`source /opt/ros/humble/setup.bash`
2. 在工作空间中编译安装 `pix_robobus_driver_msgs` 包
3. 确保运行脚本前已 source 工作空间的 setup.bash

### Q: 脚本执行权限问题？

**A**: 确保脚本有执行权限：
```bash
chmod +x tools/*.sh
chmod +x tools/*.py
```

---

## 更新日志

### 2025-02-02
- 更新 README：添加详细的依赖说明和错误处理指南
- 添加 `pix_robobus_driver_msgs` 依赖说明
- 添加常见问题解答（模块缺失、权限问题等）

### 2025-01-30
- 更新 `analyze_localization_quality.sh`：支持目录参数，自动查找和合并 CSV 文件
- 更新 `extract_ros2_mcap_pcd_jpg.py`：支持并行处理，每个 MCAP 文件独立生成 metadata.yaml
- 添加完整的工具集 README 文档

---

## 联系与支持

如有问题或建议，请查看各工具的详细文档或联系维护人员。

