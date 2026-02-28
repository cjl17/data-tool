# Tools 工具集说明

本目录包含用于处理 ROS2 MCAP 数据的各种工具脚本。

## 快速索引

| 工具 | 功能 | 输入 | 输出 |
|------|------|------|------|
| `extract_ros2_mcap_pcd_jpg1.py` | 导出图像和点云 | MCAP 文件/目录 | JPG 图片 + PCD 点云 |
| `export_localization_to_csv2.sh` | 导出定位数据 | MCAP 文件/目录 | CSV 文件（每个 MCAP 一个） |
| `analyze_localization_quality.sh` | 分析定位数据质量 | CSV 文件/目录 | 异常报告 + 可视化图表 |
| `check_pcd.py` | 检查导出文件连续性 | first_* 目录 | 连续性检查报告 |
| `2hzduiqi3.py` | 点云和相机数据匹配与预处理 | AQLoopCloseData 根目录 | ok_data/ 和 ok_data_2hz/ |
| `pc_projection4.py` | 点云投影到相机图像 | AQLoopCloseData 根目录 | 投影图像 |
| `jiance.py` | YOLO 目标检测与类别统计 | first_* 目录 | class_ratio_report.csv |
| `neieaican.py` | 相机内外参提取 | MCAP 文件 | JSON 参数文件 |
| `find_mcap_by_time.py` | 按时间查找 MCAP 文件 | perception_data 目录 + 时间 | MCAP 文件名 |

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
python3 tools/check_pcd.py <first_dir>
```

**示例**：
```bash
# 检查 first_* 目录下的所有 perception_data
python3 tools/check_pcd.py /path/to/first_20260210100908
```

**输出**：
- 每个文件夹的统计信息
- 时间戳异常报告
- 序号缺失报告
- 时间间隔异常报告
- 生成 `check_report.txt` 文件

---

### 6. `2hzduiqi3.py` - 点云和相机数据匹配与预处理工具

**功能**：匹配点云和相机数据，生成训练序列

- 自动查找所有 `first_*` 目录下的 `perception_data_*`
- 时间对齐点云和相机图像
- 生成 `ok_data`（所有匹配帧）和 `ok_data_2hz`（2Hz 抽帧）
- 使用符号链接节省空间

**使用方法**：
```bash
python3 tools/2hzduiqi3.py /media/pix/AQLoopCloseData
```

**输出**：
- `perception_data_*/ok_data/sequence00000/` - 所有匹配帧
- `perception_data_*/ok_data_2hz/sequence00000/` - 2Hz 抽帧

---

### 7. `pc_projection4.py` - 点云投影到相机图像工具

**功能**：将点云投影到相机图像上，生成可视化图像

- 支持所有相机（front_3mm, rear_3mm, front_left, front_right, rear_left, rear_right）
- 自动处理所有 sequence 目录
- 支持时间补偿配置
- 使用颜色映射显示点云强度

**使用方法**：
```bash
# 处理单个 perception_data 目录
python3 tools/pc_projection4.py /path/to/perception_data_20260210100908

# 批量处理所有 first_* 目录
python3 tools/pc_projection4.py /media/pix/AQLoopCloseData

# 随机抽样处理（可选）
python3 tools/pc_projection4.py /media/pix/AQLoopCloseData --sample 10
python3 tools/pc_projection4.py /media/pix/AQLoopCloseData --ratio 0.1
```

**配置**：
- 修改脚本中的 `dt` 变量设置时间补偿
- 修改脚本中的 `all_gen` 变量控制是否生成所有帧

**输出**：
- `perception_data_*/projection/sequence*/{camera}/map*.jpg`

---

### 8. `jiance.py` - YOLO 目标检测与类别统计工具

**功能**：使用 YOLO 模型检测图像中的目标类别，统计各类别出现的帧占比

- 使用 YOLOv8s 模型进行目标检测
- 统计目标类别：car, truck, bus, person, bicycle, motorcycle
- 批量处理所有 `first_*` 目录下的 `perception_data_*`
- 输出 CSV 格式的统计报告

**使用方法**：
```bash
python3 tools/jiance.py
```

**配置**：
- 修改脚本中的 `ROOT_DIR` 设置数据根目录
- 修改脚本中的 `MODEL_PATH` 设置 YOLO 模型路径（默认：`yolov8s.pt`）
- 修改脚本中的 `BATCH_SIZE` 设置批处理大小（默认：120）
- 修改脚本中的 `TARGET_CLASSES` 设置目标类别

**输出**：
- `/media/pix/AQLoopCloseData/class_ratio_report.csv` - 包含每个 perception_data 目录的统计信息

**依赖**：
- `ultralytics` - YOLO 模型
- `torch` - PyTorch
- `tqdm` - 进度条

---

### 9. `neieaican.py` - 相机内外参提取工具

**功能**：从 MCAP 文件或 YAML 文件提取相机内外参

- 优先从 MCAP 文件的 TF 和 camera_info 读取参数
- 失败时自动回退到 YAML 文件
- 支持所有相机（front, rear, front_left, front_right, rear_left, rear_right）
- 生成 JSON 格式的参数文件

**使用方法**：
```bash
# 仅提供 mcap_path（自动推断 base_path，处理所有相机）
python3 tools/neieaican.py /path/to/perception_data_20260210100908/perception_data_20260210100908_0.mcap

# 提供 base_path 和 mcap_path（处理所有相机）
python3 tools/neieaican.py /path/to/base_path /path/to/mcap_file

# 处理单个相机
python3 tools/neieaican.py /path/to/base_path front /path/to/mcap_file

# 处理所有相机（显式指定）
python3 tools/neieaican.py /path/to/base_path all /path/to/mcap_file
```

**输出**：
- `perception_data_*/sensor_parameter/{camera_key}_{camera_name}.json`

**依赖**：
- `mcap` - MCAP 文件读取
- `mcap-ros2-support` - ROS2 消息解码

---

### 10. `find_mcap_by_time.py` - 按时间查找 MCAP 文件工具

**功能**：按时间查找对应的 MCAP 文件

- 自动建立时间索引（首次运行）
- 快速定位包含指定时间的 MCAP 文件
- 支持前后扩展 N 个分片

**使用方法**：
```bash
# 基本用法
python3 tools/find_mcap_by_time.py /path/to/perception_data_20260210100908 "2026-02-10 09:57:13"

# 前后扩展 2 个分片
python3 tools/find_mcap_by_time.py /path/to/perception_data_20260210100908 "2026-02-10 09:57:13" --expand 2

# 打印完整路径
python3 tools/find_mcap_by_time.py /path/to/perception_data_20260210100908 "2026-02-10 09:57:13" --print-full
```

**输出**：
- 打印匹配的 MCAP 文件名或完整路径

**依赖**：
- ROS2 环境（需要 `ros2 bag info` 命令）

---

## 工作流程示例

### 完整的数据处理流程

```bash
# 1. 从 MCAP 导出图像和点云
python3 tools/extract_ros2_mcap_pcd_jpg1.py \
  /path/to/perception_data_20260129113410 \
  --jobs 8

# 2. 匹配点云和相机数据，生成训练序列
python3 tools/2hzduiqi3.py /media/pix/AQLoopCloseData

# 3. 导出定位数据到 CSV（每个 MCAP 一个 CSV）
python3 tools/export_localization_to_csv2.sh \
  /path/to/perception_data_20260129113410

# 4. 分析定位数据质量
./tools/analyze_localization_quality.sh \
  /media/ipc/AQLoopCloseData1/perception_csv

# 5. 检查导出的 PCD/JPG 连续性
python3 tools/check_pcd.py \
  /path/to/first_20260210100908

# 6. 点云投影到相机图像（可选）
python3 tools/pc_projection4.py /media/pix/AQLoopCloseData

# 7. 提取相机内外参（可选）
python3 tools/neieaican.py \
  /path/to/perception_data_20260210100908/perception_data_20260210100908_0.mcap

# 8. YOLO 目标检测与类别统计（可选）
python3 tools/jiance.py
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
- `opencv-python` - 图像处理（pc_projection4.py 需要）
- `Pillow` - 图像处理（pc_projection4.py 需要）
- `scipy` - 科学计算（pc_projection4.py 需要）
- `open3d` - 点云处理（pc_projection4.py 需要）
- `mcap` - MCAP 文件读取（neieaican.py, find_mcap_by_time.py 需要）
- `mcap-ros2-support` - ROS2 消息解码（neieaican.py 需要）
- `ultralytics` - YOLO 模型（jiance.py 需要）
- `torch` - PyTorch（jiance.py 需要）
- `tqdm` - 进度条（jiance.py 需要）

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

