# Old Tools 旧版工具集说明

本目录包含旧版的数据处理工具脚本，主要用于相机标定、坐标变换计算和 ROS2 定位数据导出。

## 工具列表

### 1. `convert_to_tencent_format.py` - 相机标定格式转换工具

**功能**：将相机标定的 YAML 文件转换为腾讯格式的 JSON 文件

- 提取相机内参矩阵（intrinsic）
- 提取畸变系数（distortion）
- 保留外参（extrinsic，可选）
- 保留平移参数（translation，可选）

**使用方法**：
```bash
python3 convert_to_tencent_format.py input.yaml output.json
```

**参数说明**：
- `input.yaml`：相机标定 YAML 文件
- `output.json`：输出的 JSON 文件路径

**输出格式**：
```json
{
  "extrinsic": [...],
  "intrinsic": [...],
  "distortion": [...],
  "translation": [...]
}
```

**注意事项**：
- 如果 YAML 中缺少外参或平移参数，会自动写入 `null`
- 使用 `yaml.safe_load` 保障安全

---

### 2. `generate_extrinsics.py` - 相机外参变换矩阵生成工具

**功能**：计算相机到车辆基准坐标系的 4×4 齐次变换矩阵

**工作流程**：
1. 从 `sensors_calibration.yaml` 读取 `base_link → sensor_kit_base_link` 的平移和旋转
2. 从 `sensor_kit_calibration.yaml` 读取 `sensor_kit_base_link → cameraX/camera_link` 的平移和旋转
3. 合成 `base_link → camera_link` 变换并求逆，得到 `camera_link → base_link` 矩阵
4. 将结果保存到指定的 JSON 文件

**使用方法**：
```bash
python3 generate_extrinsics.py camera6 output.json
```

**参数说明**：
- `camera6`：相机编号（可替换为 camera0、camera1 等）
- `output.json`：结果保存路径（JSON 文件需已存在）

**输出示例**：
```
camera6 -> Camera to Base Transformation Matrix:
['1.000', '0.000', '0.000', '0.123']
['0.000', '1.000', '0.000', '-0.456']
['0.000', '0.000', '1.000', '0.789']
['0.000', '0.000', '0.000', '1.000']
```

**JSON 输出格式**：
```json
{
  "extrinsic": [16个浮点数...],
  "translation": [x, y, z],
  ...
}
```

**注意事项**：
- 需要修改脚本中的 YAML 文件路径（第85-86行）
- 确保 `sensors_calibration.yaml` 与 `sensor_kit_calibration.yaml` 路径正确
- 角度单位：弧度
- 输出 JSON 文件需要预先存在（脚本会读取并更新）

---

### 3. `db3_to_yaml.py` - ROS2 Bag 文件转 YAML 工具

**功能**：从 ROS2 db3 bag 文件提取定位数据并转换为 YAML 格式

**提取数据**：
- GPS 位置：经纬度、海拔
- 姿态：四元数方向
- 速度：线速度和角速度（转换到 MAP 坐标系）
- IMU：角速度（可选）
- 协方差矩阵：位置和速度协方差

**输出格式**：
- 按时间戳生成 YAML 文件（10 Hz）
- 每个文件夹包含 `DATA_PER_FOLDER` 条数据（默认 200 条）
- 文件命名：`{timestamp*100}.yaml`

**使用方法**：
```bash
python3 db3_to_yaml.py your.db3
```

**参数说明**：
- `your.db3`：ROS2 bag 文件路径（db3 格式）

**输出目录**：
- 输出到 bag 文件所在目录下的 `localization/` 文件夹
- 按序号创建子文件夹：`0/`, `1/`, `2/`, ...

**YAML 文件结构**：
```yaml
header:
  frameId: vehicle-lidar
  timestampSec: 1234567890.12
pose:
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
  position:
    x: 经度
    y: 纬度
    z: 海拔
vel:
  x: MAP坐标系下的x速度
  y: MAP坐标系下的y速度
  z: z速度
acc:
  x: MAP坐标系下的x加速度
  y: MAP坐标系下的y加速度
  z: z加速度
angularV:
  x: 角速度x
  y: 角速度y
  z: 角速度z
posCov: [36个协方差值]
velCov: [9个协方差值]
```

**订阅话题**：
- `/sensing/gnss/fix` - GPS 位置（必需）
- `/sensing/gnss/pose_with_covariance` - GPS 姿态（必需）
- `/localization/twist_estimator/twist_with_covariance` - 速度（必需）
- `/sensing/imu/imu_data` - IMU 数据（可选）

**注意事项**：
- 需要 ROS2 环境（`rclpy`）
- 需要 `sensor_msgs` 和 `geometry_msgs` 消息包
- 速度数据会自动从自车坐标系转换到 MAP 坐标系
- 如果缺少必需话题，程序会退出并报错

---

### 4. `localization_exporter.py` - ROS2 实时定位数据导出工具

**功能**：ROS2 节点，实时订阅定位话题并记录到 CSV 文件

**订阅话题**：
| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/sensing/gnss/fix` | `sensor_msgs/NavSatFix` | GPS 经纬度与高度 |
| `/localization/twist_estimator/twist_with_covariance` | `geometry_msgs/TwistWithCovarianceStamped` | 线速度与角速度 |

**服务接口**：
| 服务名称 | 类型 | 功能 |
|----------|------|------|
| `/localization_exporter/set_enable_save` | `std_srvs/SetBool` | 开启/关闭数据保存 |

**CSV 格式**：
```csv
timestamp_ms, lat, lon, alt, vx, vy, vz, wx, wy, wz
```
- `v*` 为线速度 (m/s)
- `w*` 为角速度 (rad/s)
- `timestamp_ms` 为毫秒时间戳

**使用方法**：
```bash
# 启动节点（可带输出目录参数）
python3 localization_exporter.py /tmp/localization_data

# 或在 ROS2 包中
ros2 run your_pkg localization_exporter.py /tmp/localization_data

# 启用保存（通过服务调用）
ros2 service call /localization_exporter/set_enable_save std_srvs/srv/SetBool "{data: true}"

# 禁用保存
ros2 service call /localization_exporter/set_enable_save std_srvs/srv/SetBool "{data: false}"
```

**输出文件**：
- `localization.csv`：实时数据记录
- `localization.log`：日志文件，记录状态与统计

**特性**：
- 定时器频率：100 Hz
- 每 1000 条数据自动在日志中输出一次进度
- 退出时自动关闭文件句柄并写入日志
- 可通过服务动态启停数据保存

**注意事项**：
- 需在有 ROS2 话题发布的环境下运行
- 需要 ROS2 环境（`rclpy`）
- 需要 `sensor_msgs` 和 `geometry_msgs` 消息包
- 如果命令行未提供输出目录，会使用 ROS2 参数 `output_directory`（默认：`/tmp/localization_data`）

---

## 依赖要求

### Python 基础依赖
```bash
pip install numpy pyyaml
```

### ROS2 依赖（部分工具需要）
- ROS2 Humble 或兼容版本
- `rclpy` - ROS2 Python 客户端库
- `sensor_msgs` - 传感器消息包
- `geometry_msgs` - 几何消息包
- `std_srvs` - 标准服务包（仅 `localization_exporter.py` 需要）

### 安装 ROS2 依赖
```bash
# 确保 ROS2 环境已配置
source /opt/ros/humble/setup.bash  # 或您的 ROS2 安装路径

# 如果使用 ROS2 工具，确保消息包已安装
# 通常 ROS2 安装包已包含基础消息包
```

---

## 文件结构

```
old_tool/
├── README.md                      # 本文件
├── convert_to_tencent_format.py   # 相机标定格式转换工具
├── generate_extrinsics.py         # 相机外参变换矩阵生成工具
├── db3_to_yaml.py                 # ROS2 Bag 转 YAML 工具
└── localization_exporter.py       # ROS2 实时定位数据导出工具
```

---

## 使用场景

### 场景 1：相机标定数据处理
```bash
# 1. 转换相机标定文件格式
python3 convert_to_tencent_format.py camera_calibration.yaml output.json

# 2. 生成相机外参变换矩阵
python3 generate_extrinsics.py camera0 camera0_extrinsics.json
```

### 场景 2：从 ROS2 Bag 提取定位数据
```bash
# 从 db3 bag 文件提取定位数据
python3 db3_to_yaml.py /path/to/rosbag.db3
# 输出到: /path/to/rosbag.db3 所在目录/localization/
```

### 场景 3：实时记录定位数据
```bash
# 启动实时导出节点
python3 localization_exporter.py /tmp/localization_data

# 在另一个终端启用保存
ros2 service call /localization_exporter/set_enable_save std_srvs/srv/SetBool "{data: true}"
```

---

## 注意事项

### 通用注意事项
- 所有脚本遵循 PEP8 规范
- 建议为不同任务建立独立的虚拟环境，避免依赖冲突
- 处理文件前建议先备份

### 脚本特定注意事项

**`convert_to_tencent_format.py`**：
- 输入 YAML 文件需要包含 `camera_matrix` 和 `distortion_coefficients` 字段

**`generate_extrinsics.py`**：
- 需要修改脚本中的 YAML 文件路径（硬编码在第85-86行）
- 输出 JSON 文件需要预先存在

**`db3_to_yaml.py`**：
- 需要确保 bag 文件中包含必需的话题
- 输出频率固定为 10 Hz

**`localization_exporter.py`**：
- 需要在有 ROS2 话题发布的环境下运行
- 默认不保存数据，需要通过服务调用启用

---

## 版本说明

这些是旧版工具，部分功能可能已被新工具替代：

- `localization_exporter.py` → 新工具：`tools/export_localization_to_csv.py.sh`（支持 MCAP 格式，每个文件独立 CSV）
- `db3_to_yaml.py` → 新工具：`tools/export_localization_to_csv.py.sh`（支持 MCAP 格式）

建议优先使用新工具，这些旧工具保留用于兼容性。

---

## 更新日志

### 2025-02-02
- 更新 README，匹配实际存在的脚本
- 添加详细的使用说明和参数说明
- 添加依赖要求和注意事项
- 添加版本说明，标注新旧工具对应关系

---

## 联系与支持

如有问题或建议，请查看各脚本的代码注释或联系维护人员。
