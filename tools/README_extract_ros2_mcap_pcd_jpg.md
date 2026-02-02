# ROS2 MCAP 导出 JPG / PCD

本工具用于把 **ROS2 rosbag2（storage=mcap, serialization=cdr）** 的数据导出为：

- **相机**：`sensor_msgs/msg/CompressedImage` → 按 topic 分目录导出图片（通常是 `.jpg`）
- **点云**：`sensor_msgs/msg/PointCloud2` → 按 topic 分目录导出 `.pcd`（PCD v0.7, binary）

脚本位置：`tools/extract_ros2_mcap_pcd_jpg.py`

## 功能特性

- ✅ 支持处理单个 `.mcap` 文件
- ✅ 支持处理 rosbag2 目录（包含 `metadata.yaml` 和多个 `.mcap` 文件）
- ✅ **自动批量处理**：自动查找并处理所有 `perception_data_*` 目录
- ✅ 文件名格式：`{13位毫秒时间戳}_{5位序号}`（例如：`1769480236101_00001.jpg`）
- ✅ 支持按 topic 正则表达式过滤
- ✅ 支持抽帧和限量导出

## 用法

### 1) 自动批量处理所有 perception_data_* 目录（推荐）

不传参数时，脚本会自动查找当前目录下所有 `perception_data_*` 目录并逐个处理：

```bash
# 处理当前目录下所有 perception_data_* 目录
python3 tools/extract_ros2_mcap_pcd_jpg.py

# 在指定目录搜索并处理所有 perception_data_* 目录
python3 tools/extract_ros2_mcap_pcd_jpg.py --search-dir /path/to/data
```

每个目录的输出会保存到各自的 `<perception_data_dir>/export_pcd_jpg/` 目录。

### 2) 处理单个 rosbag2 目录

```bash
python3 tools/extract_ros2_mcap_pcd_jpg.py \
  /path/to/perception_data_20260127100516
```

默认输出到：`<bag_dir>/export_pcd_jpg/`

### 3) 处理单个 .mcap 文件

```bash
python3 tools/extract_ros2_mcap_pcd_jpg.py \
  /path/to/file.mcap
```

默认输出到：`<mcap_file_dir>/<mcap_filename>_export_pcd_jpg/`

注意：如果该 `.mcap` 文件不在标准 rosbag2 目录（没有 `metadata.yaml`），脚本会自动从 MCAP summary 生成临时 `metadata.yaml` 来兼容 `rosbags` 读取。

### 4) 只导出特定 topic，并抽帧/限量（推荐先验证）

```bash
python3 tools/extract_ros2_mcap_pcd_jpg.py \
  /path/to/perception_data_20260127100516 \
  --out /path/to/output \
  --image-topic-regex "electronic_rearview_mirror" \
  --pcd-topic-regex "sensing/lidar" \
  --every-n 200 \
  --max-total 30
```

## 命令行参数

- `bag_path`（可选）：rosbag2 目录或单个 `.mcap` 文件路径。如果不提供，会自动搜索并处理所有 `perception_data_*` 目录
- `--out <dir>`：指定输出目录（默认：`<bag_path>/export_pcd_jpg` 或 `<mcap_file_dir>/<filename>_export_pcd_jpg`）
- `--search-dir <dir>`：指定搜索 `perception_data_*` 目录的根目录（默认：当前目录）
- `--image-topic-regex <regex>`：只导出匹配正则表达式的 CompressedImage topics
- `--pcd-topic-regex <regex>`：只导出匹配正则表达式的 PointCloud2 topics
- `--every-n <n>`：每 N 条消息导出一次（抽帧，默认：1）
- `--max-total <n>`：最多导出 N 个文件（所有 topics 合计）
- `--max-per-topic <n>`：每个 topic 最多导出 N 个文件
- `--overwrite`：覆盖已存在的文件（默认：跳过）
- `--progress-seconds <sec>`：每 N 秒打印一次进度（默认：5.0，设为 0 禁用）

## 输出目录结构

```text
export_pcd_jpg/
  topics.json                 # topic -> msgtype 映射
  export_stats.json           # 导出数量统计
  images/
    <topic_as_dirname>/
      <stamp_ms>_<idx>.jpg    # 例如：1769480236101_00001.jpg
  pcd/
    <topic_as_dirname>/
      <stamp_ms>_<idx>.pcd    # 例如：1769480236101_00001.pcd
```

其中：
- `<topic_as_dirname>` 会把 `/a/b/c` 转换成 `a__b__c`，方便落盘到文件系统
- `<stamp_ms>` 是 13 位毫秒时间戳（从纳秒除以 1_000_000 得到）
- `<idx>` 是 5 位序号（从 00001 开始显示为 00001/00002/...；脚本内部从 1 开始计数）

另外：对 **图片 topics**（`sensor_msgs/msg/CompressedImage`），脚本会尽量生成更短的目录名：
- 例如 `/electronic_rearview_mirror/front_3mm/camera_image_jpeg` → `front_3mm_jpeg`
- 如果短名发生冲突（不同 topic 生成了相同目录名），会自动回退到完整的 `a__b__c` 形式，避免覆盖

## 使用示例

### 批量处理示例

```bash
# 处理当前目录下所有 perception_data_* 目录，每个目录抽帧（每200帧导出1帧）
python3 tools/extract_ros2_mcap_pcd_jpg.py --every-n 200

# 在指定目录搜索，只导出电子后视镜相机，每个topic最多100张
python3 tools/extract_ros2_mcap_pcd_jpg.py \
  --search-dir /media/ipc/AQLoopCloseData/failcase_data \
  --image-topic-regex "electronic_rearview_mirror" \
  --max-per-topic 100
```

### 单个文件/目录处理示例

```bash
# 处理单个目录，全量导出
python3 tools/extract_ros2_mcap_pcd_jpg.py \
  /media/ipc/AQLoopCloseData/failcase_data/perception_data_20260127100516

# 处理单个mcap文件
python3 tools/extract_ros2_mcap_pcd_jpg.py \
  /path/to/perception_data_20260127100516_0.mcap
```

## 备注

- `CompressedImage` 本身就是压缩后的图像字节，脚本会直接落盘（不做解码重编码），因此非常快且无损。
- `.pcd` 采用 **binary**，如果你的 `PointCloud2` 存在 padding 或非连续 layout，脚本会自动重排字段后写入。
- 数据量可能非常大（尤其图片+点云），建议先用 `--every-n` / `--max-total` 做小样验证，再跑全量。
- 批量处理时，如果某个目录处理失败，脚本会继续处理其他目录，最后显示成功/失败统计。
- 文件名格式已更新为毫秒时间戳（13位）+ 序号（5位），例如：`1769480236101_00001.jpg`

## 快速参考命令

```bash
# 1. 自动处理当前目录下所有perception_data_*目录
python3 tools/extract_ros2_mcap_pcd_jpg.py

# 2. 在指定目录搜索并处理所有perception_data_*目录
python3 tools/extract_ros2_mcap_pcd_jpg.py --search-dir /path/to/data

# 3. 处理单个mcap文件
python3 tools/extract_ros2_mcap_pcd_jpg.py /path/to/file.mcap

# 4. 处理单个目录
python3 tools/extract_ros2_mcap_pcd_jpg.py /path/to/perception_data_20260127100516
```

**注意**：脚本位于 `tools/` 目录下。如果从项目根目录运行，使用 `tools/extract_ros2_mcap_pcd_jpg.py`；如果从其他目录运行，请使用绝对路径 `/media/ipc/AQLoopCloseData/tools/extract_ros2_mcap_pcd_jpg.py`。

