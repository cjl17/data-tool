#!/usr/bin/env python3
import os
import sys
import json
import yaml
import numpy as np
from collections import deque


# ------------------ 相机名称映射 & 全局常量 ------------------
camera_name_map = {
    "front": "camera1",
    "rear": "camera2",
    "front_left": "camera3",
    "front_right": "camera4",
    "rear_left": "camera5",
    "rear_right": "camera6",
}

# 实际 mcap 中的 camera_info 话题映射（基于 20260207094324_0.mcap 检查结果）
camera_info_topic_map = {
    "front": "/sensing/camera/front_3mm/camera_info",
    "rear": "/sensing/camera/rear_3mm/camera_info",
    "front_left": "/sensing/camera/front_left/camera_info",
    "front_right": "/sensing/camera/front_right/camera_info",
    "rear_left": "/sensing/camera/rear_left/camera_info",
    "rear_right": "/sensing/camera/rear_right/camera_info",
}

# 根据你现有标定，默认的坐标系/TF 话题命名假设
BASE_FRAME = "base_link"
TF_TOPICS = ("/tf", "/tf_static")


# ------------------ 工具函数 ------------------
def load_yaml(file_path: str):
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def rpy_to_rot_matrix(roll, pitch, yaw):
    """将 roll, pitch, yaw 转换为旋转矩阵"""
    Rx = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ]
    )

    Ry = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )

    Rz = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ]
    )
    return Rz @ Ry @ Rx


def pose_to_matrix(pose):
    """将 x,y,z,roll,pitch,yaw 转为 4x4 矩阵"""
    t = np.array([pose.get("x", 0), pose.get("y", 0), pose.get("z", 0)])
    roll = pose.get("roll", 0.0)
    pitch = pose.get("pitch", 0.0)
    yaw = pose.get("yaw", 0.0)
    R = rpy_to_rot_matrix(roll, pitch, yaw)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    计算齐次变换矩阵的逆矩阵（4x4）
    T =
    [ R | t ]
    [ 0 | 1 ]
    返回：
    T_inv =
    [ R.T | -R.T @ t ]
    [  0  |    1     ]
    """
    assert T.shape == (4, 4), f"输入矩阵维度错误，期望(4,4)，实际{T.shape}"

    R, t = T[:3, :3], T[:3, 3]
    T_inv = np.eye(4, dtype=T.dtype)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def quat_to_rot_matrix(x, y, z, w):
    """四元数转旋转矩阵，假设已归一化"""
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    R = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ]
    )
    return R


def transform_msg_to_matrix(t):
    """
    将 geometry_msgs/TransformStamped.transform 转为 4x4 矩阵
    """
    trans = t.translation
    rot = t.rotation

    T = np.eye(4)
    T[:3, 3] = np.array([trans.x, trans.y, trans.z])
    T[:3, :3] = quat_to_rot_matrix(rot.x, rot.y, rot.z, rot.w)
    return T


def build_tf_graph_from_mcap(mcap_path: str, tf_topics=TF_TOPICS):
    """
    从 mcap 中读取 TF，构建图：
      graph[parent][child] = T_parent_child
      同时也存 child->parent 的逆变换，方便搜索任意两帧之间的变换。
    """
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import Decoder
    except ImportError as e:
        raise ImportError(
            f"导入 mcap/mcap_ros2_support 失败: {e}，"
            f"请先安装: pip install mcap mcap-ros2-support"
        )

    graph = {}

    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[Decoder])
        for schema, channel, message in reader.iter_messages(topics=list(tf_topics)):
            tf_msg = message.data  # 期望是 tf2_msgs/msg/TFMessage
            # 有些 mcap 中 data 可能还是 bytes，这里做个健壮性判断
            if isinstance(tf_msg, (bytes, bytearray)):
                # 暂时跳过无法解码的 TF 消息
                continue
            try:
                transforms = getattr(tf_msg, "transforms", None)
            except Exception:
                transforms = None
            if not transforms:
                continue
            for t in transforms:
                parent = t.header.frame_id
                child = t.child_frame_id
                T_parent_child = transform_msg_to_matrix(t.transform)
                # parent -> child
                graph.setdefault(parent, {})[child] = T_parent_child
                # child -> parent（存一份逆变换，方便搜索）
                graph.setdefault(child, {})[parent] = invert_transform(T_parent_child)

    return graph


def find_transform(graph, from_frame: str, to_frame: str) -> np.ndarray:
    """
    在 TF 图中搜索 from_frame -> to_frame 的变换矩阵
    使用 BFS 累乘。
    """
    if from_frame == to_frame:
        return np.eye(4)

    visited = set()
    q = deque()
    q.append((from_frame, np.eye(4)))
    visited.add(from_frame)

    while q:
        cur_frame, T_from_cur = q.popleft()
        for next_frame, T_cur_next in graph.get(cur_frame, {}).items():
            if next_frame in visited:
                continue
            T_from_next = T_from_cur @ T_cur_next
            if next_frame == to_frame:
                return T_from_next
            visited.add(next_frame)
            q.append((next_frame, T_from_next))

    raise RuntimeError(f"在 TF 图中找不到 {from_frame} -> {to_frame} 的变换链")


def read_extrinsic_from_mcap(mcap_path: str, base_frame: str, camera_frame: str) -> np.ndarray:
    """
    从 mcap 中的 TF 计算 base_frame -> camera_frame 的变换，
    再转成 camera_frame -> base_frame（与之前 YAML 版本保持一致）。
    """
    graph = build_tf_graph_from_mcap(mcap_path, TF_TOPICS)
    T_base_to_camera = find_transform(graph, base_frame, camera_frame)
    # 之前脚本最终导出的是 camera_link -> base_link 的矩阵
    T_camera_to_base = invert_transform(T_base_to_camera)
    return T_camera_to_base


def read_intrinsic_from_mcap(mcap_path: str, topic: str):
    """
    从 mcap 文件中读取第一个 CameraInfo 消息，
    返回 (K_list, D_list)，分别是 9 个内参元素和畸变参数列表。
    """
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import Decoder
    except ImportError as e:
        raise ImportError(
            f"导入 mcap/mcap_ros2_support 失败: {e}，"
            f"请先安装: pip install mcap mcap-ros2-support"
        )

    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[Decoder])
        for schema, channel, message in reader.iter_messages(topics=[topic]):
            cam_info = message.data  # 期望是 sensor_msgs/msg/CameraInfo
            if isinstance(cam_info, (bytes, bytearray)):
                # 解码失败的先跳过，后面统一回退到 YAML
                continue
            try:
                K = getattr(cam_info, "k", None)
                D = getattr(cam_info, "d", None)
            except Exception:
                K, D = None, None
            if K is None or D is None:
                continue
            K_list = [float(x) for x in K]
            D_list = [float(x) for x in D]
            return K_list, D_list

    raise RuntimeError(f"在 mcap={mcap_path} 里没有找到 topic={topic} 的 CameraInfo 消息")


def process_camera(base_path: str, camera_name: str, mcap_path: str):
    """
    处理单个相机：从 mcap 中读取 TF + camera_info 生成 JSON，
    失败时回退到 YAML。
    """
    if camera_name not in camera_name_map:
        raise ValueError(f"未知的 camera_name: {camera_name}, 只能是 {list(camera_name_map.keys())}")

    camera_key = camera_name_map[camera_name]
    camera_frame = f"{camera_key}/camera_link"  # 例如 camera1/camera_link（目前 TF 未能成功解析，多数会回退）
    camera_info_topic = camera_info_topic_map.get(
        camera_name, f"/{camera_key}/camera_info"
    )

    # ------------------ 1. 外参：优先从 mcap TF，失败回退到 YAML ------------------
    use_mcap_extrinsic = True
    try:
        T_camera_to_base = read_extrinsic_from_mcap(mcap_path, BASE_FRAME, camera_frame)
        print(
            f"[{camera_name}] 从 mcap 读取外参成功: {camera_frame} -> {BASE_FRAME}，"
            f"TF topics={TF_TOPICS}"
        )
    except Exception as e:
        print(f"[{camera_name}] 从 mcap 读取外参失败，回退到 YAML 外参: {e}")
        use_mcap_extrinsic = False

    if not use_mcap_extrinsic:
        # 使用原有 YAML 方式：base_link -> sensor_kit_base_link -> cameraX/camera_link，再取逆
        sensor_kit_calib_path = os.path.join(
            base_path,
            "parameter/sensor_kit/robobus_sensor_kit_description/extrinsic_parameters/sensor_kit_calibration.yaml",
        )
        sensors_calib_path = os.path.join(
            base_path,
            "parameter/sensor_kit/robobus_sensor_kit_description/extrinsic_parameters/sensors_calibration.yaml",
        )

        try:
            sensor_kit_data = load_yaml(sensor_kit_calib_path)
            sensors_data = load_yaml(sensors_calib_path)

            # base_link -> sensor_kit_base_link
            base_link2sensor_pose = sensors_data.get("base_link", {}).get("sensor_kit_base_link", {})
            # sensor_kit_base_link -> cameraX/camera_link
            sensor2camera_pose = sensor_kit_data.get("sensor_kit_base_link", {}).get(
                f"{camera_key}/camera_link", {}
            )

            T_base2sensor = pose_to_matrix(base_link2sensor_pose)
            T_sensor2camera = pose_to_matrix(sensor2camera_pose)
            T_base2camera = T_base2sensor @ T_sensor2camera  # base_link -> camera_link
            T_camera_to_base = invert_transform(T_base2camera)  # camera_link -> base_link
        except FileNotFoundError as e:
            # 如果没有标定 YAML，就退化为单位矩阵外参，至少保证脚本能跑完并生成内参
            print(
                f"[{camera_name}] 找不到标定 YAML({e.filename})，"
                f"外参将使用单位矩阵（camera_frame 与 base_link 重合）"
            )
            T_camera_to_base = np.eye(4)

    # 提取平移向量
    translation = T_camera_to_base[:3, 3].tolist()

    # ------------------ 2. 内参：优先从 mcap camera_info，失败回退到 YAML ------------------
    use_mcap_intrinsic = True
    try:
        camera_matrix_data, distortion_coeffs_data = read_intrinsic_from_mcap(
            mcap_path, camera_info_topic
        )
        print(f"[{camera_name}] 从 mcap 读取内参成功: topic={camera_info_topic}")
    except Exception as e:
        print(f"[{camera_name}] 从 mcap 读取 camera_info 失败，回退到 YAML 内参: {e}")
        use_mcap_intrinsic = False

    if not use_mcap_intrinsic:
        camera_yaml_path = os.path.join(
            base_path,
            f"parameter/sensor_kit/robobus_sensor_kit_description/intrinsic_parameters/{camera_key}_params.yaml",
        )
        try:
            camera_data = load_yaml(camera_yaml_path)
            camera_matrix_data = camera_data["camera_matrix"]["data"]
            distortion_coeffs_data = camera_data["distortion_coefficients"]["data"]
        except FileNotFoundError as e:
            # 没有 YAML，又没有 camera_info，就使用单位内参 + 0 畸变，保证脚本可运行
            print(
                f"[{camera_name}] 找不到内参 YAML({e.filename})，"
                f"且 mcap 中无 {camera_info_topic}，内参将使用单位矩阵、畸变为 0。"
            )
            camera_matrix_data = [1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0]
            distortion_coeffs_data = [0.0, 0.0, 0.0, 0.0, 0.0]

    # ------------------ 3. 输出 JSON ------------------
    output_json = {
        "extrinsic": T_camera_to_base.flatten().tolist(),
        "intrinsic": camera_matrix_data,
        "distortion": distortion_coeffs_data,
        "translation": translation,
    }

    parameter_path = os.path.join(base_path, "sensor_parameter")
    if not os.path.exists(parameter_path):
        os.makedirs(parameter_path)

    output_file = os.path.join(
        parameter_path,
        f"{camera_key}_{camera_name}.json",
    )
    with open(output_file, "w") as f:
        json.dump(output_json, f, indent=4, ensure_ascii=False)

    print(f"[{camera_name}] 生成完成: {output_file}")


if __name__ == "__main__":
    # 改成“离线”：从 mcap 中读 camera_info + TF
    # 一键生成用法：
    #   单个相机: python neieaican.py <base_path> <camera_name> <mcap_path>
    #   全部相机: python neieaican.py <base_path> all          <mcap_path>
    #
    # base_path 仍然用于 YAML 回退（读不到 mcap 时）
    if len(sys.argv) != 4:
        print(
            "Usage:\n"
            "  单个相机: python neieaican.py <base_path> <camera_name> <mcap_path>\n"
            "  全部相机: python neieaican.py <base_path> all          <mcap_path>\n"
            f"  camera_name 只能是: {list(camera_name_map.keys())}"
        )
        sys.exit(1)

    base_path_arg = sys.argv[1]
    camera_arg = sys.argv[2]
    mcap_path_arg = sys.argv[3]

    if camera_arg == "all":
        for name in camera_name_map.keys():
            print(f"\n====== 处理相机: {name} ======")
            process_camera(base_path_arg, name, mcap_path_arg)
    else:
        process_camera(base_path_arg, camera_arg, mcap_path_arg)
