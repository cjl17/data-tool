#!/usr/bin/env python3

import numpy as np
import yaml
import argparse
import json

# 从YAML文件加载数据
def load_yaml_data(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

# 欧拉角转换为旋转矩阵
def euler_to_rotation_matrix(roll, pitch, yaw):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    
    return np.dot(Rz, np.dot(Ry, Rx))

# 生成变换矩阵
def transform_to_matrix(translation, rotation):
    R = euler_to_rotation_matrix(rotation[0], rotation[1], rotation[2])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation
    return T

# 矩阵求逆（齐次矩阵）
def invert_transform(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv

# 计算从 base_link 到 cameraX/base_link 的变换矩阵
def calculate_transformation(base_to_sensor_translation, base_to_sensor_rotation,
                             sensor_to_camera_translation, sensor_to_camera_rotation):
    T_base_to_sensor = transform_to_matrix(base_to_sensor_translation, base_to_sensor_rotation)
    T_sensor_to_camera = transform_to_matrix(sensor_to_camera_translation, sensor_to_camera_rotation)
    T_base_to_camera = np.dot(T_base_to_sensor, T_sensor_to_camera)
    return T_base_to_camera

# 输出变换矩阵保留三位小数
def print_transformation_matrix(matrix):
    for row in matrix:
        print([f"{val:.3f}" for val in row])

# 保存变换矩阵到 JSON 文件中
def save_to_json(output_file, transformation_matrix):
    with open(output_file, 'r') as file:
        data = json.load(file)
    
    extrinsic = transformation_matrix.flatten().tolist()
    data['extrinsic'] = extrinsic
    data['translation'] = transformation_matrix[:3, 3].tolist()
    
    with open(output_file, 'w') as file:
        json.dump(data, file, indent=4)
    
    print(f"Transformation matrix saved to {output_file}")

# 解析命令行参数
def parse_args():
    parser = argparse.ArgumentParser(description="计算相机变换矩阵并保存到 JSON 文件")
    parser.add_argument('camera', type=str, help="指定要计算变换矩阵的相机，如 camera0、camera1、camera2 等")
    parser.add_argument('output_file', type=str, help="指定输出的 JSON 文件路径")
    return parser.parse_args()

def main():
    args = parse_args()
    camera = args.camera
    output_file = args.output_file

    base_link_file = '/home/nvidia/pix/ros2log/bev_20250812_151102_data/parameter/sensor_kit/robobus_sensor_kit_description/extrinsic_parameters/sensors_calibration.yaml'
    camera_camera_link_file = '/home/nvidia/pix/ros2log/bev_20250812_151102_data/parameter/sensor_kit/robobus_sensor_kit_description/extrinsic_parameters/sensor_kit_calibration.yaml'

    base_link_data = load_yaml_data(base_link_file)
    camera_camera_link_data = load_yaml_data(camera_camera_link_file)

    base_to_sensor_translation = np.array([base_link_data['base_link']['sensor_kit_base_link']['x'],
                                           base_link_data['base_link']['sensor_kit_base_link']['y'],
                                           base_link_data['base_link']['sensor_kit_base_link']['z']])
    base_to_sensor_rotation = np.array([base_link_data['base_link']['sensor_kit_base_link']['roll'],
                                        base_link_data['base_link']['sensor_kit_base_link']['pitch'],
                                        base_link_data['base_link']['sensor_kit_base_link']['yaw']])

    camera_link = f'{camera}/camera_link'
    if camera_link in camera_camera_link_data['sensor_kit_base_link']:
        camera_translation = np.array([camera_camera_link_data['sensor_kit_base_link'][camera_link]['x'],
                                       camera_camera_link_data['sensor_kit_base_link'][camera_link]['y'],
                                       camera_camera_link_data['sensor_kit_base_link'][camera_link]['z']])
        camera_rotation = np.array([camera_camera_link_data['sensor_kit_base_link'][camera_link]['roll'],
                                    camera_camera_link_data['sensor_kit_base_link'][camera_link]['pitch'],
                                    camera_camera_link_data['sensor_kit_base_link'][camera_link]['yaw']])

        # base_link → camera_link
        T_base_to_camera = calculate_transformation(base_to_sensor_translation, base_to_sensor_rotation,
                                                    camera_translation, camera_rotation)

        # 取逆：camera_link → base_link
        T_camera_to_base = invert_transform(T_base_to_camera)

        print(f"{camera} -> Camera to Base Transformation Matrix:")
        print_transformation_matrix(T_camera_to_base)

        # 保存反的矩阵
        save_to_json(output_file, T_camera_to_base)
    else:
        print(f"Error: {camera_link} not found in YAML data.")

if __name__ == '__main__':
    main()
