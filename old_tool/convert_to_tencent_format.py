#!/usr/bin/env python3
import yaml
import json
import sys

# 读取YAML文件
def read_input_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"文件未找到: {file_path}")
        sys.exit(2)

# 保存输出JSON文件
def save_output_file(file_path, data):
    with open(file_path, 'w') as file:
        json.dump(data, file, indent=4)

# 处理数据并生成输出数据
def process_data(input_data):
    # 从输入数据中提取相机矩阵和畸变系数
    camera_matrix_data = input_data["camera_matrix"]["data"]
    distortion_coeffs_data = input_data["distortion_coefficients"]["data"]
    
    # 生成输出数据
    output_data = {
        "extrinsic": input_data.get("extrinsic", None),  # 保留外参，若没有则为空
        "intrinsic": camera_matrix_data,  # 使用相机矩阵数据
        "distortion": distortion_coeffs_data,  # 使用畸变系数数据
        "translation": input_data.get("translation", None)  # 保留平移参数，若没有则为空
    }
    return output_data

# 主函数
def main(input_file_path, output_file_path):
    # 读取输入YAML文件
    input_data = read_input_file(input_file_path)
    
    # 处理数据并获取替换后的数据
    output_data = process_data(input_data)
    
    # 保存处理后的输出JSON文件
    save_output_file(output_file_path, output_data)
    print(f"处理后的数据已保存到: {output_file_path}")

# 示例调用
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("请提供正确的输入和输出文件路径作为参数.")
        sys.exit(1)
    
    input_file_path = sys.argv[1]  # 从命令行参数获取输入文件路径
    output_file_path = sys.argv[2]  # 从命令行参数获取输出文件路径
    
    main(input_file_path, output_file_path)
