#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云和相机数据预处理脚本
用于匹配激光雷达点云文件和相机图像文件的时间戳

作者: AI进化论-花生
版本: 1.0
"""

import os
import glob
import yaml
import argparse
import shutil
import logging
from typing import List, Tuple, Optional
from pathlib import Path
from collections import OrderedDict


class DataPreprocessor:
    """数据预处理器类"""
    
    def __init__(self, base_path: str, time_threshold: float = 0.016, dt :float = 0.0):
        """
        初始化数据预处理器
        
        Args:
            base_path: 数据根目录路径
            time_threshold: 时间戳匹配阈值（秒），默认0.016秒
        """
        self.base_path = Path(base_path)
        self.time_threshold = time_threshold
        self.camera_folders = [
            "camera/front",
            "camera/front_left", 
            "camera/front_right",
            "camera/rear",
            "camera/rear_left",
            "camera/rear_right"
        ]
        dt1=int(abs(dt*1000))
        if dt<0:
            self.lidar_folder = f"lidar-{dt1}ms"
        elif dt>0:
            self.lidar_folder = f"lidar+{dt1}ms"
        else:
            self.lidar_folder = "lidar"
        
        # 设置日志
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
    
    def extract_timestamp(self, filename: str) -> float:
        """
        从文件名中提取时间戳
        
        Args:
            filename: 文件名
            
        Returns:
            时间戳（浮点数，秒）
        """
        basename = os.path.basename(filename)
        # 移除文件扩展名
        name_without_ext = os.path.splitext(basename)[0]
        # 提取时间戳部分（格式为 timestamp_index，时间戳是毫秒，需要转换为秒）
        parts = name_without_ext.split('_')
        try:
            # 第一部分是时间戳（毫秒），转换为秒
            timestamp_ms = float(parts[0])
            return timestamp_ms / 1000.0
        except (ValueError, IndexError):
            self.logger.warning(f"无法解析时间戳: {filename}")
            return 0.0
    
    def load_files_from_subdir(self, subdir_path: Path) -> Tuple[List[str], List[List[str]]]:
        """
        从单个子目录加载并排序所有文件
        
        Args:
            subdir_path: 子目录路径（如 perception_data_20260129113410_0）
        
        Returns:
            (lidar_files, camera_files_list): 激光雷达文件列表和相机文件列表的列表
        """
        # 新的路径映射：从新的目录结构映射到旧的相机文件夹名称
        camera_path_mapping = {
            "camera/front": "front_3mm_jpeg",
            "camera/front_left": "front_left_jpeg",
            "camera/front_right": "front_right_jpeg",
            "camera/rear": "rear_3mm_jpeg",
            "camera/rear_left": "rear_left_jpeg",
            "camera/rear_right": "rear_right_jpeg",
        }
        
        # 加载激光雷达文件
        lidar_path = subdir_path / "raw_data" / "pcd" / "sensing__lidar__concatenated__pointcloud"
        lidar_files = []
        if lidar_path.exists():
            lidar_files = glob.glob(str(lidar_path / "*.pcd"))
            lidar_files = sorted(lidar_files, key=self.extract_timestamp)
        
        # 加载相机文件
        camera_files_list = []
        for camera_folder in self.camera_folders:
            camera_files = []
            image_folder_name = camera_path_mapping[camera_folder]
            camera_path = subdir_path / "raw_data" / "images" / image_folder_name
            if camera_path.exists():
                camera_files = glob.glob(str(camera_path / "*.jpg"))
                camera_files = sorted(camera_files, key=self.extract_timestamp)
            camera_files_list.append(camera_files)
        
        return lidar_files, camera_files_list
    
    def load_files(self) -> Tuple[List[str], List[List[str]]]:
        """
        加载并排序所有文件（从新的packed_data结构，所有子目录合并）
        
        Returns:
            (lidar_files, camera_files_list): 激光雷达文件列表和相机文件列表的列表
        """
        self.logger.info(f"开始加载文件，基础路径: {self.base_path}")
        
        # 查找所有 perception_data_* 子目录
        pattern = str(self.base_path / "perception_data_*")
        subdirs = sorted(glob.glob(pattern))
        if not subdirs:
            raise FileNotFoundError(f"未找到 perception_data_* 子目录: {self.base_path}")
        
        self.logger.info(f"找到 {len(subdirs)} 个子目录")
        
        # 从所有子目录加载文件并合并
        all_lidar_files = []
        all_camera_files_list = [[] for _ in self.camera_folders]
        
        for subdir in subdirs:
            lidar_files, camera_files_list = self.load_files_from_subdir(Path(subdir))
            all_lidar_files.extend(lidar_files)
            for i, camera_files in enumerate(camera_files_list):
                all_camera_files_list[i].extend(camera_files)
        
        # 排序所有文件
        all_lidar_files = sorted(all_lidar_files, key=self.extract_timestamp)
        for i in range(len(all_camera_files_list)):
            all_camera_files_list[i] = sorted(all_camera_files_list[i], key=self.extract_timestamp)
        
        self.logger.info(f"找到 {len(all_lidar_files)} 个激光雷达文件")
        for i, camera_folder in enumerate(self.camera_folders):
            self.logger.info(f"找到 {len(all_camera_files_list[i])} 个 {camera_folder} 文件")
        
        return all_lidar_files, all_camera_files_list
    
    def match_files(self, lidar_files: List[str], camera_files_list: List[List[str]]) -> List[List[str]]:
        """
        匹配激光雷达文件和相机文件
        
        Args:
            lidar_files: 激光雷达文件列表
            camera_files_list: 相机文件列表的列表
            
        Returns:
            匹配结果列表，每个元素包含一个激光雷达文件和对应的相机文件
        """
        self.logger.info("开始匹配文件...")

        # 为每个相机文件夹维护一个当前下标，确保单调前进
        camera_indices = [0 for _ in camera_files_list]
        matched_groups: List[List[str]] = []

        for lidar_file in lidar_files:
            lidar_timestamp = self.extract_timestamp(lidar_file)
            current_group: List[Optional[str]] = [lidar_file]

            for cam_idx, camera_files in enumerate(camera_files_list):
                idx = camera_indices[cam_idx]

                # 没有该相机文件
                if not camera_files:
                    current_group.append(None)
                    continue

                # 跳过明显早于阈值窗口左侧的相机帧
                while idx < len(camera_files) and \
                        self.extract_timestamp(camera_files[idx]) < (lidar_timestamp - self.time_threshold):
                    idx += 1

                # 检查当前帧是否在阈值内
                if idx < len(camera_files):
                    cam_ts = self.extract_timestamp(camera_files[idx])
                    if abs(cam_ts - lidar_timestamp) <= self.time_threshold:
                        current_group.append(camera_files[idx])
                        idx += 1  # 消耗该相机帧
                    else:
                        current_group.append(None)
                else:
                    current_group.append(None)

                camera_indices[cam_idx] = idx

            # 仅保存所有相机都匹配成功的组
            if all(file_path is not None for file_path in current_group[1:]):
                matched_groups.append(current_group)  # [lidar, cam1, cam2, ...]

        self.logger.info(f"匹配完成，共找到 {len(matched_groups)} 组匹配文件")
        return matched_groups
    
    def save_results(self, matched_results: List[List[str]], output_file: str = "lidar_cameras_files.yaml"):
        """
        保存匹配结果到YAML文件
        
        Args:
            matched_results: 匹配结果列表
            output_file: 输出文件路径（完整路径）
        """
        output_path = Path(output_file)
        
        # 准备YAML数据
        yaml_data = {
            'description': '激光雷达和相机文件匹配结果',
            'time_threshold': self.time_threshold,
            'total_matches': len(matched_results),
            'camera_folders': self.camera_folders,
            'matches': []
        }
        
        for i, match_group in enumerate(matched_results):
            match_data = {
                'index': i,
                'lidar_file': os.path.basename(match_group[0]) if match_group[0] else None,
                'lidar_timestamp': self.extract_timestamp(match_group[0]) if match_group[0] else None,
                'cameras': {}
            }
            
            for j, camera_file in enumerate(match_group[1:]):
                folder_name = self.camera_folders[j].split('/')[-1]  # 获取文件夹名
                match_data['cameras'][folder_name] = {
                    'file': os.path.basename(camera_file) if camera_file else None,
                    'timestamp': self.extract_timestamp(camera_file) if camera_file else None
                }
            
            yaml_data['matches'].append(match_data)
        
        # 保存到YAML文件
        try:
            with open(output_path, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True, indent=2)
            self.logger.info(f"匹配结果已保存到: {output_path}")
        except Exception as e:
            self.logger.error(f"保存文件失败: {e}")
            raise
    
    def save_results_simple_list(self, matched_results: List[List[str]], output_file: str = "lidar_cameras_simple_list.yaml"):
        """
        以简单列表形式保存匹配结果，每组为6个相机路径加1个激光路径，共7行。
        顺序：front, rear, front_left, front_right, rear_left, rear_right, lidar
        
        Args:
            matched_results: 匹配结果列表
            output_file: 输出文件路径（完整路径）
        """
        output_path = Path(output_file)
        try:
            lines: List[str] = []
            for group in matched_results:
                lidar_path = group[0]
                cams = group[1:]
                # 定义目标顺序映射到 self.camera_folders 的索引
                order = [
                    self.camera_folders.index("camera/front"),
                    self.camera_folders.index("camera/rear"),
                    self.camera_folders.index("camera/front_left"),
                    self.camera_folders.index("camera/front_right"),
                    self.camera_folders.index("camera/rear_left"),
                    self.camera_folders.index("camera/rear_right"),
                ]
                for idx in order:
                    lines.append(str(cams[idx]))
                lines.append(str(lidar_path))
            with open(output_path, 'w', encoding='utf-8') as f:
                for line in lines:
                    f.write(f"{line}\n")
            self.logger.info(f"简单列表结果已保存到: {output_path}")
        except Exception as e:
            self.logger.error(f"保存简单列表失败: {e}")
            raise
    def is_near_boundary(self, filename: str, tol=0.01):
        """
        判断文件名对应的时间戳的小数部分是否接近 0 或 0.5，误差容忍度为 tol。
        文件名格式为 timestamp_index，时间戳是毫秒，需要转换为秒后判断。
        例如 '1769658159399_00001' -> 时间戳 1769658159.399 -> 小数部分 0.399，不满足。
        """
        try:
            # 从文件名中提取时间戳（秒）
            timestamp_sec = self.extract_timestamp(filename)
            frac = timestamp_sec % 1  # 取小数部分

            return abs(frac - 0.000) <= tol or abs(frac - 1.0) < tol or abs(frac - 0.500) <= tol
        except (ValueError, AttributeError):
            return False

    def export_sequences(self, matched_results: List[List[str]], export_base_path: Path, sequence_size: int = 200, use_symlink: bool = True):
        """
        根据匹配结果导出为 sequence* 目录结构。
        目录结构：sequence00000/{front,rear,front_left,front_right,rear_left,rear_right,lidar}
        每个序列包含 sequence_size 组样本。
        
        Args:
            export_base_path: 导出基础路径（每个子目录下）
        """
        # 输出到 export_base_path 目录下
        base_export = export_base_path / "ok_data" 
        base_export.mkdir(parents=True, exist_ok=True)
        
        base_export_2hz = export_base_path / "ok_data_2hz" 
        base_export_2hz.mkdir(parents=True, exist_ok=True)

        def ensure_dirs(seq_dir: Path):
            for name in ["front", "rear", "front_left", "front_right", "rear_left", "rear_right", "lidar"]:
                (seq_dir / name).mkdir(parents=True, exist_ok=True)

        for i, group in enumerate(matched_results):
            seq_idx = i // sequence_size
            seq_dir = base_export / f"sequence{seq_idx:05d}"
            seq_dir_2hz = base_export_2hz / f"sequence{seq_idx:05d}"
            ensure_dirs(seq_dir)
            ensure_dirs(seq_dir_2hz)

            lidar_src = Path(group[0])
            cam_srcs = group[1:]

            # 检查整个组是否都满足 2Hz 条件（所有相机+激光雷达）
            is_group_2hz = self.is_near_boundary(lidar_src.name)
            if is_group_2hz:
                for cam_file in cam_srcs:
                    if cam_file and not self.is_near_boundary(Path(cam_file).name):
                        is_group_2hz = False
                        break

            # 相机目标顺序与目录名对应
            dir_order = [
                ("front", "camera/front"),
                ("rear", "camera/rear"),
                ("front_left", "camera/front_left"),
                ("front_right", "camera/front_right"),
                ("rear_left", "camera/rear_left"),
                ("rear_right", "camera/rear_right"),
            ]

            folder_to_index = {name: self.camera_folders.index(path) for name, path in dir_order}

            for cam_name, _ in dir_order:
                src = Path(cam_srcs[folder_to_index[cam_name]])
                dst = seq_dir / cam_name / src.name
                if use_symlink:
                    try:
                        if dst.exists():
                            dst.unlink()
                        relative_src = os.path.relpath(src, os.path.dirname(dst))
                        os.symlink(relative_src, dst)
                    except Exception:
                        shutil.copy2(src, dst)
                    # 只有当整个组都满足 2Hz 条件时，才复制到 2Hz 目录
                    if is_group_2hz:
                        dst_2hz = seq_dir_2hz / cam_name / src.name
                        try:
                            if dst_2hz.exists():
                                dst_2hz.unlink()
                            os.symlink(relative_src, dst_2hz)
                        except Exception:
                            shutil.copy2(src, dst_2hz)
                    
                else:
                    shutil.copy2(src, dst)
                    # 只有当整个组都满足 2Hz 条件时，才复制到 2Hz 目录
                    if is_group_2hz:
                        dst_2hz = seq_dir_2hz / cam_name / src.name
                        shutil.copy2(src, dst_2hz)

            # lidar
            lidar_dst = seq_dir / "lidar" / lidar_src.name
            if use_symlink:
                try:
                    if lidar_dst.exists():
                        lidar_dst.unlink()
                    relative_src = os.path.relpath(lidar_src, os.path.dirname(lidar_dst))
                    os.symlink(relative_src, lidar_dst)
                            
                except Exception:
                    shutil.copy2(lidar_src, lidar_dst)
                # 只有当整个组都满足 2Hz 条件时，才复制到 2Hz 目录
                if is_group_2hz:
                    dst_2hz = seq_dir_2hz / "lidar" / lidar_src.name
                    try:
                        if dst_2hz.exists():
                            dst_2hz.unlink()
                        os.symlink(relative_src, dst_2hz)
                    except Exception:
                        shutil.copy2(lidar_src, dst_2hz)

            else:
                shutil.copy2(lidar_src, lidar_dst)
                # 只有当整个组都满足 2Hz 条件时，才复制到 2Hz 目录
                if is_group_2hz:
                    dst_2hz = seq_dir_2hz / "lidar" / lidar_src.name
                    shutil.copy2(lidar_src, dst_2hz)
    
    def process_subdir(self, subdir_path: Path, output_file: str = "lidar_cameras_files.yaml", simple_list: bool = False, sequence_size: int = 200, symlink: bool = True):
        """
        处理单个子目录
        
        Args:
            subdir_path: 子目录路径
            output_file: 输出文件名
        """
        self.logger.info(f"处理子目录: {subdir_path}")
        
        # 加载文件
        lidar_files, camera_files_list = self.load_files_from_subdir(subdir_path)
        
        if not lidar_files:
            self.logger.warning(f"子目录 {subdir_path} 未找到激光雷达文件")
            return
        
        # 匹配文件
        matched_results = self.match_files(lidar_files, camera_files_list)
        
        if not matched_results:
            self.logger.warning(f"子目录 {subdir_path} 未找到匹配的文件")
            return
        
        # 保存结果到子目录
        output_path = subdir_path / output_file
        if simple_list:
            self.save_results_simple_list(matched_results, str(output_path))
        else:
            self.save_results(matched_results, str(output_path))

        # 导出序列目录到子目录
        self.export_sequences(matched_results, subdir_path, sequence_size=sequence_size, use_symlink=symlink)
        
        self.logger.info(f"子目录 {subdir_path} 处理完成，找到 {len(matched_results)} 组匹配文件")
    
    def process(self, output_file: str = "lidar_cameras_files.yaml", simple_list: bool = False, sequence_size: int = 200, symlink: bool = True):
        """
        执行完整的处理流程（处理所有子目录）
        
        Args:
            output_file: 输出文件名
        """
        try:
            # 检查基础路径是否存在
            if not self.base_path.exists():
                raise FileNotFoundError(f"基础路径不存在: {self.base_path}")
            
            # 查找所有 perception_data_* 子目录
            pattern = str(self.base_path / "perception_data_*")
            subdirs = sorted(glob.glob(pattern))
            if not subdirs:
                raise FileNotFoundError(f"未找到 perception_data_* 子目录: {self.base_path}")
            
            self.logger.info(f"找到 {len(subdirs)} 个子目录，开始逐个处理...")
            
            # 处理每个子目录
            for subdir in subdirs:
                subdir_path = Path(subdir)
                try:
                    self.process_subdir(subdir_path, output_file, simple_list, sequence_size, symlink)
                except Exception as e:
                    self.logger.error(f"处理子目录 {subdir_path} 时发生错误: {e}")
                    continue
            
            self.logger.info("所有子目录处理完成！")
            
        except Exception as e:
            self.logger.error(f"处理过程中发生错误: {e}")
            raise


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='点云和相机数据预处理脚本')
    parser.add_argument('base_path', help='数据根目录路径')
    parser.add_argument('dt', type=float, nargs='?', default=0.0, help='时间补偿时间（默认0.0）')
    parser.add_argument('--time-threshold', type=float, default=0.005, 
                       help='时间戳匹配阈值（秒），默认0.005')
    parser.add_argument('--output', default='lidar_cameras_files.yaml',
                       help='输出文件名，默认lidar_cameras_files.yaml')
    parser.add_argument('--simple-list', action='store_true', help='以简单列表格式保存')
    parser.add_argument('--sequence-size', type=int, default=200, help='每个序列的样本数量')
    parser.add_argument('--no-symlink', action='store_true', help='导出时禁用符号链接，改为复制文件')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='显示详细日志')
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    try:
        preprocessor = DataPreprocessor(args.base_path, args.time_threshold, args.dt)
        preprocessor.process(
            args.output,
            simple_list=args.simple_list,
            sequence_size=args.sequence_size,
            symlink=not args.no_symlink,
        )
    except Exception as e:
        logging.error(f"程序执行失败: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
