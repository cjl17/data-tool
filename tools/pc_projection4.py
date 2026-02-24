import copy
from typing import List, Optional, Tuple
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image
import pcl
import json
import yaml
import pdb
import os
import math
import sys
from pathlib import Path
try:
  import open3d as o3d
  _O3D_AVAILABLE = True
except Exception as _e:
  print("[WARN] Open3D import failed, will skip tensor reader and try legacy fallback:", _e)
  _O3D_AVAILABLE = False

# 时间补偿配置（秒），可在脚本中修改
# 向后补偿使用负值，如 -0.016 表示向后16ms
# 向前补偿使用正值，如 0.016 表示向前16ms
dt = 0.0  # 默认0ms，可根据需要修改

# 是否生成所有帧
all_gen = True  # 默认生成所有帧

# 所有相机名称
CAM_OPTIONS = ["front_3mm", "rear_3mm", "front_left", "front_right", "rear_left", "rear_right"]

if(len(sys.argv) < 2):
    print("需要1个参数: dataset_root")
    print("示例1（单组）: python3 pc_projection4.py <perception_data_xxx_dir>")
    print("  - perception_data_xxx_dir: 数据目录（包含 ok_data 目录）")
    print("示例2（批处理）: python3 pc_projection4.py <first_xxx_dir 或包含first_*的根目录>")
    print("  - first_xxx_dir: 目录下包含 perception_data_* 子目录；标定统一从 first_xxx_dir/parameter 读取")
    print("\n注意: 时间补偿和生成帧数在脚本中配置（dt 和 all_gen 变量）")
    print("脚本会自动搜索所有 sequence 目录并为所有相机生成投影图像")
    exit(1)

data_root = sys.argv[1]
map_data_root = data_root  # map_data_root 等于 data_root

dt1=int(abs(dt*1000))
dt_str=""
if dt<0:
    dt_str = f"-{dt1}ms"
elif dt>0:
    dt_str = f"+{dt1}ms"
else:
  dt_str = f"0ms"

camera_name_map = {
  'front' : 'camera1',
  'rear' : 'camera2',
  'front_left' : 'camera3',
  'front_right' : 'camera4',
  'rear_left' : 'camera5',
  'rear_right' : 'camera6',
  'front_3mm' : 'camera1',
  'rear_3mm' : 'camera2',
}

# 兼容 CAM_* 命名，映射为简洁目录名
cam_alias = {
  'CAM_FRONT': 'front',
  'CAM_BACK': 'rear',
  'CAM_FRONT_LEFT': 'front_left',
  'CAM_FRONT_RIGHT': 'front_right',
  'CAM_BACK_LEFT': 'rear_left',
  'CAM_BACK_RIGHT': 'rear_right',
}

def read_pcd(pcd_file):
  """
  读取 PCD 并返回 Nx4 矩阵（x,y,z,intensity）。
  优先使用 Open3D Tensor API，如失败则回退到 legacy API，并将强度设为距离或零。
  """
  if _O3D_AVAILABLE:
    try:
      # Tensor API
      pcd = o3d.t.io.read_point_cloud(pcd_file)
      positions = pcd.point['positions'].numpy() if 'positions' in pcd.point else None
      intensity = pcd.point['intensity'].numpy() if 'intensity' in pcd.point else None
      if positions is not None:
        if intensity is None:
          rng = np.linalg.norm(positions, axis=1, keepdims=True)
          intensity = rng
        return np.concatenate([positions, intensity], axis=1)
    except Exception as _:
      pass
  # Legacy fallback
  try:
    import open3d as _o3d_legacy
    pcd_legacy = _o3d_legacy.io.read_point_cloud(pcd_file)
    pts = np.asarray(pcd_legacy.points)
    if pts.size == 0:
      return np.zeros((0,4), dtype=np.float32)
    rng = np.linalg.norm(pts, axis=1, keepdims=True)
    return np.concatenate([pts, rng], axis=1)
  except Exception as e:
    print("[ERROR] Failed to read PCD:", e)
    return np.zeros((0,4), dtype=np.float32)

def project(points, image, M1, M2):
  """
  points: Nx3
  image: opencv img, 表示要投影的图像
  M1: 内参矩阵 K, 4*4
  M2: 外参矩阵， 4*4

  return: points 在像素坐标系下的坐标 N*4, 实际只用 N*2

  """
  resolution = image.shape

  coords = points[:, 0:3]
  ones = np.ones(len(coords)).reshape(-1, 1)
  coords = np.concatenate([coords, ones], axis=1)

  range_p = (points[:,0] * points[:,0] + points[:,1] * points[:,1] + points[:,2] * points[:,2])
  range_p = np.sqrt(range_p).reshape(-1, 1)
  coords = np.concatenate([coords, range_p], axis=1)

  transform = copy.deepcopy(M1 @ M2).reshape(4, 4)
  # coords @ transform.T == (transform @ coords.T).T
  coords[:,:4] = coords[:,:4] @ transform.T
  coords = np.concatenate([coords, points[:,-1].reshape(-1, 1)], axis=1)
  coords = coords[np.where(coords[:, 2] > 0)]

  coords[:, 2] = np.clip(coords[:, 2], a_min=1e-5, a_max=1e5)
  coords[:, 0] /= coords[:, 2]
  coords[:, 1] /= coords[:, 2]

  coords = coords[np.where(coords[:, 0] > 0)]
  coords = coords[np.where(coords[:, 0] < resolution[1])]
  coords = coords[np.where(coords[:, 1] > 0)]
  coords = coords[np.where(coords[:, 1] < resolution[0])]


  return coords

def smoothstep(t):
  return 0.5 - 0.5 * math.cos(math.pi * t)  # S 曲线

def intensity_to_rgb(intensity, min_val=0, max_val=150):
  """
  将 intensity 从 [min_val, max_val] 映射为 RGB 从蓝色到红色。
  蓝 (0,0,255) → 红 (255,0,0)
  """
  # 限制范围
  intensity = max(min_val, min(intensity, max_val))
  # 归一化比例
  ratio = (intensity - min_val) / (max_val - min_val)
  ratio = smoothstep(ratio)
  # 线性插值：r从0到255，b从255到0，g恒为0
  r = int(255 * ratio)
  g = 0
  b = int(255 * (1 - ratio))
  return b, g, r

def jet_colormap(intensity, min_val=0, max_val=150):
  """
  将强度值映射为 Jet 伪彩色：蓝 → 青 → 绿 → 黄 → 红
  """
  # 归一化
  t = (intensity - min_val) / (max_val - min_val)
  t = max(0.0, min(1.0, t))

  if t < 0.25:
    # Blue → Cyan
    r = 0
    g = int(4 * t * 255)
    b = 255
  elif t < 0.5:
    # Cyan → Green
    r = 0
    g = 255
    b = int((1 - 4 * (t - 0.25)) * 255)
  elif t < 0.75:
    # Green → Yellow
    r = int(4 * (t - 0.5) * 255)
    g = 255
    b = 0
  else:
    # Yellow → Red
    r = 255
    g = int((1 - 4 * (t - 0.75)) * 255)
    b = 0

  return int(b), int(g), int(r)

def show_with_opencv(image, coords=None, save=None, dir=None, camera_name=None, idx=0):
  """
  image: opencv image
  coords: 像素坐标系下的点, N*4
  camera_name: 相机名称，用于显示
  idx: 帧索引，用于生成文件名
  """
  canvas = image.copy()
  if camera_name:
    cv2.putText(canvas,
                text= camera_name,
                org=(0, 90),
                fontFace=cv2.FONT_HERSHEY_PLAIN,
                fontScale=5.0,
                thickness=5,
                color=(0, 0, 255))
  canvas = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)
  # 画点
  if coords is not None and coords.shape[0] > 0:
    range_p = np.clip(coords[:,-1] * 5, 0, 255).astype(np.int32)
    for index in range(coords.shape[0]):
      p = (int(coords[index, 0]), int(coords[index, 1]))          
      color = jet_colormap(coords[index, -1], min_val=1, max_val=255)
      cv2.circle(canvas, p, 2, color=color, thickness=1)
  canvas = canvas.astype(np.uint8)
  canvas = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

  canvas = cv2.resize(canvas, (1920, 1080))

  if save:
    if (os.path.exists(dir) == False):
      os.makedirs(dir)

    img_canvas = Image.fromarray(canvas)
    # pdb.set_trace()
    camera_projection = dir + "/map" + str('{:0>3d}'.format(idx)) +'.jpeg'
    img_canvas.save(camera_projection)
    # print(camera_projection)
  # cv2.namedWindow("image")  # 创建一个image的窗口
  # cv2.imshow("image", canvas)  # 显示图像
  # cv2.waitKey(0)  # 默认为0，无限等待

def get_calibration(sensor_kit_file, sensor_base_file, camera_intrinsic_file):
  with open(sensor_kit_file, 'r') as file:
    sensor_kit_data = yaml.load(file, Loader=yaml.FullLoader)

  with open(sensor_base_file, 'r') as file:
    sensor_base_data = yaml.load(file, Loader=yaml.FullLoader)
  
  with open(camera_intrinsic_file, 'r') as file:
    camera_intrinsic_data = yaml.load(file, Loader=yaml.FullLoader)

  return sensor_kit_data, sensor_base_data, camera_intrinsic_data

def resolve_calibration_paths(data_root_path: str, camera_key: str, calib_root: str = None):
  """
  解析标定文件路径，支持多种根目录：
  - 环境变量 CALIB_ROOT
  - 显式传入 calib_root（批处理：first_xxx/parameter）
  - data_root/parameter（单组：perception_data_xxx/parameter）
  - 项目内参数目录：<repo>/parameter（兜底）
  找到第一个存在的根后，拼接子路径。
  """
  # 推断项目 parameter 路径
  script_dir = os.path.dirname(os.path.realpath(__file__))
  repo_root = os.path.dirname(os.path.dirname(script_dir))
  candidates = []
  if os.environ.get('CALIB_ROOT'):
    candidates.append(os.environ.get('CALIB_ROOT'))
  if calib_root:
    candidates.append(calib_root)
  candidates.append(os.path.join(data_root_path, 'parameter'))
  candidates.append(os.path.join(repo_root, 'parameter'))

  chosen = None
  for root in candidates:
    if root and os.path.isdir(root):
      chosen = root
      break

  if chosen is None:
    print('[WARN] Calibration root not found. Tried:', candidates)
    exit(1)
    return None, None, None

  sensor_kit_file = os.path.join(chosen, 'sensor_kit/robobus_sensor_kit_description/extrinsic_parameters/sensor_kit_calibration.yaml')
  sensor_base_file = os.path.join(chosen, 'sensor_kit/robobus_sensor_kit_description/extrinsic_parameters/sensors_calibration.yaml')
  camera_intrinsic_file = os.path.join(chosen, f'sensor_kit/robobus_sensor_kit_description/intrinsic_parameters/{camera_name_map[camera_key]}_params.yaml')

  for p in [sensor_kit_file, sensor_base_file, camera_intrinsic_file]:
    if not os.path.isfile(p):
      # print('[WARN] Calibration file missing:', p)
      return None, None, None

  return sensor_kit_file, sensor_base_file, camera_intrinsic_file

def undistort(k_matrix: np.array, d_matrix: np.array, frame):
  h, w = frame.shape[:2]
  # pdb.set_trace()
  mapx, mapy = cv2.initUndistortRectifyMap(k_matrix, d_matrix, None, k_matrix, (w, h), 5)
  return cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

def distortionCorrection(k_matrix: np.array, d_matrix: np.array, frame):
  undistort_frame = undistort(k_matrix, d_matrix, frame)
  return undistort_frame

def distort(camera_intrinsic_data, data_root, sequence_index, camera_name, idx, ok_data_name='ok_data', save=None):
  K = np.array(camera_intrinsic_data['camera_matrix']['data']).reshape(
    camera_intrinsic_data['camera_matrix']['rows'], camera_intrinsic_data['camera_matrix']['cols'])

  D = np.array(camera_intrinsic_data['distortion_coefficients']['data']).reshape(14, 1).astype(np.float32)

  img_dir = os.path.join(data_root, ok_data_name, sequence_index, camera_name)
  list_files = os.listdir(img_dir)
  list_files.sort()
  if len(list_files) == 0:
    raise ValueError(f"目录中没有图片文件: {img_dir}")
  if idx >= len(list_files):
    raise IndexError(f"索引 {idx} 超出范围 (list_files 长度 {len(list_files)}): {list_files[idx-1] if idx > 0 else 'N/A'}")
  img_before = os.path.join(img_dir, list_files[idx])

  resolution=(1920, 1080)
  img = Image.open(img_before)
  img = img.resize(resolution)
  img_bgr = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)

  img_distort_np = distortionCorrection(K, D, img_bgr)

  img_distort_np = cv2.cvtColor(img_distort_np, cv2.COLOR_BGR2RGB)

  # pdb.set_trace()
  if save:
    img_distort = Image.fromarray(img_distort_np)
    img_distort = img_distort.resize((1920, 1080))
    img_distort.save(camera_name +  'distort.jpeg')

  return img_distort_np

def get_M1andM2(sensor_kit_data, sensor_base_data, camera_intrinsic_data, camera_name, camera_name_map):
  M1 = np.eye(4)
  M1[:3][...,:3] = np.array(camera_intrinsic_data['camera_matrix']['data']).reshape(
    camera_intrinsic_data['camera_matrix']['rows'], camera_intrinsic_data['camera_matrix']['cols'])

  
  camera_frame = f"{camera_name_map[camera_name]}/camera_link"
  euler = [sensor_kit_data['sensor_kit_base_link'][camera_frame]['roll'],
            sensor_kit_data['sensor_kit_base_link'][camera_frame]['pitch'],
            sensor_kit_data['sensor_kit_base_link'][camera_frame]['yaw']]
  sensor_kit2camera_R = R.from_euler('xyz', euler)
  sensor_kit2camera_matrix = sensor_kit2camera_R.as_matrix()
  sensor_kit2camera = np.eye(4)
  sensor_kit2camera[:3][...,:3] = sensor_kit2camera_matrix
  sensor_kit2camera[...,-1][:3] = np.array([sensor_kit_data['sensor_kit_base_link'][camera_frame]['x'],
                                          sensor_kit_data['sensor_kit_base_link'][camera_frame]['y'],
                                          sensor_kit_data['sensor_kit_base_link'][camera_frame]['z']])
  # pdb.set_trace()

  euler = [sensor_base_data['base_link']['sensor_kit_base_link']['roll'],
            sensor_base_data['base_link']['sensor_kit_base_link']['pitch'],
            sensor_base_data['base_link']['sensor_kit_base_link']['yaw']]
  base_link2sensor_kit_R = R.from_euler('xyz', euler)
  base_link2sensor_kit_matrix = base_link2sensor_kit_R.as_matrix()
  base_link2sensor_kit = np.eye(4)
  base_link2sensor_kit[:3][...,:3] = base_link2sensor_kit_matrix
  base_link2sensor_kit[...,-1][:3] = np.array([sensor_base_data['base_link']['sensor_kit_base_link']['x'],
                                                sensor_base_data['base_link']['sensor_kit_base_link']['y'],
                                                sensor_base_data['base_link']['sensor_kit_base_link']['z']])

  M2 = np.linalg.inv(sensor_kit2camera) @ np.linalg.inv(base_link2sensor_kit)
  # M2 = base_link2sensor_kit @ sensor_kit2camera
  # pdb.set_trace()

  return M1, M2                                            

def process_sequence_camera(data_root, sequence_index, camera_name_for_calib, camera_name_for_output, map_data_root, dt_str, camera_name_map, idx, ok_data_name='ok_data', calib_root=None):
  """
  处理单个 sequence 和单个相机的投影
  camera_name_for_calib: 用于查找标定文件的相机名称（可能是映射后的，如 front）
  camera_name_for_output: 用于输出目录的相机名称（原始名称，如 front_3mm）
  ok_data_name: ok_data 目录名称（如 'ok_data' 或 'ok_data_2hz'）
  返回: True=成功, False=失败, "skipped"=已存在跳过
  """
  # 检查输出文件是否已存在
  output_dir = os.path.join(map_data_root, f"projection{dt_str}", sequence_index, camera_name_for_output)
  output_file = os.path.join(output_dir, f"map{idx:03d}.jpeg")
  if os.path.exists(output_file):
    return "skipped"
  
  lidar_dir = os.path.join(data_root, ok_data_name, sequence_index, 'lidar')
  if not os.path.exists(lidar_dir):
    print(f"⚠️ 警告：目录 {lidar_dir} 不存在，跳过")
    return False
  
  list_files = sorted([f for f in os.listdir(lidar_dir) if f.endswith('.pcd')])
  
  if not list_files:
    print(f"⚠️ 警告：目录 {lidar_dir} 下没有找到任何 .pcd 文件")
    return False
  if idx < 0 or idx >= len(list_files):
    print(f"❌ 错误：索引 idx={idx} 超出文件列表范围 (长度={len(list_files)})")
    return False

  lidar_file = os.path.join(lidar_dir, list_files[idx])
  points = read_pcd(lidar_file)

  sensor_kit_file, sensor_base_file, camera_intrinsic_file = resolve_calibration_paths(data_root, camera_name_for_calib, calib_root=calib_root)
  if sensor_kit_file is None:
    print(f'[WARN] Skip projection due to missing calibration files for {camera_name_for_calib}.')
    return False
  
  sensor_kit_data, sensor_base_data, camera_intrinsic_data = get_calibration(sensor_kit_file, sensor_base_file, camera_intrinsic_file)
  # 读取图片时使用原始相机名称（camera_name_for_output）
  img = distort(camera_intrinsic_data, data_root, sequence_index, camera_name_for_output, idx, ok_data_name=ok_data_name, save=False)

  M1, M2 = get_M1andM2(sensor_kit_data, sensor_base_data, camera_intrinsic_data, camera_name_for_calib, camera_name_map)
  
  coords = project(points, img, M1, M2)
  show_with_opencv(img, coords=coords, save=True, dir=str(os.path.join(map_data_root, f"projection{dt_str}", sequence_index, camera_name_for_output)), camera_name=camera_name_for_output, idx=idx)
  return True


def process_perception_dir(perception_dir: str, calib_root: str = None):
  """
  处理单个 perception_data_* 目录：遍历 ok_data 目录下所有 sequence 与相机，生成 projection{dt_str}
  calib_root: 优先使用该目录作为 parameter 根（批处理时为 first_xxx/parameter）
  """
  global dt_str
  data_root = perception_dir
  map_data_root = perception_dir

  if not os.path.exists(data_root):
    print(f"错误: 数据目录不存在: {data_root}")
    return

  # 只处理 ok_data 目录（不处理 ok_data_2hz 等）
  ok_data_dir = os.path.join(data_root, 'ok_data')
  if not os.path.exists(ok_data_dir):
    print(f"⚠️ 跳过: 目录 {ok_data_dir} 不存在")
    return

  sequence_dirs = [d for d in os.listdir(ok_data_dir)
                   if os.path.isdir(os.path.join(ok_data_dir, d)) and d.startswith('sequence')]
  sequence_dirs.sort()
  
  if len(sequence_dirs) == 0:
    print(f"⚠️ 跳过: 在 {ok_data_dir} 下没有找到 sequence 目录")
    return

  print(f"\n=== 处理数据: {data_root}")
  print(f"找到 {len(sequence_dirs)} 个 sequence 目录: {sequence_dirs}")
  print(f"将处理 {len(CAM_OPTIONS)} 个相机: {CAM_OPTIONS}")
  if calib_root:
    print(f"标定参数统一从: {calib_root}")

  total_processed = 0
  total_failed = 0
  total_skipped = 0

  for sequence_index in sequence_dirs:
    for camera_name in CAM_OPTIONS:
      camera_dir = os.path.join(data_root, 'ok_data', sequence_index, camera_name)
      if not os.path.exists(camera_dir):
        print(f"⚠️ 跳过: {sequence_index}/{camera_name} (目录不存在)")
        continue

      camera_name_actual = camera_name
      if camera_name in cam_alias:
        camera_name_actual = cam_alias[camera_name]

      if camera_name_actual not in camera_name_map.keys():
        print(f"⚠️ 跳过: {sequence_index}/{camera_name} (不支持的相机名称)")
        continue

      print(f"正在处理: ok_data/{sequence_index}/{camera_name}")

      # 检查 LiDAR 和相机文件数量，取较小值作为处理帧数
      lidar_dir = os.path.join(data_root, 'ok_data', sequence_index, 'lidar')
      camera_dir = os.path.join(data_root, 'ok_data', sequence_index, camera_name)
      
      if not os.path.exists(lidar_dir):
        print(f"⚠️ 警告: {lidar_dir} 不存在，跳过")
        continue
      
      if not os.path.exists(camera_dir):
        print(f"⚠️ 警告: {camera_dir} 不存在，跳过")
        continue

      lidar_files = sorted([f for f in os.listdir(lidar_dir) if f.endswith('.pcd')])
      camera_files = sorted([f for f in os.listdir(camera_dir) if f.endswith('.jpg')])
      
      if len(lidar_files) == 0:
        print(f"⚠️ 警告: {lidar_dir} 下没有 .pcd 文件，跳过")
        continue
      
      if len(camera_files) == 0:
        print(f"⚠️ 警告: {camera_dir} 下没有 .jpg 文件，跳过")
        continue

      # 使用 LiDAR 和相机文件数量的较小值，确保不会索引越界
      if all_gen:
        count = min(len(lidar_files), len(camera_files))
      else:
        count = min(5, len(lidar_files), len(camera_files))

      if count == 0:
        print(f"⚠️ 警告: 没有可处理的文件，跳过")
        continue

      success_count = 0
      skipped_count = 0
      for idx in range(count):
        try:
          result = process_sequence_camera(
            data_root,
            sequence_index,
            camera_name_actual,
            camera_name,
            map_data_root,
            dt_str,
            camera_name_map,
            idx,
            ok_data_name='ok_data',
            calib_root=calib_root,
          )
          if result == "skipped":
            skipped_count += 1
            total_skipped += 1
          elif result:
            success_count += 1
            total_processed += 1
          else:
            total_failed += 1
        except Exception as e:
          print(f"❌ 处理 ok_data/{sequence_index}/{camera_name} 第 {idx} 帧时出错: {e}")
          total_failed += 1

      print(f"✓ 完成: ok_data/{sequence_index}/{camera_name} (成功:{success_count}, 跳过:{skipped_count}, 失败:{count-success_count-skipped_count}/{count} 帧)")

  print(f"\n数据 {data_root} 处理完成！成功 {total_processed}，跳过 {total_skipped}，失败 {total_failed}")


def find_first_dirs(root: Path) -> List[Path]:
  if root.name.startswith("first_"):
    return [root]
  return sorted([p for p in root.glob("first_*") if p.is_dir()])


if __name__ == '__main__':
  # 支持两种模式：
  # 1) 单组：传入 perception_data_* 目录（包含 ok_data）
  # 2) 批处理：传入 first_* 目录 或 包含 first_* 的根目录
  root = Path(sys.argv[1]).resolve()

  # 单组模式（直接包含 ok_data）
  if (root / "ok_data").is_dir():
    process_perception_dir(str(root), calib_root=None)
    sys.exit(0)

  # 批处理：遍历 first_* / perception_data_*
  first_dirs = find_first_dirs(root)
  if not first_dirs:
    print(f"错误: 未找到 first_* 目录，且 {root} 下无 ok_data")
    sys.exit(1)

  for first_dir in first_dirs:
    calib_root = str(first_dir / "parameter")
    # 只要目录存在就作为优先标定根
    if not os.path.isdir(calib_root):
      calib_root = None

    print(f"\n########## 批处理 first 目录: {first_dir} ##########")
    for p in sorted(first_dir.glob("perception_data_*")):
      process_perception_dir(str(p), calib_root=calib_root)
