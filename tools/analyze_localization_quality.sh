#!/bin/bash

# ------------------ 参数解析 ------------------
CSV_FILE="/media/ipc/AQLoopCloseData1/perception_csv/localization_all_20260130_173932.csv"
OUT_DIR="/media/ipc/AQLoopCloseData1/perception_csv/"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -f|--file) CSV_FILE="$2"; shift ;;
        -o|--outdir) OUT_DIR="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

mkdir -p "$OUT_DIR"

# ------------------ Python 脚本嵌入 ------------------
python3 - <<EOF
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

CSV_FILE = "$CSV_FILE"
OUT_DIR = "$OUT_DIR"

print("=== 开始分析数据 ===")

# ------------------ 读取 CSV 并强制数字 ------------------
df = pd.read_csv(CSV_FILE)
print(f"读取到 {len(df)} 行数据")

# 强制转换关键列为数字，错误值设为 NaN
for col in ['timestamp_s', 'vx', 'fused_vx', 'imu_wz', 'ax']:
    if col in df.columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    else:
        print(f"警告: 列 '{col}' 不存在")

# ------------------ 智能阈值计算 ------------------
print("\\n=== 计算智能阈值 ===")

# 1. 计算时间间隔统计
df['dt'] = df['timestamp_s'].diff()
dt_valid = df['dt'].dropna()
if len(dt_valid) > 0:
    dt_median = dt_valid.median()
    dt_std = dt_valid.std()
    dt_q99 = dt_valid.abs().quantile(0.99)
    print(f"时间间隔统计:")
    print(f"  中位数: {dt_median:.6f}s (≈{1/dt_median:.1f}Hz)")
    print(f"  标准差: {dt_std:.6f}s")
    print(f"  99%分位数: {dt_q99:.6f}s")
    
    # 时间戳阈值（基于统计）
    TS_THRESHOLD_LOW = max(0.001, dt_median - 3*dt_std) if dt_std > 0 else dt_median * 0.5
    TS_THRESHOLD_HIGH = dt_median + 3*dt_std if dt_std > 0 else dt_median * 2.0
else:
    print("警告: 无法计算时间间隔统计")
    TS_THRESHOLD_LOW = 0.005
    TS_THRESHOLD_HIGH = 0.03

print(f"时间阈值: 采样过快 < {TS_THRESHOLD_LOW:.6f}s, 采样过慢 > {TS_THRESHOLD_HIGH:.6f}s")

# 2. 基于车辆动力学的速度跳变阈值
MAX_VEHICLE_ACCEL = 8.0  # m/s² (最大制动加速度)
MAX_VEHICLE_DECEL = -10.0  # m/s²
if dt_median > 0:
    # 基础阈值 = 最大加速度 × 采样间隔 × 安全系数
    base_dvx_threshold = abs(MAX_VEHICLE_DECEL) * dt_median * 2.0
    print(f"基础速度跳变阈值: {base_dvx_threshold:.3f} m/s (基于{dt_median:.3f}s间隔)")
    
    # 统计阈值（3倍标准差）
    dvx_stats = df['vx'].diff().dropna().abs()
    if len(dvx_stats) > 10:
        dvx_stat_threshold = dvx_stats.quantile(0.995)  # 99.5%分位数
        print(f"统计速度跳变阈值: {dvx_stat_threshold:.3f} m/s (99.5%分位数)")
        
        # 取两者较小值，更敏感
        DVX_THRESHOLD = min(base_dvx_threshold, dvx_stat_threshold * 2)
        DFUSED_VX_THRESHOLD = DVX_THRESHOLD * 0.7  # 融合速度应更稳定
    else:
        DVX_THRESHOLD = base_dvx_threshold
        DFUSED_VX_THRESHOLD = base_dvx_threshold * 0.7
else:
    DVX_THRESHOLD = 0.5
    DFUSED_VX_THRESHOLD = 0.35

print(f"速度跳变阈值: vx={DVX_THRESHOLD:.3f}m/s, fused_vx={DFUSED_VX_THRESHOLD:.3f}m/s")

# 3. IMU和加速度阈值（基于统计）
DWZ_THRESHOLD = 0.5
DAX_THRESHOLD = 5.0

for col, threshold_name in [('imu_wz', 'DWZ_THRESHOLD'), ('ax', 'DAX_THRESHOLD')]:
    if col in df.columns:
        diff_vals = df[col].diff().dropna().abs()
        if len(diff_vals) > 10:
            q999 = diff_vals.quantile(0.999)  # 99.9%分位数，更严格
            q995 = diff_vals.quantile(0.995)  # 99.5%分位数
            if q999 > 0:
                if col == 'imu_wz':
                    DWZ_THRESHOLD = min(2.0, max(0.05, q999 * 1.2))  # 限制在合理范围
                else:
                    DAX_THRESHOLD = min(10.0, max(0.5, q999 * 1.2))
                print(f"{col} 跳变阈值: {q995:.3f}(99.5%) -> {q999:.3f}(99.9%) -> 使用: {DWZ_THRESHOLD if col=='imu_wz' else DAX_THRESHOLD:.3f}")

print(f"IMU/加速度阈值: imu_wz={DWZ_THRESHOLD:.3f}, ax={DAX_THRESHOLD:.3f}")

# ------------------ 计算差分 ------------------
df['dvx'] = df['vx'].diff()
df['dfused_vx'] = df['fused_vx'].diff()
df['dimu_wz'] = df['imu_wz'].diff()
df['dax'] = df['ax'].diff()

# ------------------ 异常检测 ------------------
print("\\n=== 开始异常检测 ===")

# 时间戳异常
df['ts_anomaly'] = False
if 'dt' in df.columns:
    df['ts_anomaly'] = (df['dt'] <= 0) | (df['dt'] < TS_THRESHOLD_LOW) | (df['dt'] > TS_THRESHOLD_HIGH)

# 速度异常
df['dvx_anomaly'] = (df['dvx'].abs() > DVX_THRESHOLD)
df['dfused_vx_anomaly'] = (df['dfused_vx'].abs() > DFUSED_VX_THRESHOLD)

# IMU和加速度异常
df['dimu_wz_anomaly'] = (df['dimu_wz'].abs() > DWZ_THRESHOLD)
df['dax_anomaly'] = (df['dax'].abs() > DAX_THRESHOLD)

# ------------------ 生成异常报告 ------------------
report_rows = []
print("\\n检测到以下异常:")
for idx, row in df.iterrows():
    reasons = []
    
    # 时间戳异常
    if row['ts_anomaly']:
        if pd.isna(row['dt']):
            reasons.append("时间间隔NaN")
        elif row['dt'] <= 0:
            reasons.append(f"时间倒退(dt={row['dt']:.6f})")
        elif row['dt'] < TS_THRESHOLD_LOW:
            reasons.append(f"采样过快(dt={row['dt']:.6f}<{TS_THRESHOLD_LOW:.6f})")
        else:
            reasons.append(f"采样过慢(dt={row['dt']:.6f}>{TS_THRESHOLD_HIGH:.6f})")
    
    # 速度异常
    if row['dvx_anomaly'] and not pd.isna(row['dvx']):
        reasons.append(f"定位速度跳变(dvx={row['dvx']:.3f}>{DVX_THRESHOLD:.3f})")
    if row['dfused_vx_anomaly'] and not pd.isna(row['dfused_vx']):
        reasons.append(f"融合速度跳变(dfused_vx={row['dfused_vx']:.3f}>{DFUSED_VX_THRESHOLD:.3f})")
    
    # IMU和加速度异常
    if row['dimu_wz_anomaly'] and not pd.isna(row['dimu_wz']):
        reasons.append(f"IMU yaw跳变(dimu_wz={row['dimu_wz']:.3f}>{DWZ_THRESHOLD:.3f})")
    if row['dax_anomaly'] and not pd.isna(row['dax']):
        reasons.append(f"前向加速度跳变(dax={row['dax']:.3f}>{DAX_THRESHOLD:.3f})")

    if reasons:
        report_rows.append({
            "row": idx+1,
            "timestamp_s": row['timestamp_s'],
            "dt": row['dt'] if 'dt' in row else None,
            "vx": row['vx'] if 'vx' in row else None,
            "fused_vx": row['fused_vx'] if 'fused_vx' in row else None,
            "reasons": "; ".join(reasons)
        })
        # 显示前10个异常
        if len(report_rows) <= 10:
            print(f"行{idx+1}: {reasons}")

report_df = pd.DataFrame(report_rows)
if len(report_df) > 0:
    report_csv = os.path.join(OUT_DIR, "localization_anomaly_report.csv")
    report_df.to_csv(report_csv, index=False)
    print(f"\\n共检测到 {len(report_df)} 个异常点")
    print(f"异常报告生成完成: {report_csv}")
    
    # 统计各类异常
    print("\\n=== 异常类型统计 ===")
    anomaly_counts = {
        "时间倒退": report_df['reasons'].str.contains("时间倒退").sum(),
        "采样过快": report_df['reasons'].str.contains("采样过快").sum(),
        "采样过慢": report_df['reasons'].str.contains("采样过慢").sum(),
        "定位速度跳变": report_df['reasons'].str.contains("定位速度跳变").sum(),
        "融合速度跳变": report_df['reasons'].str.contains("融合速度跳变").sum(),
        "IMU yaw跳变": report_df['reasons'].str.contains("IMU yaw跳变").sum(),
        "前向加速度跳变": report_df['reasons'].str.contains("前向加速度跳变").sum()
    }
    for anomaly_type, count in anomaly_counts.items():
        if count > 0:
            print(f"{anomaly_type}: {count}次")
else:
    print("未检测到异常")

# ------------------ 可视化 ------------------
print("\\n=== 生成可视化图表 ===")
plt.figure(figsize=(16,12))

# 1. 速度图表
plt.subplot(4,1,1)
if 'vx' in df.columns and 'fused_vx' in df.columns:
    plt.plot(df['timestamp_s'], df['vx'], label='vx (定位速度)', alpha=0.7, linewidth=1)
    plt.plot(df['timestamp_s'], df['fused_vx'], label='fused_vx (融合速度)', alpha=0.7, linewidth=1)
    
    # 标记异常点
    if 'dvx_anomaly' in df.columns:
        anomaly_mask = df['dvx_anomaly'] & ~df['vx'].isna()
        plt.scatter(df.loc[anomaly_mask, 'timestamp_s'], df.loc[anomaly_mask, 'vx'],
                   color='red', label='vx异常', s=20, alpha=0.7, marker='o')
    
    if 'dfused_vx_anomaly' in df.columns:
        anomaly_mask = df['dfused_vx_anomaly'] & ~df['fused_vx'].isna()
        plt.scatter(df.loc[anomaly_mask, 'timestamp_s'], df.loc[anomaly_mask, 'fused_vx'],
                   color='orange', label='fused_vx异常', s=20, alpha=0.7, marker='x')
    
    plt.ylabel("速度 (m/s)")
    plt.legend(loc='upper right', fontsize=8)
    plt.grid(True, alpha=0.3)
    plt.title("速度数据异常检测")

# 2. IMU和加速度图表
plt.subplot(4,1,2)
if 'imu_wz' in df.columns:
    plt.plot(df['timestamp_s'], df['imu_wz'], label='imu_wz (偏航角速度)', alpha=0.7, linewidth=1, color='green')
if 'ax' in df.columns:
    plt.plot(df['timestamp_s'], df['ax'], label='ax (前向加速度)', alpha=0.7, linewidth=1, color='purple')

# 标记异常点
if 'dimu_wz_anomaly' in df.columns and 'imu_wz' in df.columns:
    anomaly_mask = df['dimu_wz_anomaly'] & ~df['imu_wz'].isna()
    plt.scatter(df.loc[anomaly_mask, 'timestamp_s'], df.loc[anomaly_mask, 'imu_wz'],
               color='red', label='imu_wz异常', s=20, alpha=0.7, marker='o')

if 'dax_anomaly' in df.columns and 'ax' in df.columns:
    anomaly_mask = df['dax_anomaly'] & ~df['ax'].isna()
    plt.scatter(df.loc[anomaly_mask, 'timestamp_s'], df.loc[anomaly_mask, 'ax'],
               color='orange', label='ax异常', s=20, alpha=0.7, marker='x')

plt.ylabel("IMU/加速度")
plt.legend(loc='upper right', fontsize=8)
plt.grid(True, alpha=0.3)

# 3. 时间间隔图表
plt.subplot(4,1,3)
if 'dt' in df.columns:
    plt.plot(df['timestamp_s'], df['dt'], label='时间间隔(dt)', alpha=0.7, linewidth=1, color='blue')
    
    # 标记时间异常点
    if 'ts_anomaly' in df.columns:
        anomaly_mask = df['ts_anomaly'] & ~df['dt'].isna()
        plt.scatter(df.loc[anomaly_mask, 'timestamp_s'], df.loc[anomaly_mask, 'dt'],
                   color='red', label='时间异常', s=20, alpha=0.7)
    
    # 添加阈值线
    plt.axhline(y=TS_THRESHOLD_LOW, color='orange', linestyle='--', alpha=0.5, label=f'过快阈值({TS_THRESHOLD_LOW:.4f}s)')
    plt.axhline(y=TS_THRESHOLD_HIGH, color='red', linestyle='--', alpha=0.5, label=f'过慢阈值({TS_THRESHOLD_HIGH:.4f}s)')
    
    plt.ylabel("时间间隔 (s)")
    plt.xlabel("时间戳 (s)")
    plt.legend(loc='upper right', fontsize=8)
    plt.grid(True, alpha=0.3)

# 4. 异常密度图
plt.subplot(4,1,4)
if len(report_df) > 0:
    # 计算每100行的异常数量
    window_size = 100
    anomaly_density = []
    timestamps = []
    
    for i in range(0, len(df), window_size):
        end_idx = min(i + window_size, len(df))
        anomaly_count = df.iloc[i:end_idx][['ts_anomaly', 'dvx_anomaly', 'dfused_vx_anomaly', 
                                          'dimu_wz_anomaly', 'dax_anomaly']].any(axis=1).sum()
        anomaly_density.append(anomaly_count / (end_idx - i) * 100)  # 百分比
        timestamps.append(df['timestamp_s'].iloc[i])
    
    plt.bar(timestamps, anomaly_density, width=window_size*dt_median if dt_median>0 else 1, 
           alpha=0.7, color='coral', edgecolor='darkred')
    plt.ylabel("异常密度 (%)")
    plt.xlabel("时间戳 (s)")
    plt.title(f"异常密度分布 (窗口={window_size}行)")
    plt.grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plot_file = os.path.join(OUT_DIR, "localization_anomaly_plot.png")
plt.savefig(plot_file, dpi=200, bbox_inches='tight')
print(f"可视化图表已保存: {plot_file}")

# 显示数据质量统计
print("\\n=== 数据质量统计 ===")
total_rows = len(df)
valid_rows = {
    'timestamp': df['timestamp_s'].notna().sum(),
    'vx': df['vx'].notna().sum(),
    'fused_vx': df['fused_vx'].notna().sum(),
    'imu_wz': df['imu_wz'].notna().sum(),
    'ax': df['ax'].notna().sum()
}

for col, count in valid_rows.items():
    percentage = count / total_rows * 100
    print(f"{col}: {count}/{total_rows} ({percentage:.1f}%)")

# 时间戳范围
if 'timestamp_s' in df.columns and len(df) > 0:
    time_range = df['timestamp_s'].iloc[-1] - df['timestamp_s'].iloc[0]
    print(f"\\n时间范围: {time_range:.2f}秒 ({time_range/3600:.2f}小时)")
    print(f"起始时间: {df['timestamp_s'].iloc[0]:.2f}")
    print(f"结束时间: {df['timestamp_s'].iloc[-1]:.2f}")

print("\\n分析完成！")
EOF