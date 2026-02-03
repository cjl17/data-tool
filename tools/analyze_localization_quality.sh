#!/bin/bash

# ------------------ 参数解析 ------------------
CSV_FILES=()
OUT_DIR="/media/ipc/AQLoopCloseData1/perception_csv/"
RECURSIVE=false
FILE_LIST=""

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -f|--file) 
            # 支持多个文件或通配符
            shift
            while [[ "$#" -gt 0 && ! "$1" =~ ^- ]]; do
                # 支持通配符展开
                for file in $1; do
                    if [[ -f "$file" ]]; then
                        CSV_FILES+=("$file")
                    elif [[ "$file" == *"*"* ]]; then
                        # 通配符模式，展开
                        for matched_file in $file; do
                            if [[ -f "$matched_file" ]]; then
                                CSV_FILES+=("$matched_file")
                            fi
                        done
                    fi
                done
                shift
            done
            ;;
        -d|--dir)
            # 处理目录中的所有CSV文件
            if [[ -d "$2" ]]; then
                maxdepth_flag="-maxdepth 1"
                if [[ "$RECURSIVE" == true ]]; then
                    maxdepth_flag=""
                fi
                while IFS= read -r -d '' file; do
                    CSV_FILES+=("$file")
                done < <(find "$2" $maxdepth_flag -type f -name "*.csv" -print0 2>/dev/null | sort -z)
            else
                echo "错误: 目录 '$2' 不存在"
                exit 1
            fi
            shift
            ;;
        -l|--list)
            # 从文件列表读取CSV文件路径
            FILE_LIST="$2"
            if [[ ! -f "$FILE_LIST" ]]; then
                echo "错误: 文件列表 '$FILE_LIST' 不存在"
                exit 1
            fi
            while IFS= read -r line || [[ -n "$line" ]]; do
                line=$(echo "$line" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')  # 去除首尾空格
                if [[ -n "$line" && ! "$line" =~ ^# ]]; then  # 忽略空行和注释行
                    if [[ -f "$line" ]]; then
                        CSV_FILES+=("$line")
                    else
                        echo "警告: 文件不存在，跳过: $line"
                    fi
                fi
            done < "$FILE_LIST"
            shift
            ;;
        -r|--recursive) RECURSIVE=true ;;
        -o|--outdir) OUT_DIR="$2"; shift ;;
        -h|--help)
            echo "用法: $0 [选项] [文件1] [文件2] ..."
            echo "选项:"
            echo "  -f, --file FILE1 [FILE2 ...]  指定一个或多个CSV文件（支持通配符）"
            echo "  -d, --dir DIR                  处理目录中的所有CSV文件"
            echo "  -l, --list FILE                从文件列表读取CSV文件路径（每行一个路径）"
            echo "  -r, --recursive                递归查找子目录中的CSV文件（与-d一起使用）"
            echo "  -o, --outdir DIR               输出目录（默认: $OUT_DIR）"
            echo "  -h, --help                     显示此帮助信息"
            echo ""
            echo "说明:"
            echo "  1. 可以直接在命令行末尾传入多个CSV文件路径（不需要-f参数）"
            echo "  2. 如果不指定任何文件，将自动处理默认目录下的所有CSV文件"
            echo ""
            echo "示例:"
            echo "  $0                              # 处理默认目录下的所有CSV文件"
            echo "  $0 file1.csv file2.csv         # 直接处理指定的文件"
            echo "  $0 -f file1.csv file2.csv      # 使用-f参数处理指定的文件"
            echo "  $0 -f *.csv                     # 使用通配符处理当前目录的CSV文件"
            echo "  $0 -d /path/to/csv/dir          # 处理指定目录下的所有CSV文件"
            echo "  $0 -d /path/to/csv/dir -r       # 递归处理目录及其子目录下的所有CSV文件"
            echo "  $0 -l file_list.txt             # 从文件列表读取CSV文件路径"
            exit 0
            ;;
        -*)
            echo "未知参数: $1 (使用 -h 查看帮助)"
            exit 1
            ;;
        *)
            # 直接传入的文件路径（不是以-开头的参数）
            if [[ -f "$1" ]]; then
                CSV_FILES+=("$1")
            elif [[ -d "$1" ]]; then
                echo "提示: '$1' 是目录，使用 -d 参数来指定目录"
            else
                echo "警告: 文件不存在，跳过: $1"
            fi
            ;;
    esac
    shift
done

# 如果没有指定文件，自动查找默认目录下的所有CSV文件
if [[ ${#CSV_FILES[@]} -eq 0 ]]; then
    DEFAULT_DIR="/media/ipc/AQLoopCloseData1/perception_csv/"
    if [[ -d "$DEFAULT_DIR" ]]; then
        maxdepth_flag="-maxdepth 1"
        if [[ "$RECURSIVE" == true ]]; then
            maxdepth_flag=""
            echo "未指定文件，递归查找目录: $DEFAULT_DIR"
        else
            echo "未指定文件，自动查找目录: $DEFAULT_DIR"
        fi
        while IFS= read -r -d '' file; do
            CSV_FILES+=("$file")
        done < <(find "$DEFAULT_DIR" $maxdepth_flag -type f -name "*.csv" -print0 2>/dev/null | sort -z)
        
        if [[ ${#CSV_FILES[@]} -eq 0 ]]; then
            echo "错误: 在目录 '$DEFAULT_DIR' 中未找到CSV文件"
            echo "使用 -h 查看帮助"
            exit 1
        else
            echo "找到 ${#CSV_FILES[@]} 个CSV文件，将批量处理"
        fi
    else
        echo "错误: 默认目录 '$DEFAULT_DIR' 不存在"
        echo "使用 -h 查看帮助"
        exit 1
    fi
fi

# 去重文件列表
if [[ ${#CSV_FILES[@]} -gt 0 ]]; then
    declare -A seen
    UNIQUE_FILES=()
    for file in "${CSV_FILES[@]}"; do
        if [[ -z "${seen[$file]}" ]]; then
            seen[$file]=1
            UNIQUE_FILES+=("$file")
        fi
    done
    CSV_FILES=("${UNIQUE_FILES[@]}")
fi

if [[ ${#CSV_FILES[@]} -eq 0 ]]; then
    echo "错误: 没有找到有效的CSV文件"
    echo "使用 -h 查看帮助"
    exit 1
fi

echo ""
echo "=========================================="
echo "批量处理配置"
echo "=========================================="
echo "找到 ${#CSV_FILES[@]} 个CSV文件"
echo "输出目录: $OUT_DIR"
echo ""

mkdir -p "$OUT_DIR"

# 将文件列表传递给Python脚本（每行一个文件）
CSV_FILES_STR=$(printf '%s\n' "${CSV_FILES[@]}")

# ------------------ Python 脚本嵌入 ------------------
python3 - <<EOF
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from pathlib import Path

CSV_FILES_STR = """$CSV_FILES_STR"""
OUT_DIR = "$OUT_DIR"

# 解析文件列表
CSV_FILES = [f.strip() for f in CSV_FILES_STR.strip().split('\n') if f.strip()]
CSV_FILES = [f for f in CSV_FILES if os.path.exists(f)]

if len(CSV_FILES) == 0:
    print("错误: 没有找到有效的CSV文件")
    sys.exit(1)

print(f"=== 找到 {len(CSV_FILES)} 个CSV文件，开始批量处理 ===\n")
print("文件列表:")
for i, f in enumerate(CSV_FILES, 1):
    file_size = os.path.getsize(f) / (1024*1024) if os.path.exists(f) else 0
    print(f"  {i}. {os.path.basename(f)} ({file_size:.2f} MB)")
print()
print(f"开始时间: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}")
print()

# 汇总统计
summary_stats = []

def analyze_single_file(csv_file, file_idx, total_files):
    """分析单个CSV文件"""
    file_name = os.path.basename(csv_file)
    file_stem = Path(csv_file).stem
    
    print(f"\n{'='*60}")
    print(f"[{file_idx}/{total_files}] 处理文件: {file_name}")
    print(f"{'='*60}")
    
    try:
        # ------------------ 读取 CSV 并强制数字 ------------------
        df = pd.read_csv(csv_file)
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
            dt_median = 0.01

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
        
        # 使用文件名作为前缀
        report_csv = os.path.join(OUT_DIR, f"{file_stem}_anomaly_report.csv")
        plot_file = os.path.join(OUT_DIR, f"{file_stem}_anomaly_plot.png")
        
        if len(report_df) > 0:
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
            anomaly_counts = {
                "时间倒退": 0, "采样过快": 0, "采样过慢": 0,
                "定位速度跳变": 0, "融合速度跳变": 0,
                "IMU yaw跳变": 0, "前向加速度跳变": 0
            }

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
        plt.savefig(plot_file, dpi=200, bbox_inches='tight')
        plt.close()  # 关闭图形以释放内存
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
            percentage = count / total_rows * 100 if total_rows > 0 else 0
            print(f"{col}: {count}/{total_rows} ({percentage:.1f}%)")

        # 时间戳范围
        time_range = 0
        if 'timestamp_s' in df.columns and len(df) > 0:
            time_range = df['timestamp_s'].iloc[-1] - df['timestamp_s'].iloc[0]
            print(f"\\n时间范围: {time_range:.2f}秒 ({time_range/3600:.2f}小时)")
            print(f"起始时间: {df['timestamp_s'].iloc[0]:.2f}")
            print(f"结束时间: {df['timestamp_s'].iloc[-1]:.2f}")

        # 收集统计信息用于汇总
        return {
            'file': file_name,
            'total_rows': total_rows,
            'anomaly_count': len(report_df) if len(report_df) > 0 else 0,
            'anomaly_counts': anomaly_counts,
            'valid_rows': valid_rows,
            'time_range': time_range,
            'dt_median': dt_median if 'dt_median' in locals() else 0
        }
        
    except Exception as e:
        print(f"\\n错误: 处理文件 {file_name} 时出错: {str(e)}")
        import traceback
        traceback.print_exc()
        return {
            'file': file_name,
            'error': str(e)
        }

# ------------------ 批量处理所有文件 ------------------
import time
total_files = len(CSV_FILES)
success_count = 0
error_count = 0
start_time = time.time()

for idx, csv_file in enumerate(CSV_FILES, 1):
    file_start_time = time.time()
    try:
        stats = analyze_single_file(csv_file, idx, total_files)
        if stats:
            summary_stats.append(stats)
            if 'error' in stats:
                error_count += 1
                print(f"\n✗ 文件 {idx}/{total_files} 处理失败: {stats.get('error', '未知错误')}")
            else:
                success_count += 1
                elapsed = time.time() - file_start_time
                print(f"\n✓ 文件 {idx}/{total_files} 处理完成 (耗时 {elapsed:.1f}秒)")
        else:
            error_count += 1
            print(f"\n✗ 文件 {idx}/{total_files} 处理失败: 返回空结果")
    except Exception as e:
        error_count += 1
        print(f"\n✗ 文件 {idx}/{total_files} 处理失败: {str(e)}")
        summary_stats.append({
            'file': os.path.basename(csv_file),
            'error': str(e)
        })
    
    # 显示总体进度
    elapsed_total = time.time() - start_time
    avg_time = elapsed_total / idx if idx > 0 else 0
    remaining_files = total_files - idx
    estimated_remaining = avg_time * remaining_files if remaining_files > 0 else 0
    print(f"进度: {idx}/{total_files} ({idx/total_files*100:.1f}%) | "
          f"成功: {success_count} | 失败: {error_count} | "
          f"预计剩余时间: {estimated_remaining/60:.1f}分钟")

# ------------------ 生成汇总报告 ------------------
total_elapsed = time.time() - start_time
print(f"\\n\\n{'='*60}")
print(f"=== 批量处理完成，生成汇总报告 ===")
print(f"{'='*60}")
print(f"总耗时: {total_elapsed/60:.1f}分钟 ({total_elapsed:.1f}秒)")
print(f"成功: {success_count} 个文件")
print(f"失败: {error_count} 个文件")
print(f"{'='*60}\\n")

if len(summary_stats) > 0:
    summary_rows = []
    total_anomalies = 0
    total_rows_all = 0
    
    # 汇总所有异常类型
    total_anomaly_counts = {
        "时间倒退": 0, "采样过快": 0, "采样过慢": 0,
        "定位速度跳变": 0, "融合速度跳变": 0,
        "IMU yaw跳变": 0, "前向加速度跳变": 0
    }
    
    for stats in summary_stats:
        if 'error' in stats:
            summary_rows.append({
                '文件': stats['file'],
                '状态': f"错误: {stats['error']}",
                '总行数': 0,
                '异常数': 0,
                '异常率(%)': "N/A",
                '时间范围(小时)': "N/A",
                '采样频率(Hz)': "N/A"
            })
        else:
            total_anomalies += stats['anomaly_count']
            total_rows_all += stats['total_rows']
            
            # 累加异常类型统计
            if 'anomaly_counts' in stats:
                for anomaly_type, count in stats['anomaly_counts'].items():
                    if anomaly_type in total_anomaly_counts:
                        total_anomaly_counts[anomaly_type] += count
            
            summary_rows.append({
                '文件': stats['file'],
                '状态': '成功',
                '总行数': stats['total_rows'],
                '异常数': stats['anomaly_count'],
                '异常率(%)': f"{stats['anomaly_count']/stats['total_rows']*100:.2f}" if stats['total_rows'] > 0 else "0.00",
                '时间范围(小时)': f"{stats['time_range']/3600:.2f}" if stats['time_range'] > 0 else "0.00",
                '采样频率(Hz)': f"{1/stats['dt_median']:.1f}" if stats['dt_median'] > 0 else "N/A"
            })
    
    summary_df = pd.DataFrame(summary_rows)
    summary_csv = os.path.join(OUT_DIR, "localization_summary_report.csv")
    summary_df.to_csv(summary_csv, index=False, encoding='utf-8-sig')
    
    print(f"\\n处理了 {len(summary_stats)} 个文件")
    print(f"总行数: {total_rows_all:,}")
    print(f"总异常数: {total_anomalies:,}")
    if total_rows_all > 0:
        print(f"总体异常率: {total_anomalies/total_rows_all*100:.2f}%")
    
    # 显示异常类型汇总
    print(f"\\n=== 异常类型汇总 ===")
    for anomaly_type, count in total_anomaly_counts.items():
        if count > 0:
            percentage = count / total_anomalies * 100 if total_anomalies > 0 else 0
            print(f"  {anomaly_type}: {count:,} 次 ({percentage:.1f}%)")
    
    print(f"\\n汇总报告已保存: {summary_csv}")
    print("\\n详细报告:")
    print(summary_df.to_string(index=False))

print("\\n所有文件分析完成！")
EOF