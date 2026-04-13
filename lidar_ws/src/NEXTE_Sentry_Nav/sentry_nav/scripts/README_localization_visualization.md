# 定位数据可视化工具

## 概述
本工具用于可视化机器人定位过程中保存的里程计和全局定位数据。通过分析定位轨迹、速度、姿态等关键指标，可以评估定位系统的性能和稳定性。

## 功能特性
- **2D轨迹图**: 显示里程计和全局定位的XY平面轨迹，包含起点和终点标记
- **3D轨迹图**: 如果有Z坐标数据，可显示3D轨迹图
- **时间序列分析**: 显示X、Y、Z位置随时间的变化曲线
- **速度分析**: 计算并显示线速度随时间变化（如果数据中包含速度信息）
- **姿态角分析**: 将四元数转换为欧拉角（roll, pitch, yaw）并显示随时间变化
- **定位对比分析**: 对比里程计和全局定位的偏差，计算平均偏差、最大偏差等指标
- **评估指标计算**: 自动计算总移动距离、平均速度、采样频率等指标
- **轨迹动画**: 生成动态轨迹跟踪动画
- **HTML报告**: 自动生成包含所有图表和指标的综合报告

## 系统要求
- Python 3.6+
- 必要Python库:
  - numpy
  - pandas
  - matplotlib
  - matplotlib.animation (用于动画)
  - mpl_toolkits.mplot3d (用于3D可视化)
  - scipy (用于四元数转换)

## 安装依赖
```bash
pip install numpy pandas matplotlib scipy
```

## 数据格式说明

### 里程计数据CSV格式（来自 /Odometry 话题）
```
timestamp,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w,linear_velocity_x,linear_velocity_y,linear_velocity_z,angular_velocity_x,angular_velocity_y,angular_velocity_z
1704547200.123,1.0,2.0,0.0,0.0,0.0,0.0,1.0,0.1,0.0,0.0,0.0,0.0,0.0
1704547200.223,1.1,2.1,0.0,0.0,0.0,0.01,0.99,0.1,0.02,0.0,0.0,0.0,0.01
...
```

### 全局定位数据CSV格式（来自 /map_to_odom 话题）
```
timestamp,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w
1704547200.123,1.01,2.02,0.0,0.0,0.0,0.01,0.99
1704547200.223,1.11,2.12,0.0,0.0,0.0,0.02,0.98
...
```

### 路径数据CSV格式（来自 /path 话题，PoseStamped消息）
```
timestamp,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w
1704547200.123,1.0,2.0,0.0,0.0,0.0,0.0,1.0
1704547200.223,1.1,2.1,0.0,0.0,0.0,0.0,1.0
...
```

## 使用方法

### 基本使用
```bash
cd /home/liyi/lidar_ws/src/NEXTE_Sentry_Nav/sentry_nav/scripts
python3 visualize_localization_data.py \
  --odometry /path/to/odometry_YYYYMMDD_HHMMSS.csv \
  --map_to_odom /path/to/map_to_odom_YYYYMMDD_HHMMSS.csv
```

### 完整参数说明
```bash
python3 visualize_localization_data.py \
  --odometry <里程计CSV文件> \
  --map_to_odom <全局定位CSV文件> \
  --path <路径CSV文件> \
  --output_dir <输出目录> \
  --prefix <输出文件前缀> \
  --show_plots
```

参数说明:
- `--odometry`: 里程计数据CSV文件路径（可选）
- `--map_to_odom`: 全局定位数据CSV文件路径（可选）
- `--path`: 路径数据CSV文件路径（可选）
- `--output_dir`: 输出目录路径，默认为`./localization_visualization`
- `--prefix`: 输出文件前缀，默认为`localization`
- `--show_plots`: 添加此参数将在生成图表后显示它们

### 示例
```bash
# 基本示例（只使用里程计数据）
python3 visualize_localization_data.py \
  --odometry /home/liyi/lidar_ws_logs/localization_logs/odometry_20240106_150000.csv

# 使用里程计和全局定位数据进行对比分析
python3 visualize_localization_data.py \
  --odometry /home/liyi/lidar_ws_logs/localization_logs/odometry_20240106_150000.csv \
  --map_to_odom /home/liyi/lidar_ws_logs/localization_logs/map_to_odom_20240106_150000.csv

python3 visualize_localization_data.py --odometry /home/ubuntu/lidar_ws/localization_logs/odometry_20260106_154310.csv --map_to_odom /home/ubuntu/lidar_ws/localization_logs/map_to_odom_20260106_154310.csv --output_dir /home/ubuntu/lidar_ws/localization_logs/ --show_plots

# 自定义输出目录和前缀
python3 visualize_localization_data.py \
  --odometry /home/liyi/lidar_ws_logs/localization_logs/odometry_20240106_150000.csv \
  --map_to_odom /home/liyi/lidar_ws_logs/localization_logs/map_to_odom_20240106_150000.csv \
  --output_dir /home/liyi/lidar_ws/visualization_results \
  --prefix localization_test_1

# 显示生成的图表
python3 visualize_localization_data.py \
  --odometry /home/liyi/lidar_ws_logs/localization_logs/odometry_20240106_150000.csv \
  --show_plots
```

## 输出文件
工具将生成以下文件:
- `[前缀]_2d_trajectory.png`: 2D轨迹图
- `[前缀]_3d_trajectory.png`: 3D轨迹图（如果有Z坐标数据）
- `[前缀]_comparison.png`: 定位对比图（如果同时有里程计和全局定位数据）
- `[前缀]_metrics.png`: 评估指标表格
- `[前缀]_animation.gif`: 轨迹动画（如果数据量适中）
- `[前缀]_report.html`: 综合HTML报告

## 与定位数据记录器集成

### 1. 首先运行定位并记录数据
```bash
# 启动定位系统（确保已启动ROS）
roslaunch fast_lio_localization sentry_localize.launch

# 或者单独启动定位数据记录器
roslaunch sentry_nav launch_localization_recorder.launch
```

### 2. 定位完成后，数据将自动保存到:
- 默认目录: `~/lidar_ws_logs/localization_logs/`
- 文件命名格式: `odometry_YYYYMMDD_HHMMSS.csv`, `map_to_odom_YYYYMMDD_HHMMSS.csv`

### 3. 可视化分析
```bash
# 找到最新的数据文件
cd ~/lidar_ws_logs/localization_logs
ls -lt *.csv

# 使用最新的文件进行可视化
python3 /home/liyi/lidar_ws/src/NEXTE_Sentry_Nav/sentry_nav/scripts/visualize_localization_data.py \
  --odometry odometry_20240106_150000.csv \
  --map_to_odom map_to_odom_20240106_150000.csv \
  --prefix latest_localization
```

## 评估指标说明
工具将自动计算以下指标:

### 通用指标
- `avg_time_step`: 平均时间步长（秒）
- `sampling_freq`: 采样频率（Hz）
- `total_distance`: 总移动距离（米）
- `avg_speed`: 平均速度（米/秒）
- `max_displacement_step`: 最大单步位移（米）

### 速度相关指标（如果数据中包含速度信息）
- `avg_linear_speed`: 平均线速度（米/秒）
- `max_linear_speed`: 最大线速度（米/秒）
- `std_linear_speed`: 线速度标准差（米/秒）

### 定位对比指标（如果同时有里程计和全局定位数据）
- `X位置偏差`: 全局定位与里程计在X方向的偏差
- `Y位置偏差`: 全局定位与里程计在Y方向的偏差
- `总位置偏差`: 全局定位与里程计的欧氏距离偏差
- `平均偏差`: 平均位置偏差
- `最大偏差`: 最大位置偏差

## 故障排除

### 常见问题
1. **缺少Python库**
   ```
   ModuleNotFoundError: No module named 'pandas'
   ```
   解决方案: `pip install pandas matplotlib numpy scipy`

2. **文件不存在**
   ```
   错误: 文件不存在: /path/to/file.csv
   ```
   解决方案: 检查文件路径是否正确

3. **缺少时间戳数据**
   ```
   警告: 缺少 'timestamp' 列
   ```
   解决方案: 确保CSV文件包含timestamp列，或使用其他时间数据

4. **动画生成失败**
   ```
   保存动画时出错: ...
   ```
   解决方案: 安装pillow库 `pip install pillow`

5. **无法计算欧拉角**
   ```
   计算欧拉角时出错: ...
   ```
   解决方案: 检查四元数数据是否完整（orientation_x, orientation_y, orientation_z, orientation_w）

### 调试模式
如需更详细的输出，可以修改脚本中的print语句或添加日志记录。

## 高级使用

### 批量处理
可以编写脚本批量处理多个定位任务的数据:
```python
#!/usr/bin/env python3
import os
import subprocess
import glob

# 查找所有数据文件对
data_dir = "/home/liyi/lidar_ws_logs/localization_logs"
odom_files = glob.glob(os.path.join(data_dir, "odometry_*.csv"))

for odom_file in odom_files:
    # 提取时间戳
    timestamp = os.path.basename(odom_file).replace("odometry_", "").replace(".csv", "")
    map_to_odom_file = os.path.join(data_dir, f"map_to_odom_{timestamp}.csv")
    
    if os.path.exists(map_to_odom_file):
        # 运行可视化工具
        cmd = [
            "python3", "visualize_localization_data.py",
            "--odometry", odom_file,
            "--map_to_odom", map_to_odom_file,
            "--output_dir", "/home/liyi/lidar_ws/visualization_results",
            "--prefix", f"loc_{timestamp}"
        ]
        subprocess.run(cmd)
```

### 自定义图表样式
可以修改`visualize_localization_data.py`中的绘图函数来自定义图表样式、颜色、标签等。

## 与导航数据可视化工具的比较

| 功能 | 定位数据可视化 | 导航数据可视化 |
|------|---------------|---------------|
| 主要数据类型 | 里程计、全局定位 | 规划路径、实际路径 |
| 可视化重点 | 定位轨迹、姿态、速度 | 路径跟踪、偏差分析 |
| 主要指标 | 移动距离、速度、姿态角 | 路径长度、跟踪偏差 |
| 适用场景 | 定位性能评估、SLAM调试 | 导航性能评估、路径规划调试 |

## 联系与支持
如有问题或建议，请联系项目维护者。

---
*最后更新: 2026年1月7日*
