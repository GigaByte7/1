#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从rosbag提取并分析导航数据
支持规划路径和实际路径的对比分析
"""

import os
import sys
import argparse
import rosbag
import pandas as pd
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from tf2_msgs.msg import TFMessage
import tf.transformations as tf_trans
import matplotlib.pyplot as plt
from datetime import datetime
from scipy import signal
from scipy.interpolate import interp1d

# 设置中文字体支持
def setup_chinese_font():
    """设置matplotlib中文字体支持"""
    try:
        # 设置默认参数
        plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号
        
        # 检查字体是否存在
        import matplotlib.font_manager as fm
        
        # 获取所有可用字体
        available_fonts = [f.name for f in fm.fontManager.ttflist]
        print(f"系统中可用字体数量: {len(available_fonts)}")
        
        # 查找中文字体
        chinese_fonts = []
        for font_name in available_fonts:
            font_lower = font_name.lower()
            # 查找包含中文字体关键词的字体
            if any(keyword in font_lower for keyword in ['noto', 'cjk', 'chinese', 'simhei', 'yahei', 'ukai', 'uming']):
                chinese_fonts.append(font_name)
        
        print(f"找到 {len(chinese_fonts)} 个中文字体候选")
        
        # 优先选择顺序
        font_priority = [
            'Noto Sans CJK SC',    # 简体中文
            'Noto Sans CJK TC',    # 繁体中文
            'Noto Sans CJK JP',    # 日文（系统中实际有的）
            'Noto Serif CJK SC',   # 简体中文衬线
            'Noto Serif CJK TC',   # 繁体中文衬线
            'Noto Serif CJK JP',   # 日文衬线（系统中实际有的）
            'AR PL UMing CN',      # 中易宋体
            'AR PL UKai CN',       # 中易楷体
            'Microsoft YaHei',     # 微软雅黑
            'SimHei',              # 黑体
            'Arial Unicode MS'     # Arial Unicode
        ]
        
        selected_font = None
        for preferred_font in font_priority:
            for available_font in chinese_fonts:
                if preferred_font.lower() in available_font.lower():
                    selected_font = available_font
                    print(f"选择中文字体: {selected_font} (匹配优先级: {preferred_font})")
                    break
            if selected_font:
                break
        
        if not selected_font and chinese_fonts:
            # 如果没有精确匹配，使用第一个找到的中文字体
            selected_font = chinese_fonts[0]
            print(f"使用第一个找到的中文字体: {selected_font}")
        
        if selected_font:
            # 设置字体 - 确保字体名称正确
            plt.rcParams['font.sans-serif'] = [selected_font, 'DejaVu Sans', 'Arial', 'sans-serif']
            print(f"已设置中文字体: {selected_font}")
        else:
            # 如果没有找到中文字体，使用默认字体并显示警告
            plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'sans-serif']
            print("警告: 未找到中文字体，图表中的中文可能显示为方框")
            print("建议安装中文字体: sudo apt-get install fonts-noto-cjk")
        
        # 设置图形默认参数
        plt.rcParams['figure.autolayout'] = True
        plt.rcParams['figure.titlesize'] = 16
        plt.rcParams['axes.titlesize'] = 14
        plt.rcParams['axes.labelsize'] = 12
        plt.rcParams['xtick.labelsize'] = 10
        plt.rcParams['ytick.labelsize'] = 10
        plt.rcParams['legend.fontsize'] = 10
        
        # 立即应用设置
        plt.rcParams.update(plt.rcParams)
        
    except Exception as e:
        print(f"设置中文字体时出错: {e}")
        import traceback
        traceback.print_exc()
        print("图表中的中文可能无法正常显示")
        # 设置回退字体
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'sans-serif']
        plt.rcParams['axes.unicode_minus'] = False

# 初始化中文字体
setup_chinese_font()

class RosbagNavigationAnalyzer:
    def __init__(self, bag_file, output_dir):
        """初始化分析器"""
        self.bag_file = bag_file
        self.output_dir = output_dir
        
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        
        # 数据存储
        self.data = {
            'global_plan': [],
            'local_plan': [],
            'odometry': [],
            'localization': [],
            'cmd_vel': [],
            'goals': [],
            'tf_data': []
        }
        
    def extract_data(self):
        """从rosbag提取数据"""
        print(f"正在从 {self.bag_file} 提取数据...")
        
        try:
            bag = rosbag.Bag(self.bag_file)
        except Exception as e:
            print(f"无法打开rosbag文件: {e}")
            return False
        
        # 打印所有话题供调试
        print("Bag文件中的话题:")
        for topic, topic_info in bag.get_type_and_topic_info()[1].items():
            print(f"  {topic} (类型: {topic_info.msg_type}, 消息数量: {topic_info.message_count})")
        
        # 定义要提取的话题
        topics_to_extract = {
            # 规划路径相关 - 扩展更多可能的话题
            '/move_base1/DWAPlannerROS/global_plan': self.extract_global_plan,
            '/move_base1/DWAPlannerROS/local_plan': self.extract_local_plan,
            '/move_base1/NavfnROS/plan': self.extract_global_plan,
            '/move_base/GlobalPlanner/plan': self.extract_global_plan,
            '/move_base/NavfnROS/plan': self.extract_global_plan,
            '/move_base/DWAPlannerROS/global_plan': self.extract_global_plan,
            '/move_base/DWAPlannerROS/local_plan': self.extract_local_plan,
            '/move_base/TebLocalPlannerROS/global_plan': self.extract_global_plan,
            '/move_base/TebLocalPlannerROS/local_plan': self.extract_local_plan,
            '/global_plan': self.extract_global_plan,
            '/local_plan': self.extract_local_plan,
            '/plan': self.extract_global_plan,
            '/move_base/global_plan': self.extract_global_plan,
            '/move_base/local_plan': self.extract_local_plan,
            '/move_base/plan': self.extract_global_plan,
            '/planner/global_plan': self.extract_global_plan,
            '/planner/local_plan': self.extract_local_plan,
            '/navigation/global_plan': self.extract_global_plan,
            '/navigation/local_plan': self.extract_local_plan,
            
            # 实际路径相关
            '/localization': self.extract_localization,
            '/Odometry': self.extract_odometry,
            '/odom': self.extract_odometry,
            '/odometry/filtered': self.extract_odometry,
            '/odometry/local': self.extract_odometry,
            '/odometry/global': self.extract_odometry,
            '/robot_pose': self.extract_localization,
            '/amcl_pose': self.extract_localization,
            '/pose': self.extract_localization,
            
            # 控制命令
            '/cmd_vel': self.extract_cmd_vel,
            '/smooth_cmd_vel': self.extract_cmd_vel,
            '/mobile_base/commands/velocity': self.extract_cmd_vel,
            '/twist_mux/cmd_vel': self.extract_cmd_vel,
            '/cmd_vel_smooth': self.extract_cmd_vel,
            
            # 目标点
            '/move_base1/current_goal': self.extract_goal,
            '/move_base_simple/goal': self.extract_goal,
            '/move_base/goal': self.extract_goal,
            '/initialpose': self.extract_initial_pose,
            '/goal': self.extract_goal,
            '/navigation/goal': self.extract_goal,
            
            # 坐标变换
            '/tf': self.extract_tf,
            '/tf_static': self.extract_tf,
            
            # path_navigation Action话题
            '/track_points/goal': self.extract_action_goal,
            '/track_points/feedback': self.extract_action_feedback,
            '/track_points/result': self.extract_action_result,
            
            # 动态目标点直线控制器话题
            '/simple_line_goal': self.extract_simple_goal,
            '/simple_line_path': self.extract_line_path,
            '/actual_path_marker': self.extract_actual_path_marker
        }
        
        # 遍历rosbag中的消息
        total_msgs = bag.get_message_count()
        print(f"rosbag中共有 {total_msgs} 条消息")
        
        extracted_count = 0
        plan_msg_count = 0
        for topic, msg, t in bag.read_messages():
            if topic in topics_to_extract:
                try:
                    # 特别记录规划路径话题的处理
                    if 'plan' in topic.lower():
                        plan_msg_count += 1
                        if plan_msg_count <= 5:  # 只打印前5条规划路径消息的调试信息
                            print(f"DEBUG: 处理规划路径话题 {topic}, 时间戳: {t}, 消息类型: {type(msg)}")
                            if hasattr(msg, 'poses'):
                                print(f"DEBUG: 路径点数: {len(msg.poses)}")
                    
                    topics_to_extract[topic](msg, t, topic)
                    extracted_count += 1
                except Exception as e:
                    print(f"处理话题 {topic} 时出错: {e}")
                    import traceback
                    traceback.print_exc()
            
            # 进度显示
            if extracted_count % 100 == 0:
                print(f"已处理 {extracted_count} 条消息...")
        
        print(f"总共处理了 {plan_msg_count} 条规划路径消息")
        
        bag.close()
        print(f"数据提取完成，共提取 {extracted_count} 条消息")
        return True
    
    def extract_global_plan(self, msg, timestamp, topic):
        """提取全局规划路径"""
        print(f"DEBUG: 提取全局规划路径，话题: {topic}, 时间戳: {timestamp}, 消息类型: {type(msg)}")
        
        # 更灵活的类型检查
        try:
            # 检查是否有'poses'属性
            if hasattr(msg, 'poses') and hasattr(msg, 'header'):
                print(f"DEBUG: 消息具有poses属性，长度: {len(msg.poses)}")
                path_id = msg.header.seq if hasattr(msg.header, 'seq') else 0
                for i, pose_stamped in enumerate(msg.poses):
                    if hasattr(pose_stamped, 'pose'):
                        pose = pose_stamped.pose
                        self.data['global_plan'].append({
                            'timestamp': timestamp.to_sec(),
                            'path_id': path_id,
                            'point_index': i,
                            'x': pose.position.x,
                            'y': pose.position.y,
                            'z': pose.position.z,
                            'qx': pose.orientation.x,
                            'qy': pose.orientation.y,
                            'qz': pose.orientation.z,
                            'qw': pose.orientation.w,
                            'topic': topic
                        })
                print(f"DEBUG: 成功提取了 {len(msg.poses)} 个路径点")
            else:
                print(f"DEBUG: 消息没有poses属性，实际属性: {dir(msg)}")
        except Exception as e:
            print(f"DEBUG: 提取全局规划路径时出错: {e}")
            import traceback
            traceback.print_exc()
    
    def extract_local_plan(self, msg, timestamp, topic):
        """提取局部规划路径"""
        print(f"DEBUG: 提取局部规划路径，话题: {topic}, 时间戳: {timestamp}, 消息类型: {type(msg)}")
        
        # 更灵活的类型检查
        try:
            # 检查是否有'poses'属性
            if hasattr(msg, 'poses') and hasattr(msg, 'header'):
                print(f"DEBUG: 消息具有poses属性，长度: {len(msg.poses)}")
                path_id = msg.header.seq if hasattr(msg.header, 'seq') else 0
                for i, pose_stamped in enumerate(msg.poses):
                    if hasattr(pose_stamped, 'pose'):
                        pose = pose_stamped.pose
                        self.data['local_plan'].append({
                            'timestamp': timestamp.to_sec(),
                            'path_id': path_id,
                            'point_index': i,
                            'x': pose.position.x,
                            'y': pose.position.y,
                            'z': pose.position.z,
                            'qx': pose.orientation.x,
                            'qy': pose.orientation.y,
                            'qz': pose.orientation.z,
                            'qw': pose.orientation.w,
                            'topic': topic
                        })
                print(f"DEBUG: 成功提取了 {len(msg.poses)} 个路径点")
            else:
                print(f"DEBUG: 消息没有poses属性，实际属性: {dir(msg)}")
        except Exception as e:
            print(f"DEBUG: 提取局部规划路径时出错: {e}")
            import traceback
            traceback.print_exc()
    
    def extract_localization(self, msg, timestamp, topic):
        """提取定位数据"""
        if hasattr(msg, 'pose'):
            pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
            self.data['localization'].append({
                'timestamp': timestamp.to_sec(),
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'topic': topic
            })
    
    def extract_odometry(self, msg, timestamp, topic):
        """提取里程计数据"""
        if hasattr(msg, 'pose') and hasattr(msg, 'twist'):
            pose = msg.pose.pose
            twist = msg.twist.twist
            self.data['odometry'].append({
                'timestamp': timestamp.to_sec(),
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'vx': twist.linear.x,
                'vy': twist.linear.y,
                'vz': twist.linear.z,
                'wx': twist.angular.x,
                'wy': twist.angular.y,
                'wz': twist.angular.z,
                'topic': topic
            })
    
    def extract_cmd_vel(self, msg, timestamp, topic):
        """提取速度命令"""
        if hasattr(msg, 'linear') and hasattr(msg, 'angular'):
            self.data['cmd_vel'].append({
                'timestamp': timestamp.to_sec(),
                'linear_x': msg.linear.x,
                'linear_y': msg.linear.y,
                'linear_z': msg.linear.z,
                'angular_x': msg.angular.x,
                'angular_y': msg.angular.y,
                'angular_z': msg.angular.z,
                'topic': topic
            })
    
    def extract_goal(self, msg, timestamp, topic):
        """提取目标点"""
        if isinstance(msg, PoseStamped):
            pose = msg.pose
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'topic': topic,
                'goal_type': 'current_goal' if 'current_goal' in topic else 'simple_goal'
            })
    
    def extract_initial_pose(self, msg, timestamp, topic):
        """提取初始位姿"""
        if isinstance(msg, PoseWithCovarianceStamped):
            pose = msg.pose.pose
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'topic': topic,
                'goal_type': 'initial_pose'
            })
    
    def extract_action_goal(self, msg, timestamp, topic):
        """提取Action目标"""
        # 处理path_navigation的Action目标
        if hasattr(msg, 'goal') and hasattr(msg.goal, 'path_group_name'):
            # 这是PathNavigationActionGoal消息
            goal = msg.goal
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'path_group_name': goal.path_group_name,
                'dead_zone_radius': goal.dead_zone_radius,
                'topic': topic,
                'goal_type': 'action_goal'
            })
    
    def extract_action_feedback(self, msg, timestamp, topic):
        """提取Action反馈"""
        # 处理path_navigation的Action反馈
        if hasattr(msg, 'feedback') and hasattr(msg.feedback, 'current_waypoint_info'):
            # 这是PathNavigationActionFeedback消息
            feedback = msg.feedback
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'current_waypoint_info': feedback.current_waypoint_info,
                'topic': topic,
                'goal_type': 'action_feedback'
            })
    
    def extract_action_result(self, msg, timestamp, topic):
        """提取Action结果"""
        # 处理path_navigation的Action结果
        if hasattr(msg, 'result') and hasattr(msg.result, 'success'):
            # 这是PathNavigationActionResult消息
            result = msg.result
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'success': result.success,
                'message': result.message,
                'topic': topic,
                'goal_type': 'action_result'
            })
    
    def extract_simple_goal(self, msg, timestamp, topic):
        """提取simple_line_goal"""
        if isinstance(msg, PoseStamped):
            pose = msg.pose
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'topic': topic,
                'goal_type': 'simple_line_goal'
            })
    
    def extract_line_path(self, msg, timestamp, topic):
        """提取simple_line_path"""
        if hasattr(msg, 'points') and len(msg.points) >= 2:
            # 这是起点到终点的连线
            start_point = msg.points[0]
            end_point = msg.points[-1]
            
            # 将起点作为规划路径点
            self.data['global_plan'].append({
                'timestamp': timestamp.to_sec(),
                'path_id': 0,  # 使用固定ID表示这是直线路径
                'point_index': 0,
                'x': start_point.x,
                'y': start_point.y,
                'z': start_point.z,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.0,
                'qw': 1.0,
                'topic': topic
            })
            
            # 将终点作为规划路径点
            self.data['global_plan'].append({
                'timestamp': timestamp.to_sec(),
                'path_id': 0,  # 使用固定ID表示这是直线路径
                'point_index': 1,
                'x': end_point.x,
                'y': end_point.y,
                'z': end_point.z,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.0,
                'qw': 1.0,
                'topic': topic
            })
    
    def extract_actual_path_marker(self, msg, timestamp, topic):
        """提取actual_path_marker"""
        if hasattr(msg, 'points') and len(msg.points) > 0:
            # 这是实际路径的轨迹
            for i, point in enumerate(msg.points):
                self.data['odometry'].append({
                    'timestamp': timestamp.to_sec(),
                    'x': point.x,
                    'y': point.y,
                    'z': point.z,
                    'vx': 0.0,  # 路径点没有速度信息
                    'vy': 0.0,
                    'vz': 0.0,
                    'wx': 0.0,
                    'wy': 0.0,
                    'wz': 0.0,
                    'qx': 0.0,
                    'qy': 0.0,
                    'qz': 0.0,
                    'qw': 1.0,
                    'topic': topic
                })
    
    def extract_tf(self, msg, timestamp, topic):
        """提取坐标变换数据"""
        if isinstance(msg, TFMessage):
            for transform in msg.transforms:
                self.data['tf_data'].append({
                    'timestamp': timestamp.to_sec(),
                    'parent_frame': transform.header.frame_id,
                    'child_frame': transform.child_frame_id,
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z,
                    'qx': transform.transform.rotation.x,
                    'qy': transform.transform.rotation.y,
                    'qz': transform.transform.rotation.z,
                    'qw': transform.transform.rotation.w,
                    'topic': topic
                })
    
    def save_to_csv(self):
        """将提取的数据保存为CSV文件"""
        print("正在保存数据到CSV文件...")
        
        # 先打印调试信息
        print("DEBUG: 数据统计:")
        for data_type, records in self.data.items():
            print(f"  {data_type}: {len(records)} 条记录")
        
        base_name = os.path.splitext(os.path.basename(self.bag_file))[0]
        
        for data_type, records in self.data.items():
            if records:
                try:
                    df = pd.DataFrame(records)
                    csv_file = os.path.join(self.output_dir, f"{base_name}_{data_type}.csv")
                    df.to_csv(csv_file, index=False)
                    print(f"  {data_type}: {len(records)} 条记录 -> {csv_file}")
                except Exception as e:
                    print(f"  保存{data_type}时出错: {e}")
                    import traceback
                    traceback.print_exc()
            else:
                print(f"  {data_type}: 0 条记录 (跳过)")
        
        print("数据保存完成")
        
        # 返回主要的数据文件路径
        result_files = {}
        for data_type in ['global_plan', 'localization', 'odometry']:
            csv_file = os.path.join(self.output_dir, f"{base_name}_{data_type}.csv")
            if os.path.exists(csv_file):
                result_files[data_type] = csv_file
        
        return result_files
    
    def apply_filters(self, df, columns=['x', 'y'], filter_type='savgol', window_length=5, polyorder=3):
        """对数据应用滤波器"""
        if len(df) < window_length:
            print(f"警告: 数据点数量 ({len(df)}) 小于滤波器窗口长度 ({window_length})，跳过滤波")
            return df
        
        filtered_df = df.copy()
        
        for col in columns:
            if col in df.columns:
                try:
                    if filter_type == 'savgol':
                        # Savitzky-Golay滤波器 - 适合平滑路径数据
                        filtered_values = signal.savgol_filter(df[col], window_length=window_length, polyorder=polyorder)
                    elif filter_type == 'butter':
                        # Butterworth低通滤波器
                        nyquist = 0.5 * (len(df) / (df['timestamp'].max() - df['timestamp'].min()))
                        cutoff = 0.1 * nyquist  # 截止频率
                        b, a = signal.butter(4, cutoff / nyquist, btype='low')
                        filtered_values = signal.filtfilt(b, a, df[col])
                    elif filter_type == 'moving_avg':
                        # 移动平均滤波器
                        filtered_values = df[col].rolling(window=window_length, center=True).mean()
                        filtered_values = filtered_values.fillna(df[col])  # 填充NaN值
                    else:
                        filtered_values = df[col]
                    
                    filtered_df[col + '_filtered'] = filtered_values
                except Exception as e:
                    print(f"滤波器应用失败 ({col}): {e}")
                    filtered_df[col + '_filtered'] = df[col]
        
        return filtered_df
    
    def interpolate_data(self, df, target_freq=10.0):
        """对数据进行插值，统一时间频率"""
        if 'timestamp' not in df.columns or len(df) < 2:
            return df
        
        # 按时间排序
        df = df.sort_values('timestamp').reset_index(drop=True)
        
        # 计算时间范围
        t_min = df['timestamp'].min()
        t_max = df['timestamp'].max()
        
        # 创建目标时间序列
        target_times = np.arange(t_min, t_max, 1.0/target_freq)
        
        # 插值的列
        cols_to_interpolate = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz']
        interpolated_data = {'timestamp': target_times}
        
        for col in cols_to_interpolate:
            if col in df.columns:
                # 创建插值函数
                f = interp1d(df['timestamp'], df[col], kind='linear', 
                           bounds_error=False, fill_value='extrapolate')
                # 应用插值
                interpolated_data[col] = f(target_times)
        
        return pd.DataFrame(interpolated_data)
    
    def analyze_path_comparison(self, planned_csv, actual_csv):
        """分析规划路径与实际路径的对比"""
        print("正在分析路径对比...")
        
        try:
            planned_df = pd.read_csv(planned_csv)
            actual_df = pd.read_csv(actual_csv)
        except Exception as e:
            print(f"读取CSV文件失败: {e}")
            return None
        
        # 数据预处理：滤波和插值
        print("正在对数据进行滤波和插值处理...")
        
        # 对实际路径数据应用Savitzky-Golay滤波器
        filtered_actual_df = self.apply_filters(actual_df, filter_type='savgol', window_length=11, polyorder=3)
        
        # 对规划路径数据应用移动平均滤波器（如果数据点较多）
        if len(planned_df) > 20:
            filtered_planned_df = self.apply_filters(planned_df, filter_type='moving_avg', window_length=5)
        else:
            filtered_planned_df = planned_df.copy()
        
        # 插值到统一时间频率（10Hz）
        interpolated_actual_df = self.interpolate_data(filtered_actual_df, target_freq=10.0)
        interpolated_planned_df = self.interpolate_data(filtered_planned_df, target_freq=10.0)
        
        # 计算基本统计信息
        stats = {
            'planned_points': len(planned_df),
            'actual_points': len(actual_df),
            'filtered_planned_points': len(filtered_planned_df),
            'filtered_actual_points': len(filtered_actual_df),
            'interpolated_planned_points': len(interpolated_planned_df),
            'interpolated_actual_points': len(interpolated_actual_df),
            'planned_start_time': planned_df['timestamp'].min() if 'timestamp' in planned_df.columns else None,
            'planned_end_time': planned_df['timestamp'].max() if 'timestamp' in planned_df.columns else None,
            'actual_start_time': actual_df['timestamp'].min() if 'timestamp' in actual_df.columns else None,
            'actual_end_time': actual_df['timestamp'].max() if 'timestamp' in actual_df.columns else None
        }
        
        # 计算路径长度（使用滤波后的数据）
        def calculate_path_length(df, use_filtered=True):
            if len(df) < 2:
                return 0
            
            # 按时间排序
            if 'timestamp' in df.columns:
                df = df.sort_values('timestamp')
            
            # 选择使用原始数据还是滤波数据
            x_col = 'x_filtered' if use_filtered and 'x_filtered' in df.columns else 'x'
            y_col = 'y_filtered' if use_filtered and 'y_filtered' in df.columns else 'y'
            
            length = 0
            for i in range(1, len(df)):
                dx = df[x_col].iloc[i] - df[x_col].iloc[i-1]
                dy = df[y_col].iloc[i] - df[y_col].iloc[i-1]
                length += np.sqrt(dx*dx + dy*dy)
            
            return length
        
        stats['planned_length'] = calculate_path_length(planned_df, use_filtered=False)
        stats['actual_length'] = calculate_path_length(actual_df, use_filtered=False)
        stats['filtered_planned_length'] = calculate_path_length(filtered_planned_df, use_filtered=True)
        stats['filtered_actual_length'] = calculate_path_length(filtered_actual_df, use_filtered=True)
        
        # 计算路径偏差（只计算实际路径与规划路径共同的部分）
        if len(filtered_planned_df) > 0 and len(filtered_actual_df) > 0:
            # 使用滤波后的数据计算偏差
            x_plan = 'x_filtered' if 'x_filtered' in filtered_planned_df.columns else 'x'
            y_plan = 'y_filtered' if 'y_filtered' in filtered_planned_df.columns else 'y'
            x_actual = 'x_filtered' if 'x_filtered' in filtered_actual_df.columns else 'x'
            y_actual = 'y_filtered' if 'y_filtered' in filtered_actual_df.columns else 'y'
            
            deviations = []
            
            # 坐标系和时间同步检查
            print("检查坐标系和时间同步...")
            
            # 1. 时间同步检查
            if 'timestamp' in filtered_actual_df.columns and 'timestamp' in filtered_planned_df.columns:
                actual_times = filtered_actual_df['timestamp'].values
                planned_times = filtered_planned_df['timestamp'].values
                
                # 计算时间重叠范围
                actual_start = actual_times.min()
                actual_end = actual_times.max()
                planned_start = planned_times.min()
                planned_end = planned_times.max()
                
                print(f"实际路径时间范围: {actual_start:.3f} - {actual_end:.3f}")
                print(f"规划路径时间范围: {planned_start:.3f} - {planned_end:.3f}")
                
                # 检查时间偏移
                time_offset = abs(actual_start - planned_start)
                print(f"时间偏移: {time_offset:.3f}s")
                
                if time_offset > 10.0:  # 如果时间偏移超过10秒，可能存在时间同步问题
                    print(f"警告: 时间偏移较大 ({time_offset:.3f}s)，可能存在时间同步问题")
                
                # 2. 坐标系检查 - 检查起点位置差异
                if len(filtered_actual_df) > 0 and len(filtered_planned_df) > 0:
                    actual_start_pos = (filtered_actual_df[x_actual].iloc[0], filtered_actual_df[y_actual].iloc[0])
                    planned_start_pos = (filtered_planned_df[x_plan].iloc[0], filtered_planned_df[y_plan].iloc[0])
                    
                    start_dist = np.sqrt((actual_start_pos[0] - planned_start_pos[0])**2 + 
                                       (actual_start_pos[1] - planned_start_pos[1])**2)
                    print(f"起点位置差异: {start_dist:.3f} m")
                    
                    if start_dist > 5.0:  # 如果起点差异超过5米，可能存在坐标系问题
                        print(f"警告: 起点位置差异较大 ({start_dist:.3f}m)，可能存在坐标系不一致问题")
            
            # 方法1：时间对齐的偏差计算（只计算共同部分）
            if 'timestamp' in filtered_actual_df.columns and 'timestamp' in filtered_planned_df.columns:
                print("使用优化的时间对齐方法计算路径偏差（只计算共同部分）...")
                
                # 时间重叠范围内的实际路径点
                actual_times = filtered_actual_df['timestamp'].values
                planned_times = filtered_planned_df['timestamp'].values
                
                # 找到时间重叠范围
                overlap_start = max(actual_times.min(), planned_times.min())
                overlap_end = min(actual_times.max(), planned_times.max())
                
                if overlap_start < overlap_end:
                    print(f"时间重叠范围: {overlap_start:.3f} - {overlap_end:.3f}")
                    
                    # 只在时间重叠范围内计算偏差
                    overlap_mask = (actual_times >= overlap_start) & (actual_times <= overlap_end)
                    overlap_actual_df = filtered_actual_df[overlap_mask].copy()
                    
                    print(f"重叠时间范围内的实际路径点数: {len(overlap_actual_df)}")
                    
                    for _, actual_point in overlap_actual_df.iterrows():
                        actual_time = actual_point['timestamp']
                        
                        # 在规划路径中找到最接近的时间点
                        time_diffs = np.abs(filtered_planned_df['timestamp'] - actual_time)
                        closest_idx = time_diffs.idxmin()
                        
                        # 检查时间差是否过大
                        time_diff = time_diffs.min()
                        if time_diff > 2.0:  # 如果时间差超过2秒，可能有问题
                            print(f"警告: 时间差过大 {time_diff:.3f}s，可能时间戳不同步")
                            # 尝试使用最近的空间距离匹配
                            x_actual_val = actual_point[x_actual]
                            y_actual_val = actual_point[y_actual]
                            
                            # 计算所有规划路径点到当前实际点的距离
                            distances = np.sqrt(
                                (filtered_planned_df[x_plan] - x_actual_val)**2 + 
                                (filtered_planned_df[y_plan] - y_actual_val)**2
                            )
                            min_dist_idx = distances.idxmin()
                            plan_point = filtered_planned_df.loc[min_dist_idx]
                            dist = distances.min()
                        else:
                            # 时间对齐正常，使用时间匹配
                            plan_point = filtered_planned_df.loc[closest_idx]
                            dist = np.sqrt(
                                (plan_point[x_plan] - actual_point[x_actual])**2 + 
                                (plan_point[y_plan] - actual_point[y_actual])**2
                            )
                        
                        deviations.append(dist)
                else:
                    print("警告: 时间范围没有重叠，无法计算共同部分的偏差")
                    # 如果时间范围没有重叠，无法计算有意义的偏差
                    deviations = []
            else:
                # 方法2：最近点匹配（备用方法，但不推荐用于计算平均偏差）
                print("使用最近点匹配方法计算路径偏差（注意：这可能包含非共同部分）...")
                
                # 为了提高效率，对实际路径进行下采样
                if len(filtered_actual_df) > 1000:
                    # 如果实际路径点太多，进行下采样
                    sample_rate = len(filtered_actual_df) // 1000
                    actual_points = filtered_actual_df.iloc[::sample_rate]
                    print(f"实际路径点过多，已下采样到 {len(actual_points)} 个点")
                else:
                    actual_points = filtered_actual_df
                
                for _, actual_point in actual_points.iterrows():
                    distances = np.sqrt(
                        (filtered_planned_df[x_plan] - actual_point[x_actual])**2 + 
                        (filtered_planned_df[y_plan] - actual_point[y_actual])**2
                    )
                    min_dist = distances.min()
                    deviations.append(min_dist)
            
            if deviations:
                # 过滤异常大的偏差值（可能是匹配错误）
                deviations = np.array(deviations)
                
                # 方法1：基于路径长度的动态阈值（更严格的阈值）
                # 如果路径长度较短，使用较小的阈值
                path_length = stats.get('filtered_actual_length', 0)
                if path_length > 0:
                    # 动态阈值：路径长度的5%或1米，取较小值（比之前更严格）
                    dynamic_threshold = min(path_length * 0.05, 1.0)
                else:
                    # 默认阈值
                    dynamic_threshold = 0.3  # 进一步降低默认阈值
                
                # 方法2：基于数据分布的统计阈值（更严格的阈值）
                median_dev = np.median(deviations)
                mad = np.median(np.abs(deviations - median_dev))
                # 异常值阈值：中位数 + 1.0 * MAD（比之前更严格）
                statistical_threshold = median_dev + 1.0 * mad
                
                # 使用更严格的阈值
                threshold = min(dynamic_threshold, statistical_threshold)
                
                print(f"动态阈值: {dynamic_threshold:.3f} m")
                print(f"统计阈值: {statistical_threshold:.3f} m")
                print(f"最终阈值: {threshold:.3f} m")
                
                # 过滤异常值
                filtered_deviations = deviations[deviations <= threshold]
                
                if len(filtered_deviations) > 0:
                    # 计算更稳健的统计指标
                    stats['avg_deviation'] = np.mean(filtered_deviations)
                    stats['max_deviation'] = np.max(filtered_deviations)
                    stats['std_deviation'] = np.std(filtered_deviations)
                    stats['median_deviation'] = np.median(filtered_deviations)
                    # 95%分位数
                    stats['p95_deviation'] = np.percentile(filtered_deviations, 95)
                    # 99%分位数
                    stats['p99_deviation'] = np.percentile(filtered_deviations, 99)
                    
                    print(f"原始偏差统计: 平均={np.mean(deviations):.3f}, 最大={np.max(deviations):.3f}")
                    print(f"过滤后偏差统计: 平均={stats['avg_deviation']:.3f}, 最大={stats['max_deviation']:.3f}")
                    print(f"过滤了 {len(deviations) - len(filtered_deviations)} 个异常值")
                    print(f"中位数偏差: {stats['median_deviation']:.3f} m")
                    print(f"95%分位数: {stats['p95_deviation']:.3f} m")
                else:
                    # 如果过滤后没有数据，使用原始数据但记录警告
                    stats['avg_deviation'] = np.mean(deviations)
                    stats['max_deviation'] = np.max(deviations)
                    stats['std_deviation'] = np.std(deviations)
                    stats['median_deviation'] = np.median(deviations)
                    print("警告: 过滤后没有有效数据，使用原始偏差值")
            else:
                print("警告: 无法计算路径偏差（可能没有共同部分）")
        
        # 计算起点和终点误差（使用滤波后的数据）
        if len(filtered_planned_df) > 0 and len(filtered_actual_df) > 0:
            x_plan = 'x_filtered' if 'x_filtered' in filtered_planned_df.columns else 'x'
            y_plan = 'y_filtered' if 'y_filtered' in filtered_planned_df.columns else 'y'
            x_actual = 'x_filtered' if 'x_filtered' in filtered_actual_df.columns else 'x'
            y_actual = 'y_filtered' if 'y_filtered' in filtered_actual_df.columns else 'y'
            
            start_error = np.sqrt(
                (filtered_planned_df[x_plan].iloc[0] - filtered_actual_df[x_actual].iloc[0])**2 +
                (filtered_planned_df[y_plan].iloc[0] - filtered_actual_df[y_actual].iloc[0])**2
            )
            end_error = np.sqrt(
                (filtered_planned_df[x_plan].iloc[-1] - filtered_actual_df[x_actual].iloc[-1])**2 +
                (filtered_planned_df[y_plan].iloc[-1] - filtered_actual_df[y_actual].iloc[-1])**2
            )
            stats['start_error'] = start_error
            stats['end_error'] = end_error
        
        # 保存滤波后的数据
        base_name = os.path.splitext(os.path.basename(self.bag_file))[0]
        filtered_actual_csv = os.path.join(self.output_dir, f"{base_name}_actual_filtered.csv")
        filtered_planned_csv = os.path.join(self.output_dir, f"{base_name}_planned_filtered.csv")
        
        filtered_actual_df.to_csv(filtered_actual_csv, index=False)
        filtered_planned_df.to_csv(filtered_planned_csv, index=False)
        
        print(f"滤波后的数据已保存到: {filtered_actual_csv} 和 {filtered_planned_csv}")
        
        return stats
    
    def generate_report(self, stats, output_file):
        """生成分析报告"""
        print("正在生成分析报告...")
        
        with open(output_file, 'w') as f:
            f.write("=" * 60 + "\n")
            f.write("导航数据分析报告\n")
            f.write("=" * 60 + "\n\n")
            
            f.write(f"分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"rosbag文件: {self.bag_file}\n")
            f.write(f"输出目录: {self.output_dir}\n\n")
            
            f.write("数据统计:\n")
            f.write("-" * 40 + "\n")
            for key, value in stats.items():
                if isinstance(value, float):
                    f.write(f"{key}: {value:.4f}\n")
                else:
                    f.write(f"{key}: {value}\n")
            
            f.write("\n路径评估指标:\n")
            f.write("-" * 40 + "\n")
            
            # 路径长度对比
            if 'planned_length' in stats and 'actual_length' in stats:
                length_ratio = stats['actual_length'] / stats['planned_length'] if stats['planned_length'] > 0 else 0
                f.write(f"规划路径长度: {stats['planned_length']:.3f} m\n")
                f.write(f"实际路径长度: {stats['actual_length']:.3f} m\n")
                f.write(f"长度比率: {length_ratio:.3f}\n")
            
            # 路径偏差（增强版）
            if 'avg_deviation' in stats:
                f.write(f"\n路径跟踪精度:\n")
                f.write(f"  平均偏差: {stats['avg_deviation']:.3f} m\n")
                f.write(f"  中位数偏差: {stats['median_deviation']:.3f} m\n")
                f.write(f"  最大偏差: {stats['max_deviation']:.3f} m\n")
                f.write(f"  95%分位数: {stats['p95_deviation']:.3f} m\n")
                f.write(f"  99%分位数: {stats['p99_deviation']:.3f} m\n")
                f.write(f"  偏差标准差: {stats['std_deviation']:.3f} m\n")
            
            # 起点终点误差
            if 'start_error' in stats and 'end_error' in stats:
                f.write(f"\n起点终点误差:\n")
                f.write(f"  起点误差: {stats['start_error']:.3f} m\n")
                f.write(f"  终点误差: {stats['end_error']:.3f} m\n")
            
            # 偏差分析说明
            f.write(f"\n偏差分析说明:\n")
            f.write(f"  - 平均偏差反映整体跟踪精度\n")
            f.write(f"  - 中位数偏差对异常值不敏感，更稳定\n")
            f.write(f"  - 95%分位数表示95%的时间偏差不超过此值\n")
            f.write(f"  - 如果偏差过大，可能原因：\n")
            f.write(f"    1. 时间戳不同步\n")
            f.write(f"    2. 坐标系不一致\n")
            f.write(f"    3. 路径匹配算法问题\n")
            f.write(f"    4. 数据质量问题\n")
            
            f.write("\n" + "=" * 60 + "\n")
            f.write("报告结束\n")
            f.write("=" * 60 + "\n")
        
        print(f"分析报告已保存到: {output_file}")
        
    def create_visualization(self, planned_csv, actual_csv):
        """创建可视化图表（包含滤波功能）"""
        print("正在创建可视化图表...")
        
        try:
            planned_df = pd.read_csv(planned_csv)
            actual_df = pd.read_csv(actual_csv)
        except Exception as e:
            print(f"读取CSV文件失败: {e}")
            return
        
        # 应用滤波器
        print("正在对数据应用滤波器...")
        filtered_actual_df = self.apply_filters(actual_df, filter_type='savgol', window_length=11, polyorder=3)
        if len(planned_df) > 20:
            filtered_planned_df = self.apply_filters(planned_df, filter_type='moving_avg', window_length=5)
        else:
            filtered_planned_df = planned_df.copy()
        
        # 计算路径长度用于偏差图的动态阈值
        def calculate_path_length(df, use_filtered=True):
            if len(df) < 2:
                return 0
            
            # 按时间排序
            if 'timestamp' in df.columns:
                df = df.sort_values('timestamp')
            
            # 选择使用原始数据还是滤波数据
            x_col = 'x_filtered' if use_filtered and 'x_filtered' in df.columns else 'x'
            y_col = 'y_filtered' if use_filtered and 'y_filtered' in df.columns else 'y'
            
            length = 0
            for i in range(1, len(df)):
                dx = df[x_col].iloc[i] - df[x_col].iloc[i-1]
                dy = df[y_col].iloc[i] - df[y_col].iloc[i-1]
                length += np.sqrt(dx*dx + dy*dy)
            
            return length
        
        # 计算实际路径长度
        path_length = calculate_path_length(filtered_actual_df, use_filtered=True)
        
        fig, axes = plt.subplots(3, 2, figsize=(18, 16))
        fig.suptitle('导航路径分析（含滤波处理）', fontsize=16)
        
        # 1. 路径对比图（原始数据）
        ax1 = axes[0, 0]
        
        if len(planned_df) > 0 and len(actual_df) > 0:
            # 找到实际路径的起始时间
            actual_start_time = actual_df['timestamp'].min() if 'timestamp' in actual_df.columns else 0
            
            # 找到规划路径中时间最接近实际起始时间的path_id
            planned_paths_by_time = planned_df.groupby('path_id')['timestamp'].min().reset_index()
            planned_paths_by_time['time_diff'] = abs(planned_paths_by_time['timestamp'] - actual_start_time)
            
            # 找到时间最接近的规划路径
            if len(planned_paths_by_time) > 0:
                closest_path = planned_paths_by_time.loc[planned_paths_by_time['time_diff'].idxmin()]
                closest_path_id = closest_path['path_id']
                closest_path_time = closest_path['timestamp']
                
                # 获取最接近的规划路径数据
                closest_plan = planned_df[planned_df['path_id'] == closest_path_id].sort_values('point_index')
                
                if len(closest_plan) > 0:
                    # 绘制规划路径
                    ax1.plot(closest_plan['x'], closest_plan['y'], 'r-', linewidth=2, alpha=0.7, label='规划路径')
                    
                    # 标记规划起点和终点
                    ax1.scatter(closest_plan['x'].iloc[0], closest_plan['y'].iloc[0], 
                               c='green', s=150, marker='o', label='规划起点', zorder=5)
                    ax1.scatter(closest_plan['x'].iloc[-1], closest_plan['y'].iloc[-1], 
                               c='red', s=150, marker='*', label='规划终点', zorder=5)
                    
                    # 记录用于后续误差计算
                    selected_planned_path = closest_plan
                else:
                    # 回退到第一个规划路径
                    unique_paths = planned_df['path_id'].unique()
                    if len(unique_paths) > 0:
                        first_path_id = sorted(unique_paths)[0]
                        selected_planned_path = planned_df[planned_df['path_id'] == first_path_id].sort_values('point_index')
                        ax1.plot(selected_planned_path['x'], selected_planned_path['y'], 'r-', linewidth=2, alpha=0.7, label='规划路径')
                        ax1.scatter(selected_planned_path['x'].iloc[0], selected_planned_path['y'].iloc[0], 
                                   c='green', s=150, marker='o', label='规划起点', zorder=5)
                        ax1.scatter(selected_planned_path['x'].iloc[-1], selected_planned_path['y'].iloc[-1], 
                                   c='red', s=150, marker='*', label='规划终点', zorder=5)
            else:
                # 如果没有时间信息，使用第一个path_id
                unique_paths = planned_df['path_id'].unique()
                if len(unique_paths) > 0:
                    first_path_id = sorted(unique_paths)[0]
                    selected_planned_path = planned_df[planned_df['path_id'] == first_path_id].sort_values('point_index')
                    ax1.plot(selected_planned_path['x'], selected_planned_path['y'], 'r-', linewidth=2, alpha=0.7, label='规划路径')
                    ax1.scatter(selected_planned_path['x'].iloc[0], selected_planned_path['y'].iloc[0], 
                               c='green', s=150, marker='o', label='规划起点', zorder=5)
                    ax1.scatter(selected_planned_path['x'].iloc[-1], selected_planned_path['y'].iloc[-1], 
                               c='red', s=150, marker='*', label='规划终点', zorder=5)
        
        elif len(planned_df) > 0:
            # 只有规划路径数据，没有实际路径数据
            unique_paths = planned_df['path_id'].unique()
            if len(unique_paths) > 0:
                first_path_id = sorted(unique_paths)[0]
                selected_planned_path = planned_df[planned_df['path_id'] == first_path_id].sort_values('point_index')
                ax1.plot(selected_planned_path['x'], selected_planned_path['y'], 'r-', linewidth=2, alpha=0.7, label='规划路径')
                ax1.scatter(selected_planned_path['x'].iloc[0], selected_planned_path['y'].iloc[0], 
                           c='green', s=150, marker='o', label='规划起点', zorder=5)
                ax1.scatter(selected_planned_path['x'].iloc[-1], selected_planned_path['y'].iloc[-1], 
                           c='red', s=150, marker='*', label='规划终点', zorder=5)
        
        if len(actual_df) > 0:
            # 绘制实际路径（原始数据）
            ax1.plot(actual_df['x'], actual_df['y'], 'b-', linewidth=2, label='实际路径（原始）', alpha=0.7)
            
            # 标记实际起点和终点
            ax1.scatter(actual_df['x'].iloc[0], actual_df['y'].iloc[0], 
                       c='green', s=150, marker='s', label='实际起点', zorder=5)
            ax1.scatter(actual_df['x'].iloc[-1], actual_df['y'].iloc[-1], 
                       c='red', s=150, marker='x', label='实际终点', zorder=5)
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('路径对比（原始数据）')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        ax1.axis('equal')
        
        # 2. 路径对比图（滤波后数据）
        ax2 = axes[0, 1]
        
        if len(filtered_planned_df) > 0 and len(filtered_actual_df) > 0:
            # 绘制滤波后的规划路径
            x_plan = 'x_filtered' if 'x_filtered' in filtered_planned_df.columns else 'x'
            y_plan = 'y_filtered' if 'y_filtered' in filtered_planned_df.columns else 'y'
            ax2.plot(filtered_planned_df[x_plan], filtered_planned_df[y_plan], 'r-', linewidth=2, alpha=0.7, label='规划路径（滤波后）')
            
            # 绘制滤波后的实际路径
            x_actual = 'x_filtered' if 'x_filtered' in filtered_actual_df.columns else 'x'
            y_actual = 'y_filtered' if 'y_filtered' in filtered_actual_df.columns else 'y'
            ax2.plot(filtered_actual_df[x_actual], filtered_actual_df[y_actual], 'b-', linewidth=2, label='实际路径（滤波后）', alpha=0.7)
            
            # 标记起点和终点
            ax2.scatter(filtered_planned_df[x_plan].iloc[0], filtered_planned_df[y_plan].iloc[0], 
                       c='green', s=150, marker='o', label='规划起点', zorder=5)
            ax2.scatter(filtered_planned_df[x_plan].iloc[-1], filtered_planned_df[y_plan].iloc[-1], 
                       c='red', s=150, marker='*', label='规划终点', zorder=5)
            ax2.scatter(filtered_actual_df[x_actual].iloc[0], filtered_actual_df[y_actual].iloc[0], 
                       c='green', s=150, marker='s', label='实际起点', zorder=5)
            ax2.scatter(filtered_actual_df[x_actual].iloc[-1], filtered_actual_df[y_actual].iloc[-1], 
                       c='red', s=150, marker='x', label='实际终点', zorder=5)
        
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('路径对比（滤波后数据）')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.axis('equal')
        
        # 3. X位置随时间变化（原始 vs 滤波）
        ax3 = axes[1, 0]
        if 'timestamp' in planned_df.columns:
            planned_time = planned_df['timestamp'] - planned_df['timestamp'].min()
            ax3.plot(planned_time, planned_df['x'], 'r-', linewidth=2, label='规划路径X', alpha=0.7)
        
        if 'timestamp' in actual_df.columns:
            actual_time = actual_df['timestamp'] - actual_df['timestamp'].min()
            ax3.plot(actual_time, actual_df['x'], 'b-', linewidth=2, label='实际路径X（原始）', alpha=0.7)
            
            # 绘制滤波后的X位置
            if 'x_filtered' in filtered_actual_df.columns:
                ax3.plot(actual_time, filtered_actual_df['x_filtered'], 'b--', linewidth=2, label='实际路径X（滤波后）', alpha=0.9)
        
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('X 位置 (m)')
        ax3.set_title('X位置随时间变化（原始 vs 滤波）')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        
        # 4. Y位置随时间变化（原始 vs 滤波）
        ax4 = axes[1, 1]
        if 'timestamp' in planned_df.columns:
            ax4.plot(planned_time, planned_df['y'], 'r-', linewidth=2, label='规划路径Y', alpha=0.7)
        
        if 'timestamp' in actual_df.columns:
            ax4.plot(actual_time, actual_df['y'], 'b-', linewidth=2, label='实际路径Y（原始）', alpha=0.7)
            
            # 绘制滤波后的Y位置
            if 'y_filtered' in filtered_actual_df.columns:
                ax4.plot(actual_time, filtered_actual_df['y_filtered'], 'b--', linewidth=2, label='实际路径Y（滤波后）', alpha=0.9)
        
        ax4.set_xlabel('时间 (s)')
        ax4.set_ylabel('Y 位置 (m)')
        ax4.set_title('Y位置随时间变化（原始 vs 滤波）')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        
        # 5. 路径偏差图（滤波后数据，全面优化的算法）
        ax5 = axes[2, 0]
        if len(filtered_planned_df) > 0 and len(filtered_actual_df) > 0:
            # 使用滤波后的数据计算偏差
            x_plan = 'x_filtered' if 'x_filtered' in filtered_planned_df.columns else 'x'
            y_plan = 'y_filtered' if 'y_filtered' in filtered_planned_df.columns else 'y'
            x_actual = 'x_filtered' if 'x_filtered' in filtered_actual_df.columns else 'x'
            y_actual = 'y_filtered' if 'y_filtered' in filtered_actual_df.columns else 'y'
            
            deviations = []
            times = []
            
            # 坐标系和时间同步检查
            print("检查坐标系和时间同步...")
            
            # 1. 时间同步检查
            if 'timestamp' in filtered_actual_df.columns and 'timestamp' in filtered_planned_df.columns:
                actual_times = filtered_actual_df['timestamp'].values
                planned_times = filtered_planned_df['timestamp'].values
                
                # 计算时间重叠范围
                actual_start = actual_times.min()
                actual_end = actual_times.max()
                planned_start = planned_times.min()
                planned_end = planned_times.max()
                
                print(f"实际路径时间范围: {actual_start:.3f} - {actual_end:.3f}")
                print(f"规划路径时间范围: {planned_start:.3f} - {planned_end:.3f}")
                
                # 检查时间偏移
                time_offset = abs(actual_start - planned_start)
                print(f"时间偏移: {time_offset:.3f}s")
                
                if time_offset > 10.0:  # 如果时间偏移超过10秒，可能存在时间同步问题
                    print(f"警告: 时间偏移较大 ({time_offset:.3f}s)，可能存在时间同步问题")
                
                # 2. 坐标系检查 - 检查起点位置差异
                if len(filtered_actual_df) > 0 and len(filtered_planned_df) > 0:
                    actual_start_pos = (filtered_actual_df[x_actual].iloc[0], filtered_actual_df[y_actual].iloc[0])
                    planned_start_pos = (filtered_planned_df[x_plan].iloc[0], filtered_planned_df[y_plan].iloc[0])
                    
                    start_dist = np.sqrt((actual_start_pos[0] - planned_start_pos[0])**2 + 
                                       (actual_start_pos[1] - planned_start_pos[1])**2)
                    print(f"起点位置差异: {start_dist:.3f} m")
                    
                    if start_dist > 5.0:  # 如果起点差异超过5米，可能存在坐标系问题
                        print(f"警告: 起点位置差异较大 ({start_dist:.3f}m)，可能存在坐标系不一致问题")
            
            # 使用优化的时间对齐方法计算偏差（只计算共同部分）
            if 'timestamp' in filtered_actual_df.columns and 'timestamp' in filtered_planned_df.columns:
                print("使用优化的时间对齐方法绘制路径偏差图（只计算共同部分）...")
                
                # 时间重叠范围内的实际路径点
                actual_times = filtered_actual_df['timestamp'].values
                planned_times = filtered_planned_df['timestamp'].values
                
                # 找到时间重叠范围
                overlap_start = max(actual_times.min(), planned_times.min())
                overlap_end = min(actual_times.max(), planned_times.max())
                
                if overlap_start < overlap_end:
                    print(f"时间重叠范围: {overlap_start:.3f} - {overlap_end:.3f}")
                    
                    # 只在时间重叠范围内计算偏差
                    overlap_mask = (actual_times >= overlap_start) & (actual_times <= overlap_end)
                    overlap_actual_df = filtered_actual_df[overlap_mask].copy()
                    
                    print(f"重叠时间范围内的实际路径点数: {len(overlap_actual_df)}")
                    
                    for _, actual_point in overlap_actual_df.iterrows():
                        actual_time = actual_point['timestamp']
                        
                        # 在规划路径中找到最接近的时间点
                        time_diffs = np.abs(filtered_planned_df['timestamp'] - actual_time)
                        closest_idx = time_diffs.idxmin()
                        
                        # 检查时间差是否过大
                        time_diff = time_diffs.min()
                        if time_diff > 2.0:  # 如果时间差超过2秒，可能有问题
                            print(f"警告: 时间差过大 {time_diff:.3f}s，可能时间戳不同步")
                            # 尝试使用最近的空间距离匹配
                            x_actual_val = actual_point[x_actual]
                            y_actual_val = actual_point[y_actual]
                            
                            # 计算所有规划路径点到当前实际点的距离
                            distances = np.sqrt(
                                (filtered_planned_df[x_plan] - x_actual_val)**2 + 
                                (filtered_planned_df[y_plan] - y_actual_val)**2
                            )
                            min_dist_idx = distances.idxmin()
                            plan_point = filtered_planned_df.loc[min_dist_idx]
                            dist = distances.min()
                        else:
                            # 时间对齐正常，使用时间匹配
                            plan_point = filtered_planned_df.loc[closest_idx]
                            dist = np.sqrt(
                                (plan_point[x_plan] - actual_point[x_actual])**2 + 
                                (plan_point[y_plan] - actual_point[y_actual])**2
                            )
                        
                        deviations.append(dist)
                        times.append(actual_time - overlap_start)
                else:
                    print("警告: 时间范围没有重叠，无法计算共同部分的偏差")
                    # 如果时间范围没有重叠，无法计算有意义的偏差
                    deviations = []
                    times = []
            else:
                # 备用方法：最近点匹配（注意：这可能包含非共同部分）
                print("使用最近点匹配方法绘制路径偏差图（注意：这可能包含非共同部分）...")
                
                for _, actual_point in filtered_actual_df.iterrows():
                    distances = np.sqrt(
                        (filtered_planned_df[x_plan] - actual_point[x_actual])**2 + 
                        (filtered_planned_df[y_plan] - actual_point[y_actual])**2
                    )
                    min_dist = distances.min()
                    deviations.append(min_dist)
                    
                    if 'timestamp' in filtered_actual_df.columns:
                        times.append(actual_point['timestamp'] - filtered_actual_df['timestamp'].min())
                    else:
                        times.append(len(times))
            
            # 过滤异常值用于绘图（更严格的过滤）
            if deviations:
                deviations = np.array(deviations)
                times = np.array(times)
                
                # 使用更严格的过滤方法
                # 方法1：基于路径长度的动态阈值（更严格）
                if path_length > 0:
                    dynamic_threshold = min(path_length * 0.05, 1.0)  # 进一步降低阈值
                else:
                    dynamic_threshold = 0.3  # 进一步降低默认阈值
                
                # 方法2：基于数据分布的统计阈值（更严格）
                median_dev = np.median(deviations)
                mad = np.median(np.abs(deviations - median_dev))
                statistical_threshold = median_dev + 1.0 * mad  # 进一步降低MAD倍数
                
                # 使用更严格的阈值
                threshold = min(dynamic_threshold, statistical_threshold)
                
                print(f"动态阈值: {dynamic_threshold:.3f} m")
                print(f"统计阈值: {statistical_threshold:.3f} m")
                print(f"最终阈值: {threshold:.3f} m")
                
                # 过滤异常值
                valid_mask = deviations <= threshold
                filtered_deviations = deviations[valid_mask]
                filtered_times = times[valid_mask]
                
                if len(filtered_deviations) > 0:
                    ax5.plot(filtered_times, filtered_deviations, 'g-', linewidth=2, label='路径偏差（滤波后，已过滤异常值）')
                    ax5.fill_between(filtered_times, 0, filtered_deviations, alpha=0.3, color='green')
                    
                    # 绘制统计线
                    avg_dev = np.mean(filtered_deviations)
                    median_dev = np.median(filtered_deviations)
                    p95_dev = np.percentile(filtered_deviations, 95)
                    
                    ax5.axhline(y=avg_dev, color='red', linestyle='--', 
                               label=f'平均偏差: {avg_dev:.3f} m')
                    ax5.axhline(y=median_dev, color='orange', linestyle='--', 
                               label=f'中位数偏差: {median_dev:.3f} m')
                    ax5.axhline(y=p95_dev, color='purple', linestyle='--', 
                               label=f'95%分位数: {p95_dev:.3f} m')
                    
                    ax5.legend()
                    
                    print(f"绘图偏差统计: 平均={avg_dev:.3f}, 中位数={median_dev:.3f}, 95%分位数={p95_dev:.3f}")
                else:
                    # 如果过滤后没有数据，绘制原始数据但标记为异常
                    ax5.plot(times, deviations, 'r-', linewidth=2, label='路径偏差（异常值过多）')
                    ax5.text(0.5, 0.5, '偏差数据异常，\n建议检查数据对齐', 
                            transform=ax5.transAxes, ha='center', va='center',
                            bbox=dict(boxstyle='round', facecolor='red', alpha=0.3))
        
        ax5.set_xlabel('时间 (s)' if 'timestamp' in filtered_actual_df.columns else '点索引')
        ax5.set_ylabel('路径偏差 (m)')
        ax5.set_title('实际路径与规划路径的偏差（滤波后）')
        ax5.grid(True, alpha=0.3)
        
        # 6. 滤波效果对比
        ax6 = axes[2, 1]
        if len(actual_df) > 0 and 'x_filtered' in filtered_actual_df.columns:
            if 'timestamp' in actual_df.columns:
                # 计算速度（数值微分）
                actual_time = actual_df['timestamp'] - actual_df['timestamp'].min()
                
                # 原始速度
                dx_raw = np.gradient(actual_df['x'], actual_time)
                dy_raw = np.gradient(actual_df['y'], actual_time)
                speed_raw = np.sqrt(dx_raw**2 + dy_raw**2)
                
                # 滤波后速度
                dx_filtered = np.gradient(filtered_actual_df['x_filtered'], actual_time)
                dy_filtered = np.gradient(filtered_actual_df['y_filtered'], actual_time)
                speed_filtered = np.sqrt(dx_filtered**2 + dy_filtered**2)
                
                ax6.plot(actual_time, speed_raw, 'b-', linewidth=2, label='速度（原始）', alpha=0.7)
                ax6.plot(actual_time, speed_filtered, 'r-', linewidth=2, label='速度（滤波后）', alpha=0.9)
        
        ax6.set_xlabel('时间 (s)')
        ax6.set_ylabel('速度 (m/s)')
        ax6.set_title('速度对比（滤波效果）')
        ax6.grid(True, alpha=0.3)
        ax6.legend()
        
        plt.tight_layout()
        
        # 保存图表
        base_name = os.path.splitext(os.path.basename(self.bag_file))[0]
        plot_file = os.path.join(self.output_dir, f"{base_name}_analysis_plot_filtered.png")
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"可视化图表（含滤波）已保存到: {plot_file}")
        
        return plot_file

def main():
    parser = argparse.ArgumentParser(description='从rosbag提取并分析导航数据')
    parser.add_argument('bag_file', type=str, help='rosbag文件路径')
    parser.add_argument('--output_dir', type=str, default='./analysis_results',
                       help='输出目录 (默认: ./analysis_results)')
    parser.add_argument('--extract_only', action='store_true',
                       help='仅提取数据，不进行分析')
    
    args = parser.parse_args()
    
    # 检查文件是否存在
    if not os.path.exists(args.bag_file):
        print(f"错误: 文件不存在: {args.bag_file}")
        sys.exit(1)
    
    # 创建分析器
    analyzer = RosbagNavigationAnalyzer(args.bag_file, args.output_dir)
    
    # 提取数据
    if not analyzer.extract_data():
        print("数据提取失败")
        sys.exit(1)
    
    # 保存为CSV
    result_files = analyzer.save_to_csv()
    
    if args.extract_only:
        print("数据提取完成")
        return
    
    # 进行分析（如果有关键数据）
    if 'global_plan' in result_files:
        planned_csv = result_files['global_plan']
        
        # 选择实际路径数据
        actual_csv = None
        for key in ['localization', 'odometry']:
            if key in result_files:
                actual_csv = result_files[key]
                break
        
        if actual_csv:
            # 分析路径对比
            stats = analyzer.analyze_path_comparison(planned_csv, actual_csv)
            
            if stats:
                # 生成报告
                base_name = os.path.splitext(os.path.basename(args.bag_file))[0]
                report_file = os.path.join(args.output_dir, f"{base_name}_analysis_report.txt")
                analyzer.generate_report(stats, report_file)
                
                # 创建可视化
                plot_file = analyzer.create_visualization(planned_csv, actual_csv)
                
                print(f"\n分析完成!")
                print(f"数据文件保存在: {args.output_dir}")
                print(f"分析报告: {report_file}")
                if plot_file:
                    print(f"可视化图表: {plot_file}")
        else:
            print("警告: 未找到实际路径数据，无法进行路径对比分析")
    else:
        print("警告: 未找到规划路径数据，无法进行路径对比分析")
        print("可能的原因:")
        print("  1. rosbag文件中不包含规划路径话题")
        print("  2. 规划路径话题名称不在预定义列表中")
        print("  3. 规划路径数据为空")
        print("建议:")
        print("  1. 检查rosbag文件中的话题列表: rosbag info <bag_file>")
        print("  2. 确保录制rosbag时包含了规划路径话题")
        print("  3. 如果使用其他话题名称，请在脚本中添加相应的话题映射")
        print("  4. 可以尝试分析其他包含规划路径数据的rosbag文件")
        print("  5. 使用 --extract_only 参数仅提取数据而不进行分析")
        
        # 列出当前rosbag中的话题供用户参考
        print("\n当前rosbag中的话题:")
        try:
            bag = rosbag.Bag(analyzer.bag_file)
            for topic, topic_info in bag.get_type_and_topic_info()[1].items():
                print(f"  {topic} (类型: {topic_info.msg_type}, 消息数量: {topic_info.message_count})")
            bag.close()
        except Exception as e:
            print(f"  无法读取rosbag话题信息: {e}")

if __name__ == "__main__":
    main()
