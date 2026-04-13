#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
空间匹配版导航数据分析脚本
专门使用空间匹配算法解决平均偏差过大的问题
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
try:
    # SciPy >= 1.2 usually has this; some environments only expose gaussian under signal.windows
    from scipy.signal import windows as signal_windows  # type: ignore
except Exception:
    signal_windows = None
from scipy.interpolate import interp1d
from scipy.spatial.distance import cdist
import warnings
warnings.filterwarnings('ignore')

# 设置中文字体支持
def setup_chinese_font():
    """设置matplotlib中文字体支持"""
    try:
        plt.rcParams['axes.unicode_minus'] = False
        
        import matplotlib.font_manager as fm
        available_fonts = [f.name for f in fm.fontManager.ttflist]
        
        chinese_fonts = []
        for font_name in available_fonts:
            font_lower = font_name.lower()
            if any(keyword in font_lower for keyword in ['noto', 'cjk', 'chinese', 'simhei', 'yahei', 'ukai', 'uming']):
                chinese_fonts.append(font_name)
        
        font_priority = [
            'Noto Sans CJK SC',
            'Noto Sans CJK TC', 
            'Noto Sans CJK JP',
            'Noto Serif CJK SC',
            'Noto Serif CJK TC',
            'Noto Serif CJK JP',
            'AR PL UMing CN',
            'AR PL UKai CN',
            'Microsoft YaHei',
            'SimHei',
            'Arial Unicode MS'
        ]
        
        selected_font = None
        for preferred_font in font_priority:
            for available_font in chinese_fonts:
                if preferred_font.lower() in available_font.lower():
                    selected_font = available_font
                    break
            if selected_font:
                break
        
        if selected_font:
            plt.rcParams['font.sans-serif'] = [selected_font, 'DejaVu Sans', 'Arial', 'sans-serif']
        else:
            plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'sans-serif']
            print("警告: 未找到中文字体，图表中的中文可能显示为方框")
        
        plt.rcParams['figure.autolayout'] = True
        plt.rcParams['figure.titlesize'] = 16
        plt.rcParams['axes.titlesize'] = 14
        plt.rcParams['axes.labelsize'] = 12
        plt.rcParams['xtick.labelsize'] = 10
        plt.rcParams['ytick.labelsize'] = 10
        plt.rcParams['legend.fontsize'] = 10
        
    except Exception as e:
        print(f"设置中文字体时出错: {e}")
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'sans-serif']
        plt.rcParams['axes.unicode_minus'] = False

setup_chinese_font()

class SpatialMatchingNavigationAnalyzer:
    def __init__(self, bag_file, output_dir):
        """初始化空间匹配版分析器"""
        self.bag_file = bag_file
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
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
        
        topics_to_extract = {
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
            
            '/localization': self.extract_localization,
            '/Odometry': self.extract_odometry,
            '/odom': self.extract_odometry,
            '/odometry/filtered': self.extract_odometry,
            '/odometry/local': self.extract_odometry,
            '/odometry/global': self.extract_odometry,
            '/robot_pose': self.extract_localization,
            '/amcl_pose': self.extract_localization,
            '/pose': self.extract_localization,
            
            '/cmd_vel': self.extract_cmd_vel,
            '/smooth_cmd_vel': self.extract_cmd_vel,
            '/mobile_base/commands/velocity': self.extract_cmd_vel,
            '/twist_mux/cmd_vel': self.extract_cmd_vel,
            '/cmd_vel_smooth': self.extract_cmd_vel,
            
            '/move_base1/current_goal': self.extract_goal,
            '/move_base_simple/goal': self.extract_goal,
            '/move_base/goal': self.extract_goal,
            '/initialpose': self.extract_initial_pose,
            '/goal': self.extract_goal,
            '/navigation/goal': self.extract_goal,
            
            '/tf': self.extract_tf,
            '/tf_static': self.extract_tf,
            
            '/track_points/goal': self.extract_action_goal,
            '/track_points/feedback': self.extract_action_feedback,
            '/track_points/result': self.extract_action_result,
            
            '/simple_line_goal': self.extract_simple_goal,
            '/simple_line_path': self.extract_line_path,
            '/actual_path_marker': self.extract_actual_path_marker
        }
        
        total_msgs = bag.get_message_count()
        print(f"rosbag中共有 {total_msgs} 条消息")
        
        extracted_count = 0
        for topic, msg, t in bag.read_messages():
            if topic in topics_to_extract:
                try:
                    topics_to_extract[topic](msg, t, topic)
                    extracted_count += 1
                except Exception as e:
                    print(f"处理话题 {topic} 时出错: {e}")
            
            if extracted_count % 100 == 0:
                print(f"已处理 {extracted_count} 条消息...")
        
        bag.close()
        print(f"数据提取完成，共提取 {extracted_count} 条消息")
        return True
    
    def extract_global_plan(self, msg, timestamp, topic):
        """提取全局规划路径"""
        try:
            if hasattr(msg, 'poses') and hasattr(msg, 'header'):
                path_id = msg.header.seq if hasattr(msg.header, 'seq') else 0
                frame_id = msg.header.frame_id if hasattr(msg.header, 'frame_id') else ''
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
                            'frame_id': frame_id,
                            'topic': topic
                        })
        except Exception as e:
            print(f"提取全局规划路径时出错: {e}")
    
    def extract_local_plan(self, msg, timestamp, topic):
        """提取局部规划路径"""
        try:
            if hasattr(msg, 'poses') and hasattr(msg, 'header'):
                path_id = msg.header.seq if hasattr(msg.header, 'seq') else 0
                frame_id = msg.header.frame_id if hasattr(msg.header, 'frame_id') else ''
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
                            'frame_id': frame_id,
                            'topic': topic
                        })
        except Exception as e:
            print(f"提取局部规划路径时出错: {e}")
    
    def extract_localization(self, msg, timestamp, topic):
        """提取定位数据"""
        if hasattr(msg, 'pose'):
            pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
            frame_id = msg.header.frame_id if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id') else ''
            self.data['localization'].append({
                'timestamp': timestamp.to_sec(),
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'frame_id': frame_id,
                'topic': topic
            })
    
    def extract_odometry(self, msg, timestamp, topic):
        """提取里程计数据"""
        if hasattr(msg, 'pose') and hasattr(msg, 'twist'):
            pose = msg.pose.pose
            twist = msg.twist.twist
            frame_id = msg.header.frame_id if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id') else ''
            child_frame_id = msg.child_frame_id if hasattr(msg, 'child_frame_id') else ''
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
                'frame_id': frame_id,
                'child_frame_id': child_frame_id,
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
            frame_id = msg.header.frame_id if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id') else ''
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'frame_id': frame_id,
                'topic': topic,
                'goal_type': 'current_goal' if 'current_goal' in topic else 'simple_goal'
            })
    
    def extract_initial_pose(self, msg, timestamp, topic):
        """提取初始位姿"""
        if isinstance(msg, PoseWithCovarianceStamped):
            pose = msg.pose.pose
            frame_id = msg.header.frame_id if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id') else ''
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'frame_id': frame_id,
                'topic': topic,
                'goal_type': 'initial_pose'
            })
    
    def extract_action_goal(self, msg, timestamp, topic):
        """提取Action目标"""
        if hasattr(msg, 'goal') and hasattr(msg.goal, 'path_group_name'):
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
        if hasattr(msg, 'feedback') and hasattr(msg.feedback, 'current_waypoint_info'):
            feedback = msg.feedback
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'current_waypoint_info': feedback.current_waypoint_info,
                'topic': topic,
                'goal_type': 'action_feedback'
            })
    
    def extract_action_result(self, msg, timestamp, topic):
        """提取Action结果"""
        if hasattr(msg, 'result') and hasattr(msg.result, 'success'):
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
            frame_id = msg.header.frame_id if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id') else ''
            self.data['goals'].append({
                'timestamp': timestamp.to_sec(),
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'frame_id': frame_id,
                'topic': topic,
                'goal_type': 'simple_line_goal'
            })
    
    def extract_line_path(self, msg, timestamp, topic):
        """提取simple_line_path"""
        if hasattr(msg, 'points') and len(msg.points) >= 2:
            start_point = msg.points[0]
            end_point = msg.points[-1]
            
            self.data['global_plan'].append({
                'timestamp': timestamp.to_sec(),
                'path_id': 0,
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
            
            self.data['global_plan'].append({
                'timestamp': timestamp.to_sec(),
                'path_id': 0,
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
            for i, point in enumerate(msg.points):
                self.data['odometry'].append({
                    'timestamp': timestamp.to_sec(),
                    'x': point.x,
                    'y': point.y,
                    'z': point.z,
                    'vx': 0.0,
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
                    'parent_frame': (transform.header.frame_id or '').lstrip('/'),
                    'child_frame': (transform.child_frame_id or '').lstrip('/'),
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
        
        print("数据保存完成")
        
        result_files = {}
        for data_type in ['global_plan', 'localization', 'odometry']:
            csv_file = os.path.join(self.output_dir, f"{base_name}_{data_type}.csv")
            if os.path.exists(csv_file):
                result_files[data_type] = csv_file
        
        return result_files
    
    def apply_filters(self, df, columns=['x', 'y'], filter_type='savgol', window_length=5, polyorder=3):
        """对数据应用滤波器（增强版）"""
        if len(df) < window_length:
            print(f"警告: 数据点数量 ({len(df)}) 小于滤波器窗口长度 ({window_length})，跳过滤波")
            return df
        
        filtered_df = df.copy()
        
        for col in columns:
            if col in df.columns:
                try:
                    if filter_type == 'savgol':
                        # 使用更长的窗口和更高阶数
                        filtered_values = signal.savgol_filter(df[col], window_length=window_length, polyorder=polyorder)
                    elif filter_type == 'butter':
                        nyquist = 0.5 * (len(df) / (df['timestamp'].max() - df['timestamp'].min()))
                        cutoff = 0.05 * nyquist  # 更低的截止频率
                        b, a = signal.butter(6, cutoff / nyquist, btype='low')  # 更高阶数
                        filtered_values = signal.filtfilt(b, a, df[col])
                    elif filter_type == 'moving_avg':
                        # 使用加权移动平均
                        filtered_values = df[col].rolling(window=window_length, center=True).mean()
                        filtered_values = filtered_values.fillna(df[col])
                    elif filter_type == 'gaussian':
                        # 高斯滤波（兼容不同SciPy版本）
                        sigma = window_length / 6
                        if signal_windows is not None and hasattr(signal_windows, 'gaussian'):
                            kernel = signal_windows.gaussian(window_length, std=sigma)
                        else:
                            # fallback: 手动生成高斯核
                            n = np.arange(window_length) - (window_length - 1) / 2.0
                            kernel = np.exp(-0.5 * (n / (sigma + 1e-9)) ** 2)
                        kernel = kernel / (np.sum(kernel) + 1e-12)
                        filtered_values = np.convolve(df[col], kernel, mode='same')
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
        
        df = df.sort_values('timestamp').reset_index(drop=True)
        
        t_min = df['timestamp'].min()
        t_max = df['timestamp'].max()
        
        target_times = np.arange(t_min, t_max, 1.0/target_freq)
        
        cols_to_interpolate = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz']
        interpolated_data = {'timestamp': target_times}
        
        for col in cols_to_interpolate:
            if col in df.columns:
                f = interp1d(df['timestamp'], df[col], kind='linear', 
                           bounds_error=False, fill_value='extrapolate')
                interpolated_data[col] = f(target_times)
        
        return pd.DataFrame(interpolated_data)

    @staticmethod
    def _point_to_polyline_distance(point_xy: np.ndarray, polyline_xy: np.ndarray) -> float:
        """
        计算二维点到二维折线(由顶点序列组成)的最短距离。
        - point_xy: shape (2,)
        - polyline_xy: shape (N, 2), N>=1
        返回值单位与输入一致（通常为米）。
        """
        if polyline_xy is None or len(polyline_xy) == 0:
            return float('nan')
        if len(polyline_xy) == 1:
            d = polyline_xy[0] - point_xy
            return float(np.hypot(d[0], d[1]))

        p = point_xy.astype(float)
        a = polyline_xy[:-1].astype(float)  # (N-1, 2)
        b = polyline_xy[1:].astype(float)   # (N-1, 2)
        ab = b - a
        ap = p - a

        ab2 = np.sum(ab * ab, axis=1)  # (N-1,)
        # 防止重复点导致除0
        ab2 = np.where(ab2 < 1e-12, 1e-12, ab2)

        t = np.sum(ap * ab, axis=1) / ab2
        t = np.clip(t, 0.0, 1.0)
        proj = a + (ab * t[:, None])
        diff = proj - p
        d2 = np.sum(diff * diff, axis=1)
        return float(np.sqrt(np.min(d2)))

    @staticmethod
    def _point_to_infinite_line_distance(point_xy: np.ndarray, a_xy: np.ndarray, b_xy: np.ndarray) -> float:
        """
        二维点到“无限延长直线”(由a->b定义)的最短距离。
        适用于逐点导航中每段只有两个端点的直线任务（避免线段端点效应导致段首/段尾尖峰）。
        """
        p = point_xy.astype(float)
        a = a_xy.astype(float)
        b = b_xy.astype(float)
        ab = b - a
        ab_len = float(np.hypot(ab[0], ab[1]))
        if ab_len < 1e-9:
            d = p - a
            return float(np.hypot(d[0], d[1]))
        # 2D cross product magnitude divided by |ab|
        ap = p - a
        cross = abs(ab[0] * ap[1] - ab[1] * ap[0])
        return float(cross / ab_len)

    @staticmethod
    def _line_segment_projection_t(point_xy: np.ndarray, a_xy: np.ndarray, b_xy: np.ndarray) -> float:
        """
        计算点在a->b方向上的投影参数t：
        p_proj = a + t*(b-a)
        - t<0: 投影在a点之前
        - 0<=t<=1: 投影落在线段内部
        - t>1: 投影在b点之后
        """
        p = point_xy.astype(float)
        a = a_xy.astype(float)
        b = b_xy.astype(float)
        ab = b - a
        ab2 = float(ab[0] * ab[0] + ab[1] * ab[1])
        if ab2 < 1e-12:
            return float('nan')
        ap = p - a
        return float((ap[0] * ab[0] + ap[1] * ab[1]) / ab2)

    @staticmethod
    def _densify_polyline(polyline_xy: np.ndarray, resolution: float = 0.05) -> np.ndarray:
        """
        将折线按给定分辨率（米）加密采样，降低“离散顶点过稀导致的偏差偏大”问题。
        resolution<=0 时返回原始折线。
        """
        if polyline_xy is None or len(polyline_xy) < 2 or resolution is None or resolution <= 0:
            return polyline_xy

        pts = polyline_xy.astype(float)
        out = [pts[0]]
        for i in range(1, len(pts)):
            p0 = pts[i - 1]
            p1 = pts[i]
            v = p1 - p0
            seg_len = float(np.hypot(v[0], v[1]))
            if seg_len < 1e-9:
                continue
            n = int(np.floor(seg_len / resolution))
            # 在(0,1)之间均匀插入点，避免重复加入端点
            for k in range(1, n + 1):
                alpha = (k * resolution) / seg_len
                if alpha >= 1.0:
                    break
                out.append(p0 + alpha * v)
            out.append(p1)
        return np.asarray(out)

    @staticmethod
    def _normalize_frame_id(frame_id: str) -> str:
        return (frame_id or '').strip().lstrip('/')

    def _build_tf_interpolator(self, tf_df: pd.DataFrame, parent: str, child: str):
        """
        构建 parent->child 的二维TF插值器（x,y,yaw），用于坐标变换。
        仅处理“单跳”TF（直接父子关系），足够覆盖常见 map<->odom。
        返回: (times, x, y, yaw) 或 None
        """
        if tf_df is None or len(tf_df) == 0:
            return None
        if 'timestamp' not in tf_df.columns:
            return None

        parent = self._normalize_frame_id(parent)
        child = self._normalize_frame_id(child)
        if not parent or not child:
            return None

        df = tf_df.copy()
        df['parent_frame'] = df['parent_frame'].astype(str).map(self._normalize_frame_id)
        df['child_frame'] = df['child_frame'].astype(str).map(self._normalize_frame_id)

        df = df[(df['parent_frame'] == parent) & (df['child_frame'] == child)].sort_values('timestamp')
        if len(df) < 2:
            return None

        # quat->yaw
        def quat_to_yaw(row):
            q = [row['qx'], row['qy'], row['qz'], row['qw']]
            try:
                _, _, yaw = tf_trans.euler_from_quaternion(q)
                return yaw
            except Exception:
                return 0.0

        yaws = df.apply(quat_to_yaw, axis=1).values.astype(float)
        ts = df['timestamp'].values.astype(float)
        xs = df['x'].values.astype(float)
        ys = df['y'].values.astype(float)

        # unwrap yaw for interpolation stability
        yaws = np.unwrap(yaws)
        return ts, xs, ys, yaws

    def _transform_points_2d(self, points_df: pd.DataFrame, tf_interp, direction: str):
        """
        使用二维TF把 points_df 里的 (x,y) 进行变换。
        - tf_interp: (ts, tx, ty, tyaw) for parent->child
        - direction:
            - 'child_to_parent': 输入点在 child，输出到 parent（使用 parent->child 的逆）
            - 'parent_to_child': 输入点在 parent，输出到 child（直接用 parent->child）
        要求 points_df 有 timestamp, x, y。
        """
        if tf_interp is None or points_df is None or len(points_df) == 0:
            return points_df
        if 'timestamp' not in points_df.columns:
            return points_df

        ts, tx, ty, tyaw = tf_interp
        t_query = points_df['timestamp'].values.astype(float)

        # 线性插值 (bounds外用端点外推，减少短bag边界问题)
        fx = interp1d(ts, tx, kind='linear', bounds_error=False, fill_value=(tx[0], tx[-1]))
        fy = interp1d(ts, ty, kind='linear', bounds_error=False, fill_value=(ty[0], ty[-1]))
        fyaw = interp1d(ts, tyaw, kind='linear', bounds_error=False, fill_value=(tyaw[0], tyaw[-1]))

        x_t = fx(t_query)
        y_t = fy(t_query)
        yaw_t = fyaw(t_query)

        x = points_df['x'].values.astype(float)
        y = points_df['y'].values.astype(float)

        c = np.cos(yaw_t)
        s = np.sin(yaw_t)

        out = points_df.copy()
        if direction == 'parent_to_child':
            # p_child = R * p_parent + t
            out['x'] = c * x - s * y + x_t
            out['y'] = s * x + c * y + y_t
        else:
            # child_to_parent: p_parent = R^T * (p_child - t)
            dx = x - x_t
            dy = y - y_t
            out['x'] = c * dx + s * dy
            out['y'] = -s * dx + c * dy
        return out
    
    def analyze_path_comparison_spatial(self, planned_csv, actual_csv):
        """空间匹配版路径对比分析（只计算共同部分，增强滤波）"""
        print("正在使用空间匹配算法分析路径对比（只计算共同部分，增强滤波）...")
        
        try:
            planned_df = pd.read_csv(planned_csv)
            actual_df = pd.read_csv(actual_csv)
        except Exception as e:
            print(f"读取CSV文件失败: {e}")
            return None
        
        # 注意：逐点导航/频繁重规划时会产生多个path_id
        # 偏差计算阶段会为每个实际点“动态选择当时最新的规划路径”，避免只选一条path导致后半段（如后退/换目标）偏差虚高
        
        # 数据预处理：多级滤波和插值
        print("正在对数据进行多级滤波和插值处理...")
        
        # 第一级滤波：对实际路径数据应用Savitzky-Golay滤波器（更强的滤波）
        filtered_actual_df = self.apply_filters(actual_df, filter_type='savgol', window_length=15, polyorder=4)
        
        # 第二级滤波：对实际路径数据应用高斯滤波（进一步平滑）
        if len(filtered_actual_df) > 30:
            filtered_actual_df = self.apply_filters(filtered_actual_df, filter_type='gaussian', window_length=11, polyorder=0)
        
        # 对规划路径数据应用移动平均滤波器
        if len(planned_df) > 20:
            filtered_planned_df = self.apply_filters(planned_df, filter_type='moving_avg', window_length=7)
        else:
            filtered_planned_df = planned_df.copy()

        # 构建规划路径分段（按“每次plan消息”分组），用于逐点导航/重规划下的动态匹配
        # 说明：
        # - move_base 场景下 path_id 通常会变；而 simple_line_path 场景下 path_id 可能固定为0
        # - 因此这里用 (topic, timestamp, path_id) 作为一条“plan实例”，更稳健
        plan_segments = []
        if len(filtered_planned_df) > 0 and 'timestamp' in filtered_planned_df.columns:
            x_plan_col = 'x_filtered' if 'x_filtered' in filtered_planned_df.columns else 'x'
            y_plan_col = 'y_filtered' if 'y_filtered' in filtered_planned_df.columns else 'y'
            group_cols = ['timestamp']
            if 'topic' in filtered_planned_df.columns:
                group_cols.append('topic')
            if 'path_id' in filtered_planned_df.columns:
                group_cols.append('path_id')

            for keys, g in filtered_planned_df.groupby(group_cols):
                g2 = g.sort_values('point_index' if 'point_index' in g.columns else 'timestamp')
                if len(g2) < 2:
                    continue
                seg_start_t = float(g2['timestamp'].min())
                poly_raw = g2[[x_plan_col, y_plan_col]].values
                is_two_point_line = len(poly_raw) == 2
                poly = self._densify_polyline(poly_raw, resolution=0.05)
                # 尽量保留path_id（无则为-1）
                pid = int(g2['path_id'].iloc[0]) if 'path_id' in g2.columns else -1
                topic = str(g2['topic'].iloc[0]) if 'topic' in g2.columns else ''
                # store raw endpoints for infinite-line distance when needed
                a0 = poly_raw[0].copy()
                b0 = poly_raw[1].copy()
                plan_segments.append((seg_start_t, pid, topic, poly, is_two_point_line, a0, b0))

            plan_segments.sort(key=lambda x: x[0])

        if len(plan_segments) > 0:
            # 统计一下来源topic，帮助判断参考路径是否选对
            topics = {}
            for seg in plan_segments:
                tp = seg[2]
                topics[tp] = topics.get(tp, 0) + 1
            print(f"检测到 {len(plan_segments)} 条规划路径实例（按消息分组），来源分布: {topics}")
            # 统计两点直线段数量（通常对应/simple_line_path）
            two_pt = sum(1 for s in plan_segments if len(s) > 4 and s[4])
            print(f"其中两点直线段数量: {two_pt}")

        # 坐标系一致性检查：若frame不同且tf可用，则尝试把实际路径变换到规划路径frame
        plan_frame = ''
        actual_frame = ''
        if 'frame_id' in filtered_planned_df.columns and len(filtered_planned_df) > 0:
            plan_frame = self._normalize_frame_id(str(filtered_planned_df['frame_id'].dropna().iloc[0])) if filtered_planned_df['frame_id'].notna().any() else ''
        if 'frame_id' in filtered_actual_df.columns and len(filtered_actual_df) > 0:
            actual_frame = self._normalize_frame_id(str(filtered_actual_df['frame_id'].dropna().iloc[0])) if filtered_actual_df['frame_id'].notna().any() else ''

        if plan_frame and actual_frame and plan_frame != actual_frame:
            print(f"警告: 规划路径frame_id='{plan_frame}', 实际路径frame_id='{actual_frame}' 不一致，偏差可能显著偏大")
            if len(self.data.get('tf_data', [])) > 0:
                tf_df = pd.DataFrame(self.data['tf_data'])
                # 尝试常见的 map<->odom / prior_map<->odom
                # 优先找 plan_frame -> actual_frame（parent->child）
                tf_interp = self._build_tf_interpolator(tf_df, parent=plan_frame, child=actual_frame)
                if tf_interp is not None:
                    print(f"使用TF {plan_frame}->{actual_frame} 将实际路径从'{actual_frame}'变换到'{plan_frame}'（取逆变换）")
                    filtered_actual_df = self._transform_points_2d(filtered_actual_df, tf_interp, direction='child_to_parent')
                else:
                    # 再尝试 actual_frame -> plan_frame
                    tf_interp = self._build_tf_interpolator(tf_df, parent=actual_frame, child=plan_frame)
                    if tf_interp is not None:
                        print(f"使用TF {actual_frame}->{plan_frame} 将实际路径从'{actual_frame}'变换到'{plan_frame}'（直接变换）")
                        filtered_actual_df = self._transform_points_2d(filtered_actual_df, tf_interp, direction='parent_to_child')
                    else:
                        print("警告: 未找到可用的单跳TF用于坐标变换（常见原因：frame名不匹配，如 'prior_map' vs 'map'，或bag未录到对应TF）")

        # 应用静态坐标变换补偿雷达到机器人中心的偏移
        # 根据 sentry_localize.launch 中的静态变换: body -> body_foot 平移 (-0.75, 0, 0.75)
        # 假设实际路径是雷达位置(body)，规划路径是机器人中心(body_foot)
        # 需要将实际路径从雷达坐标系变换到机器人中心坐标系
        print("应用静态坐标变换补偿雷达到机器人中心的偏移...")
        x_offset = -0.75  # x方向偏移：雷达在机器人中心后面0.75米，所以需要减去0.75米（机器人中心在雷达前面）
        y_offset = 0.0   # y方向无偏移

        # 应用平移变换到实际路径数据
        if 'x_filtered' in filtered_actual_df.columns:
            filtered_actual_df['x_filtered'] = filtered_actual_df['x_filtered'] + x_offset
        elif 'x' in filtered_actual_df.columns:
            filtered_actual_df['x'] = filtered_actual_df['x'] + x_offset

        if 'y_filtered' in filtered_actual_df.columns:
            filtered_actual_df['y_filtered'] = filtered_actual_df['y_filtered'] + y_offset
        elif 'y' in filtered_actual_df.columns:
            filtered_actual_df['y'] = filtered_actual_df['y'] + y_offset

        print(f"应用静态变换: x += {x_offset}, y += {y_offset}")

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
        
        # 计算路径长度
        def calculate_path_length(df, use_filtered=True):
            if len(df) < 2:
                return 0
            
            if 'timestamp' in df.columns:
                df = df.sort_values('timestamp')
            
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
        
        # 空间匹配版路径偏差计算（只计算共同部分）
        if len(filtered_planned_df) > 0 and len(filtered_actual_df) > 0:
            print("开始空间匹配版路径偏差计算（只计算共同部分）...")
            
            # 使用滤波后的数据
            x_plan = 'x_filtered' if 'x_filtered' in filtered_planned_df.columns else 'x'
            y_plan = 'y_filtered' if 'y_filtered' in filtered_planned_df.columns else 'y'
            x_actual = 'x_filtered' if 'x_filtered' in filtered_actual_df.columns else 'x'
            y_actual = 'y_filtered' if 'y_filtered' in filtered_actual_df.columns else 'y'
            
            deviations = []
            times = []
            actual_xy_for_dev = []  # 用于导出偏差序列，便于定位“突增”原因
            chosen_plan_meta = []   # (plan_path_id, plan_start_time) for each deviation sample
            chosen_plan_topic = []
            chosen_plan_is_line2 = []
            chosen_dev_method = []  # 'polyline' or 'infinite_line'
            chosen_line_t = []      # projection t for two-point lines, else nan
            chosen_in_segment = []  # whether 0<=t<=1 for two-point lines, else True
            
            # 1. 确定共同部分的时间范围
            print("=== 确定共同部分的时间范围 ===")
            
            if 'timestamp' in filtered_actual_df.columns and 'timestamp' in filtered_planned_df.columns:
                actual_times = filtered_actual_df['timestamp'].values
                planned_times = filtered_planned_df['timestamp'].values
                
                # 计算时间重叠范围
                overlap_start = max(actual_times.min(), planned_times.min())
                overlap_end = min(actual_times.max(), planned_times.max())
                
                print(f"实际路径时间范围: {actual_times.min():.3f} - {actual_times.max():.3f}")
                print(f"规划路径时间范围: {planned_times.min():.3f} - {planned_times.max():.3f}")
                print(f"共同部分时间范围: {overlap_start:.3f} - {overlap_end:.3f}")
                
                # 检查是否有重叠
                if overlap_start < overlap_end:
                    print(f"时间重叠时长: {overlap_end - overlap_start:.3f}s")
                    
                    # 2. 提取共同部分的数据
                    print("=== 提取共同部分的数据 ===")
                    
                    # 提取共同时间范围内的实际路径点
                    overlap_mask_actual = (actual_times >= overlap_start) & (actual_times <= overlap_end)
                    overlap_actual_df = filtered_actual_df[overlap_mask_actual].copy()
                    
                    # 提取共同时间范围内的规划路径点
                    overlap_mask_planned = (planned_times >= overlap_start) & (planned_times <= overlap_end)
                    overlap_planned_df = filtered_planned_df[overlap_mask_planned].copy()
                    
                    print(f"共同部分实际路径点数: {len(overlap_actual_df)}")
                    print(f"共同部分规划路径点数: {len(overlap_planned_df)}")
                    
                    if len(overlap_actual_df) > 0 and len(overlap_planned_df) > 0:
                        # 3. 空间匹配算法：为每个实际路径点找到最近的规划路径点
                        print("=== 空间匹配算法（只对共同部分）===")
                        
                        # 为了提高效率，对实际路径进行下采样
                        if len(overlap_actual_df) > 2000:
                            sample_rate = len(overlap_actual_df) // 2000
                            actual_points = overlap_actual_df.iloc[::sample_rate]
                            print(f"共同部分实际路径点过多，已下采样到 {len(actual_points)} 个点")
                        else:
                            actual_points = overlap_actual_df
                        
                        # 为每个实际点动态选择“当时最新的规划路径段”
                        seg_times = np.array([s[0] for s in plan_segments], dtype=float) if len(plan_segments) > 0 else None
                        # 逐点直线导航优先使用 /simple_line_path 作为参考（若存在）
                        preferred_topics = ['/simple_line_path']
                        
                        print(f"共同部分实际路径点数: {len(actual_points)}")
                        
                        # 为每个实际路径点计算到“当时规划折线(线段)”的最小距离
                        for _, actual_point in actual_points.iterrows():
                            actual_pos = np.array([actual_point[x_actual], actual_point[y_actual]])
                            if len(plan_segments) > 0 and 'timestamp' in overlap_actual_df.columns:
                                t_abs = float(actual_point['timestamp'])
                                # 先在 preferred_topics 中找最新一条；找不到则退化为任意topic最新一条
                                idx = int(np.searchsorted(seg_times, t_abs, side='right') - 1)
                                if idx < 0:
                                    idx = 0
                                chosen = plan_segments[idx]
                                if preferred_topics:
                                    for j in range(idx, -1, -1):
                                        if plan_segments[j][2] in preferred_topics:
                                            chosen = plan_segments[j]
                                            break
                                plan_start_t, plan_pid, plan_topic, plan_poly, is_line2, a0, b0 = chosen
                            else:
                                # 退化：用共同时间范围内的规划点
                                plan_poly = self._densify_polyline(overlap_planned_df[[x_plan, y_plan]].values, resolution=0.05)
                                plan_pid = -1
                                plan_start_t = float('nan')
                                plan_topic = ''
                                is_line2 = False
                                a0 = None
                                b0 = None

                            # 对两点直线任务（常见/simple_line_path），用“点到无限延长直线距离”避免端点尖峰
                            if is_line2 and a0 is not None and b0 is not None:
                                t_seg = self._line_segment_projection_t(actual_pos, a0, b0)
                                in_seg = (t_seg >= 0.0) and (t_seg <= 1.0)
                                # 逐点导航“切段初期”常出现 t<0：此时机器人尚未进入该段，计入偏差会形成尖峰
                                # 因此：只在投影落在线段内时才用于统计与绘图；否则仍导出但标记为不计入
                                dev = self._point_to_infinite_line_distance(actual_pos, a0, b0)
                                dev_method = 'infinite_line'
                            else:
                                t_seg = float('nan')
                                in_seg = True
                                dev = self._point_to_polyline_distance(actual_pos, plan_poly)
                                dev_method = 'polyline'
                            deviations.append(dev if in_seg else float('nan'))
                            actual_xy_for_dev.append((float(actual_point[x_actual]), float(actual_point[y_actual])))
                            chosen_plan_meta.append((int(plan_pid), float(plan_start_t)))
                            chosen_plan_topic.append(str(plan_topic))
                            chosen_plan_is_line2.append(bool(is_line2))
                            chosen_dev_method.append(dev_method)
                            chosen_line_t.append(float(t_seg))
                            chosen_in_segment.append(bool(in_seg))
                            
                            if 'timestamp' in overlap_actual_df.columns:
                                times.append(actual_point['timestamp'] - overlap_actual_df['timestamp'].min())
                            else:
                                times.append(len(times))
                        
                        print(f"共同部分空间匹配完成，共计算 {len(deviations)} 个偏差值")
                    else:
                        print("警告: 共同部分没有有效数据")
                        deviations = []
                        times = []
                else:
                    print("警告: 时间范围没有重叠，无法计算共同部分的偏差")
                    deviations = []
                    times = []
            else:
                # 如果没有时间戳，使用纯空间匹配（但会标记为不可靠）
                print("=== 无时间戳，使用纯空间匹配（可能包含非共同部分）===")
                
                # 为了提高效率，对实际路径进行下采样
                if len(filtered_actual_df) > 2000:
                    sample_rate = len(filtered_actual_df) // 2000
                    actual_points = filtered_actual_df.iloc[::sample_rate]
                    print(f"实际路径点过多，已下采样到 {len(actual_points)} 个点")
                else:
                    actual_points = filtered_actual_df
                
                # 无时间戳时无法动态选择规划段：退化为使用全部规划点的折线（可能偏小/偏大，取决于路径重叠程度）
                plan_positions_raw = filtered_planned_df[[x_plan, y_plan]].values
                plan_positions = self._densify_polyline(plan_positions_raw, resolution=0.05)
                
                print(f"规划路径点数: {len(plan_positions)}")
                print(f"实际路径点数: {len(actual_points)}")
                
                # 为每个实际路径点计算到规划折线(线段)的最小距离
                for _, actual_point in actual_points.iterrows():
                    actual_pos = np.array([actual_point[x_actual], actual_point[y_actual]])
                    dev = self._point_to_polyline_distance(actual_pos, plan_positions)
                    deviations.append(dev)
                    actual_xy_for_dev.append((float(actual_point[x_actual]), float(actual_point[y_actual])))
                    chosen_plan_meta.append((-1, float('nan')))
                    chosen_plan_topic.append('')
                    chosen_plan_is_line2.append(False)
                    chosen_dev_method.append('polyline')
                    chosen_line_t.append(float('nan'))
                    chosen_in_segment.append(True)
                    
                    if 'timestamp' in filtered_actual_df.columns:
                        times.append(actual_point['timestamp'] - filtered_actual_df['timestamp'].min())
                    else:
                        times.append(len(times))
                
                print(f"空间匹配完成，共计算 {len(deviations)} 个偏差值（注意：可能包含非共同部分）")
            
            # 导出偏差时间序列（方便定位“偏差突然变大”的时刻）
            try:
                if len(deviations) > 0:
                    base_name = os.path.splitext(os.path.basename(self.bag_file))[0]
                    dev_csv = os.path.join(self.output_dir, f"{base_name}_deviation_series.csv")
                    dev_df = pd.DataFrame({
                        't_rel': times if len(times) == len(deviations) else np.arange(len(deviations)),
                        'deviation': deviations,
                        'x_actual': [p[0] for p in actual_xy_for_dev] if len(actual_xy_for_dev) == len(deviations) else np.nan,
                        'y_actual': [p[1] for p in actual_xy_for_dev] if len(actual_xy_for_dev) == len(deviations) else np.nan,
                        'plan_path_id': [m[0] for m in chosen_plan_meta] if len(chosen_plan_meta) == len(deviations) else np.nan,
                        'plan_start_time': [m[1] for m in chosen_plan_meta] if len(chosen_plan_meta) == len(deviations) else np.nan,
                        'plan_topic': chosen_plan_topic if len(chosen_plan_topic) == len(deviations) else '',
                        'plan_is_two_point_line': chosen_plan_is_line2 if len(chosen_plan_is_line2) == len(deviations) else False,
                        'dev_method': chosen_dev_method if len(chosen_dev_method) == len(deviations) else '',
                        'line_t': chosen_line_t if len(chosen_line_t) == len(deviations) else np.nan,
                        'in_segment': chosen_in_segment if len(chosen_in_segment) == len(deviations) else True,
                    })
                    dev_df.to_csv(dev_csv, index=False)
                    print(f"偏差时间序列已保存到: {dev_csv}")
            except Exception as e:
                print(f"保存偏差时间序列失败: {e}")

            # 空间匹配的异常值过滤（简化版，避免把真实较大的偏差过滤掉）
            if deviations:
                deviations = np.array(deviations, dtype=float)
                times = np.array(times)
                
                print("=== 空间匹配的异常值过滤（简化版）===")
                # 先去掉 NaN（例如两点直线段切段初期 t<0 的点）
                finite_mask = np.isfinite(deviations)
                deviations = deviations[finite_mask]
                times = times[finite_mask]
                # 仅去掉明显不合理的极端值（例如 >20m），其余全部保留参与统计
                max_reasonable_deviation = 20.0  # 室内/楼宇场景已经非常保守
                valid_mask = deviations < max_reasonable_deviation
                filtered_deviations = deviations[valid_mask]
                filtered_times = times[valid_mask]
                
                if len(filtered_deviations) == 0:
                    # 如果全部被判为极端值，就退回使用原始数据
                    filtered_deviations = deviations
                    filtered_times = times
                    print("警告: 所有偏差都超过极端阈值，直接使用原始偏差")
                
                print(f"原始偏差数量: {len(deviations)}")
                print(f"极端值阈值: {max_reasonable_deviation:.1f} m")
                print(f"过滤后偏差数量: {len(filtered_deviations)}")
                
                if len(filtered_deviations) > 0:
                    # 计算统计指标
                    stats['avg_deviation'] = np.mean(filtered_deviations)
                    stats['max_deviation'] = np.max(filtered_deviations)
                    stats['std_deviation'] = np.std(filtered_deviations)
                    stats['median_deviation'] = np.median(filtered_deviations)
                    stats['p95_deviation'] = np.percentile(filtered_deviations, 95)
                    stats['p99_deviation'] = np.percentile(filtered_deviations, 99)
                    
                    # 计算偏差的变异系数（CV）
                    stats['cv_deviation'] = stats['std_deviation'] / stats['avg_deviation'] if stats['avg_deviation'] > 0 else 0
                    
                    print(f"原始偏差统计: 平均={np.mean(deviations):.3f}, 最大={np.max(deviations):.3f}")
                    print(f"过滤后偏差统计: 平均={stats['avg_deviation']:.3f}, 最大={stats['max_deviation']:.3f}")
                    print(f"过滤了 {len(deviations) - len(filtered_deviations)} 个异常值")
                    print(f"中位数偏差: {stats['median_deviation']:.3f} m")
                    print(f"95%分位数: {stats['p95_deviation']:.3f} m")
                    print(f"偏差变异系数: {stats['cv_deviation']:.3f}")
                    
                    # 添加偏差质量评估（阈值保持不变）
                    if stats['avg_deviation'] < 0.1:
                        stats['deviation_quality'] = '优秀'
                    elif stats['avg_deviation'] < 0.3:
                        stats['deviation_quality'] = '良好'
                    elif stats['avg_deviation'] < 0.5:
                        stats['deviation_quality'] = '一般'
                    else:
                        stats['deviation_quality'] = '较差'
                    
                    print(f"偏差质量评估: {stats['deviation_quality']}")
                else:
                    # 如果过滤后没有数据，使用原始数据但记录警告
                    stats['avg_deviation'] = np.mean(deviations)
                    stats['max_deviation'] = np.max(deviations)
                    stats['std_deviation'] = np.std(deviations)
                    stats['median_deviation'] = np.median(deviations)
                    stats['p95_deviation'] = np.percentile(deviations, 95) if len(deviations) > 0 else 0
                    stats['p99_deviation'] = np.percentile(deviations, 99) if len(deviations) > 0 else 0
                    stats['cv_deviation'] = stats['std_deviation'] / stats['avg_deviation'] if stats['avg_deviation'] > 0 else 0
                    stats['deviation_quality'] = '数据异常'
                    print("警告: 过滤后没有有效数据，使用原始偏差值")
            else:
                print("警告: 无法计算路径偏差")
        
        # 计算起点和终点误差
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
    
    def generate_spatial_report(self, stats, output_file):
        """生成空间匹配版分析报告"""
        print("正在生成空间匹配版分析报告...")
        
        with open(output_file, 'w') as f:
            f.write("=" * 60 + "\n")
            f.write("空间匹配版导航数据分析报告\n")
            f.write("=" * 60 + "\n\n")
            
            f.write(f"分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"rosbag文件: {self.bag_file}\n")
            f.write(f"输出目录: {self.output_dir}\n\n")
            
            f.write("算法说明:\n")
            f.write("-" * 40 + "\n")
            f.write("  本报告使用空间匹配算法：优先使用时间重叠段，仅对共同时间范围内数据计算\n")
            f.write("  空间距离采用“点到规划折线(线段)的最短距离”，而非仅到离散顶点的最近距离\n")
            f.write("  为每个实际路径点找到最近的规划路径点计算偏差\n")
            f.write("  这种方法避免了时间同步问题，适用于:\n")
            f.write("    - 时间戳不同步的数据\n")
            f.write("    - 不同时期录制的数据\n")
            f.write("    - 时间戳质量差的数据\n")
            f.write("    - 需要纯粹空间对比的场景\n\n")
            
            f.write("数据统计:\n")
            f.write("-" * 40 + "\n")
            for key, value in stats.items():
                if key not in ['deviation_quality']:
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
            
            # 路径偏差（空间匹配版）
            if 'avg_deviation' in stats:
                f.write(f"\n路径跟踪精度（空间匹配算法）:\n")
                f.write(f"  平均偏差: {stats['avg_deviation']:.3f} m\n")
                f.write(f"  中位数偏差: {stats['median_deviation']:.3f} m\n")
                f.write(f"  最大偏差: {stats['max_deviation']:.3f} m\n")
                f.write(f"  95%分位数: {stats['p95_deviation']:.3f} m\n")
                f.write(f"  99%分位数: {stats['p99_deviation']:.3f} m\n")
                f.write(f"  偏差标准差: {stats['std_deviation']:.3f} m\n")
                f.write(f"  偏差变异系数: {stats['cv_deviation']:.3f}\n")
                f.write(f"  偏差质量评估: {stats.get('deviation_quality', '未知')}\n")
            
            # 起点终点误差
            if 'start_error' in stats and 'end_error' in stats:
                f.write(f"\n起点终点误差:\n")
                f.write(f"  起点误差: {stats['start_error']:.3f} m\n")
                f.write(f"  终点误差: {stats['end_error']:.3f} m\n")
            
            # 空间匹配算法说明
            f.write(f"\n空间匹配算法说明:\n")
            f.write(f"  - 完全忽略时间戳，只考虑空间位置\n")
            f.write(f"  - 为每个实际路径点计算到规划折线(线段)的最短距离\n")
            f.write(f"  - 使用欧几里得距离（点到线段距离）计算偏差\n")
            f.write(f"  - 适用于时间戳不同步或不可靠的情况\n")
            f.write(f"  - 异常值过滤相对宽松，保留更多有效数据\n")
            f.write(f"\n与时间匹配算法的区别:\n")
            f.write(f"  - 时间匹配: 基于时间戳对齐，要求时间同步\n")
            f.write(f"  - 空间匹配: 基于空间位置匹配，忽略时间因素\n")
            f.write(f"  - 空间匹配通常能获得更稳定的结果\n")
            f.write(f"  - 空间匹配更适合分析路径形状的相似性\n")
            
            f.write("\n" + "=" * 60 + "\n")
            f.write("报告结束\n")
            f.write("=" * 60 + "\n")
        
        print(f"空间匹配版分析报告已保存到: {output_file}")
    
    def create_spatial_visualization(self, planned_csv, actual_csv, stats=None):
        """创建空间匹配版可视化图表（使用与报告相同的偏差数据）"""
        print("正在创建空间匹配版可视化图表...")
        
        try:
            planned_df = pd.read_csv(planned_csv)
            actual_df = pd.read_csv(actual_csv)
        except Exception as e:
            print(f"读取CSV文件失败: {e}")
            return
        
        # 应用滤波器
        filtered_actual_df = self.apply_filters(actual_df, filter_type='savgol', window_length=11, polyorder=3)
        if len(planned_df) > 20:
            filtered_planned_df = self.apply_filters(planned_df, filter_type='moving_avg', window_length=5)
        else:
            filtered_planned_df = planned_df.copy()

        # 应用静态坐标变换补偿雷达到机器人中心的偏移（与analyze_path_comparison_spatial保持一致）
        print("可视化：应用静态坐标变换补偿雷达到机器人中心的偏移...")
        x_offset = -0.75  # x方向偏移：雷达在机器人中心后面0.75米，所以需要减去0.75米（机器人中心在雷达前面）
        y_offset = 0.0   # y方向无偏移

        # 应用平移变换到实际路径数据
        if 'x_filtered' in filtered_actual_df.columns:
            filtered_actual_df['x_filtered'] = filtered_actual_df['x_filtered'] + x_offset
        elif 'x' in filtered_actual_df.columns:
            filtered_actual_df['x'] = filtered_actual_df['x'] + x_offset

        if 'y_filtered' in filtered_actual_df.columns:
            filtered_actual_df['y_filtered'] = filtered_actual_df['y_filtered'] + y_offset
        elif 'y' in filtered_actual_df.columns:
            filtered_actual_df['y'] = filtered_actual_df['y'] + y_offset

        print(f"可视化：应用静态变换: x += {x_offset}, y += {y_offset}")

        # 构建规划路径分段（按"每次plan消息"分组），用于逐点导航/重规划场景下的动态匹配
        plan_segments = []
        if len(filtered_planned_df) > 0 and 'timestamp' in filtered_planned_df.columns:
            x_plan_col = 'x_filtered' if 'x_filtered' in filtered_planned_df.columns else 'x'
            y_plan_col = 'y_filtered' if 'y_filtered' in filtered_planned_df.columns else 'y'
            group_cols = ['timestamp']
            if 'topic' in filtered_planned_df.columns:
                group_cols.append('topic')
            if 'path_id' in filtered_planned_df.columns:
                group_cols.append('path_id')

            for keys, g in filtered_planned_df.groupby(group_cols):
                g2 = g.sort_values('point_index' if 'point_index' in g.columns else 'timestamp')
                if len(g2) < 2:
                    continue
                seg_start_t = float(g2['timestamp'].min())
                poly_raw = g2[[x_plan_col, y_plan_col]].values
                is_two_point_line = len(poly_raw) == 2
                poly = self._densify_polyline(poly_raw, resolution=0.05)
                pid = int(g2['path_id'].iloc[0]) if 'path_id' in g2.columns else -1
                topic = str(g2['topic'].iloc[0]) if 'topic' in g2.columns else ''
                a0 = poly_raw[0].copy()
                b0 = poly_raw[1].copy()
                plan_segments.append((seg_start_t, pid, topic, poly, is_two_point_line, a0, b0))
            plan_segments.sort(key=lambda x: x[0])

        seg_times = np.array([s[0] for s in plan_segments], dtype=float) if len(plan_segments) > 0 else None
        
        fig, axes = plt.subplots(3, 2, figsize=(18, 16))
        fig.suptitle('空间匹配版导航路径分析', fontsize=16)
        
        # 1. 路径对比图（原始数据）
        ax1 = axes[0, 0]
        
        if len(planned_df) > 0 and len(actual_df) > 0:
            actual_start_time = actual_df['timestamp'].min() if 'timestamp' in actual_df.columns else 0
            planned_paths_by_time = planned_df.groupby('path_id')['timestamp'].min().reset_index()
            planned_paths_by_time['time_diff'] = abs(planned_paths_by_time['timestamp'] - actual_start_time)
            
            if len(planned_paths_by_time) > 0:
                closest_path = planned_paths_by_time.loc[planned_paths_by_time['time_diff'].idxmin()]
                closest_path_id = closest_path['path_id']
                closest_plan = planned_df[planned_df['path_id'] == closest_path_id].sort_values('point_index')
                
                if len(closest_plan) > 0:
                    ax1.plot(closest_plan['x'], closest_plan['y'], 'r-', linewidth=2, alpha=0.7, label='规划路径')
                    ax1.scatter(closest_plan['x'].iloc[0], closest_plan['y'].iloc[0], 
                               c='green', s=150, marker='o', label='规划起点', zorder=5)
                    ax1.scatter(closest_plan['x'].iloc[-1], closest_plan['y'].iloc[-1], 
                               c='red', s=150, marker='*', label='规划终点', zorder=5)
        
        if len(actual_df) > 0:
            ax1.plot(actual_df['x'], actual_df['y'], 'b-', linewidth=2, label='实际路径（原始）', alpha=0.7)
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
            x_plan = 'x_filtered' if 'x_filtered' in filtered_planned_df.columns else 'x'
            y_plan = 'y_filtered' if 'y_filtered' in filtered_planned_df.columns else 'y'
            ax2.plot(filtered_planned_df[x_plan], filtered_planned_df[y_plan], 'r-', linewidth=2, alpha=0.7, label='规划路径（滤波后）')
            
            x_actual = 'x_filtered' if 'x_filtered' in filtered_actual_df.columns else 'x'
            y_actual = 'y_filtered' if 'y_filtered' in filtered_actual_df.columns else 'y'
            ax2.plot(filtered_actual_df[x_actual], filtered_actual_df[y_actual], 'b-', linewidth=2, label='实际路径（滤波后）', alpha=0.7)
            
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
        
            # 3. 空间匹配路径偏差图（使用与报告相同的简化过滤逻辑）
        ax3 = axes[1, 0]
        deviations = []  # 初始化为空列表
        times = []
        actual_xy_for_dev = []
        
        if len(filtered_planned_df) > 0 and len(filtered_actual_df) > 0:
            x_plan = 'x_filtered' if 'x_filtered' in filtered_planned_df.columns else 'x'
            y_plan = 'y_filtered' if 'y_filtered' in filtered_planned_df.columns else 'y'
            x_actual = 'x_filtered' if 'x_filtered' in filtered_actual_df.columns else 'x'
            y_actual = 'y_filtered' if 'y_filtered' in filtered_actual_df.columns else 'y'
            
            # 空间匹配算法（逐点导航/重规划时动态选择当前规划段；否则退化为使用全部规划折线）
            fallback_poly = self._densify_polyline(filtered_planned_df[[x_plan, y_plan]].values, resolution=0.05)
            
            # 为了提高效率，对实际路径进行下采样
            if len(filtered_actual_df) > 2000:
                sample_rate = len(filtered_actual_df) // 2000
                actual_points = filtered_actual_df.iloc[::sample_rate]
            else:
                actual_points = filtered_actual_df
            
            for _, actual_point in actual_points.iterrows():
                actual_pos = np.array([actual_point[x_actual], actual_point[y_actual]])
                if len(plan_segments) > 0 and 'timestamp' in filtered_actual_df.columns:
                    t_abs = float(actual_point['timestamp'])
                    idx = int(np.searchsorted(seg_times, t_abs, side='right') - 1)
                    if idx < 0:
                        idx = 0
                    plan_poly = plan_segments[idx][3]
                    is_line2 = plan_segments[idx][4]
                    a0 = plan_segments[idx][5]
                    b0 = plan_segments[idx][6]
                else:
                    plan_poly = fallback_poly
                    is_line2 = False
                    a0 = None
                    b0 = None

                if is_line2 and a0 is not None and b0 is not None:
                    t_seg = self._line_segment_projection_t(actual_pos, a0, b0)
                    in_seg = (t_seg >= 0.0) and (t_seg <= 1.0)
                    dev = self._point_to_infinite_line_distance(actual_pos, a0, b0)
                    deviations.append(dev if in_seg else float('nan'))
                else:
                    deviations.append(self._point_to_polyline_distance(actual_pos, plan_poly))
                actual_xy_for_dev.append((float(actual_point[x_actual]), float(actual_point[y_actual])))
                
                if 'timestamp' in filtered_actual_df.columns:
                    times.append(actual_point['timestamp'] - filtered_actual_df['timestamp'].min())
                else:
                    times.append(len(times))
            
            # 使用与报告相同的简化过滤（仅去掉明显极端值）
            if len(deviations) > 0:
                deviations = np.array(deviations, dtype=float)
                times = np.array(times)
                max_reasonable_deviation = 20.0
                finite_mask = np.isfinite(deviations)
                deviations = deviations[finite_mask]
                times = times[finite_mask]
                valid_mask = deviations < max_reasonable_deviation
                filtered_deviations = deviations[valid_mask]
                filtered_times = times[valid_mask]
                
                if len(filtered_deviations) == 0:
                    filtered_deviations = deviations
                    filtered_times = times
                    print("图表过滤 - 所有偏差都超过极端阈值，直接使用原始偏差")
                
                print(f"图表过滤 - 原始偏差数量: {len(deviations)}")
                print(f"图表过滤 - 极端值阈值: {max_reasonable_deviation:.1f} m")
                print(f"图表过滤 - 过滤后偏差数量: {len(filtered_deviations)}")
                
                if len(filtered_deviations) > 0:
                    ax3.plot(filtered_times, filtered_deviations, 'g-', linewidth=2, 
                            label='路径偏差（空间匹配，已过滤）')
                    ax3.fill_between(filtered_times, 0, filtered_deviations, alpha=0.3, color='green')

                    # 标记“突增点”（用于快速定位问题时刻）
                    try:
                        med = float(np.median(filtered_deviations))
                        mad = float(np.median(np.abs(filtered_deviations - med))) + 1e-9
                        spike_th = med + 6.0 * mad  # 很保守：只标非常突出的点
                        spike_idx = np.where(filtered_deviations >= spike_th)[0]
                        if len(spike_idx) > 0:
                            ax3.scatter(filtered_times[spike_idx], filtered_deviations[spike_idx],
                                       c='red', s=18, alpha=0.8, label=f'突增点(>{spike_th:.2f}m)')
                    except Exception:
                        pass
                    
                    # 使用与报告相同的统计值
                    avg_dev = np.mean(filtered_deviations)
                    median_dev = np.median(filtered_deviations)
                    p95_dev = np.percentile(filtered_deviations, 95)
                    
                    ax3.axhline(y=avg_dev, color='red', linestyle='--', 
                               label=f'平均偏差: {avg_dev:.3f} m')
                    ax3.axhline(y=median_dev, color='orange', linestyle='--', 
                               label=f'中位数偏差: {median_dev:.3f} m')
                    ax3.axhline(y=p95_dev, color='purple', linestyle='--', 
                               label=f'95%分位数: {p95_dev:.3f} m')
                    
                    ax3.legend()
        
        ax3.set_xlabel('点索引' if 'timestamp' not in filtered_actual_df.columns else '时间 (s)')
        ax3.set_ylabel('路径偏差 (m)')
        ax3.set_title('空间匹配路径偏差')
        ax3.grid(True, alpha=0.3)
        
        # 4. 偏差分布直方图（使用与报告一致的简化过滤）
        ax4 = axes[1, 1]
        if len(deviations) > 0:
            deviations = np.array(deviations)
            max_reasonable_deviation = 20.0
            valid_mask = deviations < max_reasonable_deviation
            filtered_deviations = deviations[valid_mask]
            
            if len(filtered_deviations) == 0:
                filtered_deviations = deviations
            
            ax4.hist(filtered_deviations, bins=50, alpha=0.7, color='green', edgecolor='black')
            ax4.axvline(x=np.mean(filtered_deviations), color='red', linestyle='--', 
                       label=f'平均值: {np.mean(filtered_deviations):.3f} m')
            ax4.axvline(x=np.median(filtered_deviations), color='orange', linestyle='--', 
                       label=f'中位数: {np.median(filtered_deviations):.3f} m')
            ax4.set_xlabel('偏差 (m)')
            ax4.set_ylabel('频次')
            ax4.set_title('偏差分布直方图（空间匹配，已过滤）')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
        
        # 5. X位置随时间变化
        ax5 = axes[2, 0]
        if 'timestamp' in planned_df.columns:
            planned_time = planned_df['timestamp'] - planned_df['timestamp'].min()
            ax5.plot(planned_time, planned_df['x'], 'r-', linewidth=2, label='规划路径X', alpha=0.7)
        
        if 'timestamp' in actual_df.columns:
            actual_time = actual_df['timestamp'] - actual_df['timestamp'].min()
            ax5.plot(actual_time, actual_df['x'], 'b-', linewidth=2, label='实际路径X（原始）', alpha=0.7)
            
            if 'x_filtered' in filtered_actual_df.columns:
                ax5.plot(actual_time, filtered_actual_df['x_filtered'], 'b--', linewidth=2, label='实际路径X（滤波后）', alpha=0.9)
        
        ax5.set_xlabel('时间 (s)')
        ax5.set_ylabel('X 位置 (m)')
        ax5.set_title('X位置随时间变化')
        ax5.grid(True, alpha=0.3)
        ax5.legend()
        
        # 6. Y位置随时间变化
        ax6 = axes[2, 1]
        if 'timestamp' in planned_df.columns:
            ax6.plot(planned_time, planned_df['y'], 'r-', linewidth=2, label='规划路径Y', alpha=0.7)
        
        if 'timestamp' in actual_df.columns:
            ax6.plot(actual_time, actual_df['y'], 'b-', linewidth=2, label='实际路径Y（原始）', alpha=0.7)
            
            if 'y_filtered' in filtered_actual_df.columns:
                ax6.plot(actual_time, filtered_actual_df['y_filtered'], 'b--', linewidth=2, label='实际路径Y（滤波后）', alpha=0.9)
        
        ax6.set_xlabel('时间 (s)')
        ax6.set_ylabel('Y 位置 (m)')
        ax6.set_title('Y位置随时间变化')
        ax6.grid(True, alpha=0.3)
        ax6.legend()
        
        plt.tight_layout()
        
        # 保存图表
        base_name = os.path.splitext(os.path.basename(self.bag_file))[0]
        plot_file = os.path.join(self.output_dir, f"{base_name}_analysis_plot_spatial.png")
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"空间匹配版可视化图表已保存到: {plot_file}")
        
        return plot_file

def main():
    parser = argparse.ArgumentParser(description='空间匹配版导航数据分析（忽略时间戳）')
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
    
    # 创建空间匹配版分析器
    analyzer = SpatialMatchingNavigationAnalyzer(args.bag_file, args.output_dir)
    
    # 提取数据
    if not analyzer.extract_data():
        print("数据提取失败")
        sys.exit(1)
    
    # 保存为CSV
    result_files = analyzer.save_to_csv()
    
    if args.extract_only:
        print("数据提取完成")
        return
    
    # 进行空间匹配版分析
    if 'global_plan' in result_files:
        planned_csv = result_files['global_plan']
        
        actual_csv = None
        for key in ['localization', 'odometry']:
            if key in result_files:
                actual_csv = result_files[key]
                break
        
        if actual_csv:
            # 使用空间匹配版分析
            stats = analyzer.analyze_path_comparison_spatial(planned_csv, actual_csv)
            
            if stats:
                # 生成空间匹配版报告
                base_name = os.path.splitext(os.path.basename(args.bag_file))[0]
                report_file = os.path.join(args.output_dir, f"{base_name}_analysis_report_spatial.txt")
                analyzer.generate_spatial_report(stats, report_file)
                
                # 创建空间匹配版可视化（传递stats参数以确保一致性）
                plot_file = analyzer.create_spatial_visualization(planned_csv, actual_csv, stats)
                
                print(f"\n空间匹配版分析完成!")
                print(f"数据文件保存在: {args.output_dir}")
                print(f"空间匹配版分析报告: {report_file}")
                if plot_file:
                    print(f"空间匹配版可视化图表: {plot_file}")
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

if __name__ == "__main__":
    main()