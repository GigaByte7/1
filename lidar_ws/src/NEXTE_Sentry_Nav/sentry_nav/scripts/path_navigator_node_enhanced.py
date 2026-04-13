#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import json
import tf
import math
import re
import threading

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sentry_nav.msg import PathNavigationAction, PathNavigationGoal, PathNavigationResult, PathNavigationFeedback
from sentry_nav.srv import SetPathGoal, SetPathGoalResponse, PauseNavigation, PauseNavigationResponse

def strip_comments(json_str):
    """移除JSON字符串中的单行和多行注释"""
    # 移除多行注释 /* ... */
    json_str = re.sub(r'/\*.*?\*/', '', json_str, flags=re.DOTALL)
    # 移除单行注释 // ...
    json_str = re.sub(r'//.*?$', '', json_str, flags=re.MULTILINE)
    return json_str

def load_nav_points(file_path):
    """从JSON文件中加载导航点，支持带注释的JSON"""
    try:
        with open(file_path, 'r') as f:
            content = f.read()
            # 移除注释
            content = strip_comments(content)
            data = json.loads(content)
            return data
    except Exception as e:
        rospy.logerr("Failed to load or parse JSON file: %s", e)
        return None

class PathNavigatorEnhanced:
    def __init__(self):
        rospy.init_node('path_navigator_node_enhanced')
        
        # 从参数服务器获取参数
        json_file_path = rospy.get_param('~json_file_path', 'nav_points.json')
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'body_foot')
        self.dead_zone_radius = rospy.get_param('~dead_zone_radius', 0.06)
        
        rospy.loginfo("Path Navigator Enhanced 参数:")
        rospy.loginfo("  JSON文件: %s", json_file_path)
        rospy.loginfo("  世界坐标系: %s", self.world_frame)
        rospy.loginfo("  机器人坐标系: %s", self.robot_frame)
        rospy.loginfo("  死区半径: %.3f m", self.dead_zone_radius)
        
        self.nav_point_groups = load_nav_points(json_file_path)

        if not self.nav_point_groups:
            rospy.logerr("Failed to load navigation points. Shutting down.")
            return

        # TF监听器，用于获取机器人位置
        self.tf_listener = tf.TransformListener()
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # 添加完整路径发布器
        self.path_pub = rospy.Publisher('/path_navigator/full_path', Path, queue_size=1, latch=True)
        
        # 添加可视化标记发布器
        self.path_marker_pub = rospy.Publisher('/path_navigator/path_marker', Marker, queue_size=1, latch=True)
        self.marker_array_pub = rospy.Publisher('/path_navigator/marker_array', MarkerArray, queue_size=1, latch=True)
        
        # 添加截断路径发布器
        self.truncated_path_pub = rospy.Publisher('/path_navigator/truncated_path', Path, queue_size=1, latch=True)
        self.truncated_marker_pub = rospy.Publisher('/path_navigator/truncated_marker', Marker, queue_size=1, latch=True)
        
        # 订阅RViz的2D Nav Goal话题
        self.rviz_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        
        # 动态目标点服务
        self.dynamic_goal_service = rospy.Service('/set_path_goal', SetPathGoal, self.handle_set_path_goal)
        
        # 状态变量
        self.dynamic_goal = None
        self.dynamic_goal_event = threading.Event()
        self.current_path_group_name = None
        self.truncated_waypoints = None
        
        # 暂停功能相关变量
        self.is_paused = False
        self.pause_lock = threading.Lock()
        
        # 创建Action服务器
        self._action_server = actionlib.SimpleActionServer(
            '/track_points', 
            PathNavigationAction, 
            execute_cb=self.execute_cb, 
            auto_start=False
        )
        self._action_server.start()
        
        # 创建暂停导航服务
        self.pause_service = rospy.Service('/pause_navigation', PauseNavigation, self.handle_pause_navigation)

        rospy.loginfo("Path Navigation Action Server (Enhanced) is ready. 等待Action目标...")
        rospy.loginfo("支持动态目标点截断功能：")
        rospy.loginfo("  1. 发送Action目标到 /track_points")
        rospy.loginfo("  2. 在RViz中使用2D Nav Goal设置新目标点")
        rospy.loginfo("  3. 机器人将从路径起点导航到最近的路径点（截断点）")
        rospy.loginfo("路径可视化话题:")
        rospy.loginfo("  - 原始路径: /path_navigator/full_path")
        rospy.loginfo("  - 截断路径: /path_navigator/truncated_path")
        rospy.loginfo("  - 标记数组: /path_navigator/marker_array")
        rospy.loginfo("  - 截断标记: /path_navigator/truncated_marker")

    def rviz_goal_callback(self, msg):
        """处理RViz中的2D Nav Goal"""
        rospy.loginfo("收到RViz 2D Nav Goal: (%.3f, %.3f, %.3f)", 
                     msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.dynamic_goal = msg
        self.dynamic_goal_event.set()
        rospy.loginfo("动态目标点已设置，等待导航开始...")

    def handle_set_path_goal(self, req):
        """处理动态目标点服务请求"""
        rospy.loginfo("收到SetPathGoal服务请求")
        self.dynamic_goal = req.dynamic_goal
        self.dynamic_goal_event.set()
        return SetPathGoalResponse(success=True, message="动态目标点已接收")
    
    def handle_pause_navigation(self, req):
        """处理暂停/继续导航服务请求"""
        with self.pause_lock:
            if req.pause:
                if not self.is_paused:
                    self.is_paused = True
                    rospy.loginfo("导航暂停服务调用: 暂停导航")
                    return PauseNavigationResponse(success=True, message="导航已暂停")
                else:
                    rospy.logwarn("导航已处于暂停状态")
                    return PauseNavigationResponse(success=False, message="导航已处于暂停状态")
            else:
                if self.is_paused:
                    self.is_paused = False
                    rospy.loginfo("导航暂停服务调用: 继续导航")
                    return PauseNavigationResponse(success=True, message="导航已继续")
                else:
                    rospy.logwarn("导航已处于运行状态")
                    return PauseNavigationResponse(success=False, message="导航已处于运行状态")

    def find_nearest_waypoint(self, waypoints, target_pose):
        """在路径点中找到距离目标点最近的点"""
        if not waypoints:
            return -1
        
        min_distance = float('inf')
        nearest_index = -1
        
        for i, waypoint_data in enumerate(waypoints):
            dx = waypoint_data['position']['x'] - target_pose.pose.position.x
            dy = waypoint_data['position']['y'] - target_pose.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < min_distance:
                min_distance = distance
                nearest_index = i
        
        rospy.loginfo("找到最近路径点索引: %d, 距离: %.3f 米", nearest_index, min_distance)
        return nearest_index

    def execute_cb(self, goal):
        # 使用Action目标中的dead_zone_radius，如果没有则使用节点参数
        dead_zone_radius = goal.dead_zone_radius if goal.dead_zone_radius > 0 else self.dead_zone_radius
        
        rospy.loginfo("收到导航目标: 路径组 '%s', 死区半径: %.3f", 
                      goal.path_group_name, dead_zone_radius)

        # 1. 验证路径组是否存在
        if goal.path_group_name not in self.nav_point_groups:
            rospy.logerr("路径组 '%s' 在JSON文件中不存在.", goal.path_group_name)
            self._action_server.set_aborted(result=PathNavigationResult(success=False, message="Path group not found."))
            return
        
        original_waypoints = self.nav_point_groups[goal.path_group_name]
        rospy.loginfo("原始路径包含 %d 个路径点.", len(original_waypoints))

        # 2. 发布完整原始路径到RViz
        self.publish_full_path(original_waypoints, goal.path_group_name)

        # 3. 等待动态目标点（来自RViz的2D Nav Goal）
        rospy.loginfo("等待动态目标点... 请在RViz中使用2D Nav Goal工具设置新目标点")
        rospy.loginfo("或者通过服务调用设置: rosservice call /set_path_goal")
        
        # 重置事件和动态目标
        self.dynamic_goal = None
        self.dynamic_goal_event.clear()
        self.current_path_group_name = goal.path_group_name
        
        # 等待动态目标点或Action被取消
        while not rospy.is_shutdown() and not self.dynamic_goal_event.is_set():
            if self._action_server.is_preempt_requested():
                rospy.loginfo("导航被客户端取消.")
                self._action_server.set_preempted()
                return
            rospy.sleep(0.1)
        
        # 检查是否有动态目标点
        if self.dynamic_goal is None:
            rospy.logerr("未收到动态目标点，导航取消.")
            self._action_server.set_aborted(result=PathNavigationResult(success=False, message="No dynamic goal received."))
            return
        
        rospy.loginfo("已收到动态目标点，计算截断路径...")
        
        # 4. 找到最近的路径点
        nearest_index = self.find_nearest_waypoint(original_waypoints, self.dynamic_goal)
        
        if nearest_index < 0:
            rospy.logerr("无法找到最近路径点，导航取消.")
            self._action_server.set_aborted(result=PathNavigationResult(success=False, message="Failed to find nearest waypoint."))
            return
        
        # 截断路径：从起点到最近路径点（包含该点）
        self.truncated_waypoints = original_waypoints[:nearest_index + 1]
        rospy.loginfo("路径截断: 原始 %d 个点 -> 截断 %d 个点", 
                     len(original_waypoints), len(self.truncated_waypoints))
        
        # 5. 发布截断路径到RViz
        self.publish_truncated_path(self.truncated_waypoints, goal.path_group_name)
        
        # 6. 循环遍历截断后的路径点
        for i, waypoint_data in enumerate(self.truncated_waypoints):
            # 检查Action是否被客户端取消
            if self._action_server.is_preempt_requested():
                rospy.loginfo("导航被客户端取消.")
                self._action_server.set_preempted()
                return

            # 创建并发布目标点
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = self.world_frame
            
            goal_pose.pose.position.x = waypoint_data['position']['x']
            goal_pose.pose.position.y = waypoint_data['position']['y']
            goal_pose.pose.position.z = waypoint_data['position']['z']
            goal_pose.pose.orientation.x = waypoint_data['orientation']['x']
            goal_pose.pose.orientation.y = waypoint_data['orientation']['y']
            goal_pose.pose.orientation.z = waypoint_data['orientation']['z']
            goal_pose.pose.orientation.w = waypoint_data['orientation']['w']
            
            rospy.loginfo("发布路径点 %d/%d: (%.3f, %.3f, %.3f)", 
                         i + 1, len(self.truncated_waypoints),
                         goal_pose.pose.position.x,
                         goal_pose.pose.position.y,
                         goal_pose.pose.position.z)
            
            self.goal_pub.publish(goal_pose)
            
            # 发布反馈
            feedback = PathNavigationFeedback()
            feedback.current_waypoint_info = "导航到路径点 %d/%d (截断路径组 '%s')" % (i + 1, len(self.truncated_waypoints), goal.path_group_name)
            self._action_server.publish_feedback(feedback)
            rospy.loginfo(feedback.current_waypoint_info)

            # 等待机器人到达死区
            arrived = self.wait_for_arrival(goal_pose, dead_zone_radius)
            
            # 如果在等待时被抢占，则退出
            if self._action_server.is_preempt_requested():
                rospy.loginfo("等待到达时被客户端取消.")
                self._action_server.set_preempted()
                return
            
            if arrived:
                rospy.loginfo("路径点 %d 到达成功.", i + 1)
            else:
                rospy.logwarn("路径点 %d 在超时时间内未到达，继续下一个路径点.", i + 1)

        # 7. 所有点都尝试完成
        result = PathNavigationResult(success=True, message="导航成功完成（截断路径）。")
        self._action_server.set_succeeded(result)
        rospy.loginfo("路径组 '%s' 截断导航完成.", goal.path_group_name)
        
        # 8. 清理状态
        self.dynamic_goal = None
        self.dynamic_goal_event.clear()
        self.truncated_waypoints = None

    def wait_for_arrival(self, target_pose, radius, timeout_seconds=60.0):
        """等待机器人到达目标点指定的半径内（支持暂停功能）"""
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.get_time()
        arrived = False
        
        # 记录初始距离
        initial_distance = None
        last_log_time = start_time
        
        # 暂停相关变量
        pause_start_time = None
        total_pause_duration = 0.0
        
        while not rospy.is_shutdown() and not arrived:
            current_time = rospy.get_time()
            
            # 检查暂停状态
            with self.pause_lock:
                is_paused = self.is_paused
            
            # 处理暂停状态
            if is_paused:
                if pause_start_time is None:
                    pause_start_time = current_time
                    rospy.loginfo("导航已暂停，等待继续...")
                
                # 发布零速度命令（通过goal_pub发布停止目标）
                stop_pose = PoseStamped()
                stop_pose.header.stamp = rospy.Time.now()
                stop_pose.header.frame_id = self.world_frame
                stop_pose.pose = target_pose.pose
                # 发布当前位置作为目标点（机器人会停止）
                self.goal_pub.publish(stop_pose)
                
                # 检查是否继续
                rate.sleep()
                continue
            else:
                # 如果刚从暂停状态恢复
                if pause_start_time is not None:
                    pause_end_time = current_time
                    pause_duration = pause_end_time - pause_start_time
                    total_pause_duration += pause_duration
                    rospy.loginfo("导航继续，暂停时间: %.1f 秒", pause_duration)
                    pause_start_time = None
                    # 重新发布目标点
                    self.goal_pub.publish(target_pose)
            
            # 调整超时时间（排除暂停时间）
            adjusted_current_time = current_time - total_pause_duration
            adjusted_start_time = start_time
            
            # 检查超时（排除暂停时间）
            if adjusted_current_time - adjusted_start_time > timeout_seconds:
                rospy.logwarn("到达目标点超时 (%.1f 秒，排除暂停时间).", timeout_seconds)
                break
            
            # 检查Action是否被客户端取消
            if self._action_server.is_preempt_requested():
                rospy.loginfo("等待到达时被客户端取消.")
                break
            
            try:
                # 获取机器人位置
                (trans, rot) = self.tf_listener.lookupTransform(
                    self.world_frame, self.robot_frame, rospy.Time(0))
                
                # 计算2D距离
                dx = trans[0] - target_pose.pose.position.x
                dy = trans[1] - target_pose.pose.position.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # 记录初始距离
                if initial_distance is None:
                    initial_distance = distance
                    rospy.loginfo("初始距离到目标点: %.3f 米", initial_distance)
                
                # 每5秒记录一次进度
                if current_time - last_log_time > 5.0:
                    remaining_time = timeout_seconds - (adjusted_current_time - adjusted_start_time)
                    rospy.loginfo("导航中... 距离: %.3f 米, 容差: %.3f 米, 剩余时间: %.1f 秒, 暂停状态: %s", 
                                 distance, radius, remaining_time, "是" if is_paused else "否")
                    last_log_time = current_time
                
                # 检查是否到达
                if distance < radius:
                    rospy.loginfo("到达目标点! 最终距离: %.3f 米 (小于容差 %.3f 米)", distance, radius)
                    arrived = True
                    break
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn_throttle(5.0, "TF异常: %s. 重试中...", e)
                # 等待一下再重试
                rospy.sleep(0.1)
                continue
            
            rate.sleep()
        
        return arrived

    def get_robot_position(self):
        """获取机器人在世界坐标系中的位置"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.world_frame, self.robot_frame, rospy.Time(0))
            return trans[0], trans[1], trans[2]
        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException) as e:
            rospy.logwarn("TF异常: %s", e)
            return None, None, None

    def publish_full_path(self, waypoints, path_group_name):
        """发布完整路径到话题 /path_navigator/full_path 以及可视化标记"""
        if not waypoints:
            rospy.logwarn("路径组 '%s' 为空，不发布路径", path_group_name)
            return
        
        # 1. 发布nav_msgs/Path消息
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.world_frame
        
        for waypoint_data in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.world_frame
            
            pose_stamped.pose.position.x = waypoint_data['position']['x']
            pose_stamped.pose.position.y = waypoint_data['position']['y']
            pose_stamped.pose.position.z = waypoint_data['position']['z']
            pose_stamped.pose.orientation.x = waypoint_data['orientation']['x']
            pose_stamped.pose.orientation.y = waypoint_data['orientation']['y']
            pose_stamped.pose.orientation.z = waypoint_data['orientation']['z']
            pose_stamped.pose.orientation.w = waypoint_data['orientation']['w']
            
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
        rospy.loginfo("已发布完整原始路径 '%s' 到话题 /path_navigator/full_path，共 %d 个点", 
                     path_group_name, len(waypoints))
        
        # 2. 发布Marker用于可视化（线条）
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_line_" + path_group_name
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # 线宽
        marker.color.a = 0.6   # 透明度
        marker.color.r = 0.0   # 红色分量
        marker.color.g = 1.0   # 绿色分量
        marker.color.b = 0.0   # 蓝色分量
        
        for waypoint_data in waypoints:
            p = Point()
            p.x = waypoint_data['position']['x']
            p.y = waypoint_data['position']['y']
            p.z = waypoint_data['position']['z']
            marker.points.append(p)
        
        self.path_marker_pub.publish(marker)
        
        # 3. 发布MarkerArray（包含路径点和线条）
        marker_array = MarkerArray()
        
        # 添加路径线
        marker_array.markers.append(marker)
        
        # 添加起点和终点标记
        if len(waypoints) > 0:
            # 起点标记
            start_marker = Marker()
            start_marker.header.frame_id = self.world_frame
            start_marker.header.stamp = rospy.Time.now()
            start_marker.ns = "path_points_" + path_group_name
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position.x = waypoints[0]['position']['x']
            start_marker.pose.position.y = waypoints[0]['position']['y']
            start_marker.pose.position.z = waypoints[0]['position']['z']
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = 0.15
            start_marker.scale.y = 0.15
            start_marker.scale.z = 0.15
            start_marker.color.a = 1.0
            start_marker.color.r = 1.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0
            marker_array.markers.append(start_marker)
            
            # 终点标记
            if len(waypoints) > 1:
                end_marker = Marker()
                end_marker.header.frame_id = self.world_frame
                end_marker.header.stamp = rospy.Time.now()
                end_marker.ns = "path_points_" + path_group_name
                end_marker.id = 2
                end_marker.type = Marker.SPHERE
                end_marker.action = Marker.ADD
                end_marker.pose.position.x = waypoints[-1]['position']['x']
                end_marker.pose.position.y = waypoints[-1]['position']['y']
                end_marker.pose.position.z = waypoints[-1]['position']['z']
                end_marker.pose.orientation.w = 1.0
                end_marker.scale.x = 0.15
                end_marker.scale.y = 0.15
                end_marker.scale.z = 0.15
                end_marker.color.a = 1.0
                end_marker.color.r = 1.0
                end_marker.color.g = 0.0
                end_marker.color.b = 0.0
                marker_array.markers.append(end_marker)
            
            # 添加中间路径点标记（除了起点和终点）
            for i in range(1, len(waypoints) - 1):
                point_marker = Marker()
                point_marker.header.frame_id = self.world_frame
                point_marker.header.stamp = rospy.Time.now()
                point_marker.ns = "path_points_" + path_group_name
                point_marker.id = 3 + i  # 从3开始，避免与起点和终点冲突
                point_marker.type = Marker.CUBE
                point_marker.action = Marker.ADD
                point_marker.pose.position.x = waypoints[i]['position']['x']
                point_marker.pose.position.y = waypoints[i]['position']['y']
                point_marker.pose.position.z = waypoints[i]['position']['z']
                point_marker.pose.orientation.w = 1.0
                point_marker.scale.x = 0.25
                point_marker.scale.y = 0.25
                point_marker.scale.z = 0.25
                point_marker.color.a = 1.0
                point_marker.color.r = 0.0
                point_marker.color.g = 0.0
                point_marker.color.b = 0.8
                marker_array.markers.append(point_marker)
        
        self.marker_array_pub.publish(marker_array)
        rospy.loginfo("已发布路径标记数组 '%s' 到话题 /path_navigator/marker_array，共 %d 个标记", 
                     path_group_name, len(marker_array.markers))

    def publish_truncated_path(self, waypoints, path_group_name):
        """发布截断路径到话题 /path_navigator/truncated_path 以及可视化标记"""
        if not waypoints:
            rospy.logwarn("截断路径组 '%s' 为空，不发布路径", path_group_name)
            return
        
        # 1. 发布截断路径消息
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.world_frame
        
        for waypoint_data in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.world_frame
            
            pose_stamped.pose.position.x = waypoint_data['position']['x']
            pose_stamped.pose.position.y = waypoint_data['position']['y']
            pose_stamped.pose.position.z = waypoint_data['position']['z']
            pose_stamped.pose.orientation.x = waypoint_data['orientation']['x']
            pose_stamped.pose.orientation.y = waypoint_data['orientation']['y']
            pose_stamped.pose.orientation.z = waypoint_data['orientation']['z']
            pose_stamped.pose.orientation.w = waypoint_data['orientation']['w']
            
            path_msg.poses.append(pose_stamped)
        
        self.truncated_path_pub.publish(path_msg)
        rospy.loginfo("已发布截断路径 '%s' 到话题 /path_navigator/truncated_path，共 %d 个点", 
                     path_group_name, len(waypoints))
        
        # 2. 发布截断路径标记（不同颜色）
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "truncated_line_" + path_group_name
        marker.id = 100  # 使用不同的ID避免冲突
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08  # 更宽的线宽
        marker.color.a = 0.8   # 更高的透明度
        marker.color.r = 1.0   # 红色分量（区分原始路径）
        marker.color.g = 0.5   # 绿色分量
        marker.color.b = 0.0   # 蓝色分量
        
        for waypoint_data in waypoints:
            p = Point()
            p.x = waypoint_data['position']['x']
            p.y = waypoint_data['position']['y']
            p.z = waypoint_data['position']['z']
            marker.points.append(p)
        
        self.truncated_marker_pub.publish(marker)
        rospy.loginfo("已发布截断路径标记 '%s' 到话题 /path_navigator/truncated_marker", path_group_name)

if __name__ == '__main__':
    from sentry_nav.msg import PathNavigationAction

    try:
        PathNavigatorEnhanced()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass