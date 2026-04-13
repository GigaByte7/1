#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import json
import tf
import math
import re

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sentry_nav.msg import PathNavigationAction, PathNavigationGoal, PathNavigationResult, PathNavigationFeedback

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

class PathNavigator:
    def __init__(self):
        rospy.init_node('path_navigator_node')
        
        # 从参数服务器获取参数
        json_file_path = rospy.get_param('~json_file_path', 'nav_points.json')
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'body_foot')
        self.dead_zone_radius = rospy.get_param('~dead_zone_radius', 0.06)
        
        rospy.loginfo("Path Navigator 参数:")
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
        
        
        
        # 创建Action服务器
        self._action_server = actionlib.SimpleActionServer(
            '/track_points', 
            PathNavigationAction, 
            execute_cb=self.execute_cb, 
            auto_start=False
        )
        self._action_server.start()

        rospy.loginfo("Path Navigation Action Server is ready. 等待Action目标...")
        rospy.loginfo("路径可视化话题:")
        rospy.loginfo("  - Path: /path_navigator/full_path")
        rospy.loginfo("  - Marker: /path_navigator/path_marker")
        rospy.loginfo("  - MarkerArray: /path_navigator/marker_array")
   

    def execute_cb(self, goal):
        # 使用Action目标中的dead_zone_radius，如果没有则使用节点参数
        dead_zone_radius = goal.dead_zone_radius if goal.dead_zone_radius > 0 else self.dead_zone_radius
        
        rospy.loginfo("Received navigation goal for path group: '%s' with dead zone radius: %.3f", 
                      goal.path_group_name, dead_zone_radius)

        # 1. 验证路径组是否存在
        if goal.path_group_name not in self.nav_point_groups:
            rospy.logerr("Path group '%s' not found in JSON file.", goal.path_group_name)
            self._action_server.set_aborted(result=PathNavigationResult(success=False, message="Path group not found."))
            return
        waypoints = self.nav_point_groups[goal.path_group_name]
        rospy.loginfo("Executing path with %d waypoints.", len(waypoints))

        # 2. 发布完整路径到RViz
        self.publish_full_path(waypoints, goal.path_group_name)

        # 3. 循环遍历路径点
        for i, waypoint_data in enumerate(waypoints):
            # 检查Action是否被客户端取消
            if self._action_server.is_preempt_requested():
                rospy.loginfo("Navigation preempted by client.")
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
                         i + 1, len(waypoints),
                         goal_pose.pose.position.x,
                         goal_pose.pose.position.y,
                         goal_pose.pose.position.z)
            
            self.goal_pub.publish(goal_pose)
            
            # 发布反馈
            feedback = PathNavigationFeedback()
            feedback.current_waypoint_info = "Navigating to waypoint %d of %d in group '%s'." % (i + 1, len(waypoints), goal.path_group_name)
            self._action_server.publish_feedback(feedback)
            rospy.loginfo(feedback.current_waypoint_info)

            # 3. 等待机器人到达死区
            arrived = self.wait_for_arrival(goal_pose, dead_zone_radius)
            
            # 如果在等待时被抢占，则退出
            if self._action_server.is_preempt_requested():
                rospy.loginfo("Navigation preempted by client while waiting for arrival.")
                self._action_server.set_preempted()
                return
            
            if arrived:
                rospy.loginfo("Waypoint %d reached successfully.", i + 1)
            else:
                rospy.logwarn("Waypoint %d not reached within timeout, continuing to next waypoint.", i + 1)

        # 4. 所有点都尝试完成
        result = PathNavigationResult(success=True, message="Navigation completed successfully.")
        self._action_server.set_succeeded(result)
        rospy.loginfo("Path group '%s' navigation completed.", goal.path_group_name)

    def wait_for_arrival(self, target_pose, radius, timeout_seconds=60.0):
        """等待机器人到达目标点指定的半径内"""
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.get_time()
        arrived = False
        
        # 记录初始距离
        initial_distance = None
        last_log_time = start_time
        
        while not rospy.is_shutdown() and not arrived:
            current_time = rospy.get_time()
            
            # 检查超时
            if current_time - start_time > timeout_seconds:
                rospy.logwarn("到达目标点超时 (%.1f 秒).", timeout_seconds)
                break
            
            # 检查Action是否被客户端取消
            if self._action_server.is_preempt_requested():
                rospy.loginfo("Navigation preempted while waiting for arrival.")
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
                    remaining_time = timeout_seconds - (current_time - start_time)
                    rospy.loginfo("导航中... 距离: %.3f 米, 容差: %.3f 米, 剩余时间: %.1f 秒", 
                                 distance, radius, remaining_time)
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
        rospy.loginfo("已发布完整路径 '%s' 到话题 /path_navigator/full_path，共 %d 个点", 
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
        rospy.loginfo("已发布路径线条标记 '%s' 到话题 /path_navigator/path_marker", path_group_name)
        
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

if __name__ == '__main__':
    from sentry_nav.msg import PathNavigationAction

    try:
        PathNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass