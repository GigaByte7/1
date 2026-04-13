#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import json
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#适配直线控制器

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator')
        
        # 加载参数
        self.json_file_path = rospy.get_param('~json_file_path', 'nav_points.json')
        self.path_group_name = rospy.get_param('~path_group_name', 'default_path')
        self.dead_zone_radius = rospy.get_param('~dead_zone_radius', 0.02)  # 目标点容差半径（5厘米）
        self.wait_between_points = rospy.get_param('~wait_between_points', 2.0)  # 点间等待时间
        self.loop_path = rospy.get_param('~loop_path', False)  # 是否循环路径
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 目标点发布器
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # 世界坐标系和机器人坐标系参数
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'body_foot')  # 与动态控制器保持一致
        
        rospy.loginfo("路径点导航器初始化完成")
        rospy.loginfo("目标点容差半径: %.3f米", self.dead_zone_radius)
        rospy.loginfo("机器人坐标系: %s", self.robot_frame)
        
        # 加载路径点
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            rospy.logerr("无法加载路径点，节点将关闭")
            return
        
        rospy.loginfo("加载了 %d 个路径点", len(self.waypoints))
        
        # 主循环
        self.navigate_waypoints()
    
    def load_waypoints(self):
        """从JSON文件加载路径点"""
        try:
            with open(self.json_file_path, 'r') as f:
                data = json.load(f)
                
            # 检查是否为带路径组的格式
            if isinstance(data, dict) and self.path_group_name in data:
                waypoints_data = data[self.path_group_name]
            else:
                # 假设直接是路径点数组
                waypoints_data = data
                
            waypoints = []
            for wp in waypoints_data:
                if isinstance(wp, dict):
                    # 标准格式：包含position和orientation
                    pose = PoseStamped()
                    pose.header.frame_id = self.world_frame
                    pose.pose.position.x = wp.get('position', {}).get('x', 0.0)
                    pose.pose.position.y = wp.get('position', {}).get('y', 0.0)
                    pose.pose.position.z = wp.get('position', {}).get('z', 0.0)
                    pose.pose.orientation.x = wp.get('orientation', {}).get('x', 0.0)
                    pose.pose.orientation.y = wp.get('orientation', {}).get('y', 0.0)
                    pose.pose.orientation.z = wp.get('orientation', {}).get('z', 0.0)
                    pose.pose.orientation.w = wp.get('orientation', {}).get('w', 1.0)
                    waypoints.append(pose)
                else:
                    rospy.logwarn("忽略无效的路径点格式: %s", wp)
            
            return waypoints
            
        except Exception as e:
            rospy.logerr("加载路径点文件失败: %s", e)
            return []
    
    def get_robot_pose(self):
        """获取机器人在世界坐标系中的位置"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.world_frame, self.robot_frame, rospy.Time(0))
            return trans[0], trans[1], trans[2]
        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, "TF异常: %s", e)
            return None, None, None
    
    def distance_to_goal(self, goal_pose):
        """计算机器人到目标点的二维平面距离"""
        x, y, z = self.get_robot_pose()
        if x is None:
            return float('inf')
        
        dx = x - goal_pose.pose.position.x
        dy = y - goal_pose.pose.position.y
        
        # 忽略高度差，只计算二维距离
        return math.sqrt(dx*dx + dy*dy)
    
    def wait_for_arrival(self, goal_pose, timeout=None):
        """等待机器人到达目标点（简化版本，参考path_navigator_node.py）"""
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.get_time()
        
        # 设置合理的固定超时时间，避免过早超时
        if timeout is None:
            # 根据距离动态计算超时，但给予足够裕量
            initial_distance = self.distance_to_goal(goal_pose)
            # 假设速度为0.3 m/s，加上额外缓冲时间
            timeout = max(initial_distance / 0.3 + 30.0, 45.0)  # 最少45秒，足够机器人运动
            rospy.loginfo("超时时间设置为: %.1f 秒 (初始距离: %.2f 米)", timeout, initial_distance)
        
        arrival_count = 0
        required_arrival_count = 3  # 需要连续3次检测到在死区内才认为到达
        
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            
            # 检查超时
            if current_time - start_time > timeout:
                rospy.logwarn("到达目标点超时 (%.1f 秒)，但机器人仍在尝试", timeout)
                # 不立即返回False，给机器人更多时间
                if current_time - start_time > timeout * 1.5:  # 再给50%额外时间
                    return False
                # 继续等待
        
            # 计算距离
            distance = self.distance_to_goal(goal_pose)
            
            # 如果距离在死区内，增加计数
            if distance <= self.dead_zone_radius:
                arrival_count += 1
                if arrival_count >= required_arrival_count:
                    rospy.loginfo("成功到达目标点! 最终距离: %.3f 米", distance)
                    return True
            else:
                # 如果距离超出死区，重置计数
                arrival_count = 0
            
            # 定期打印进度信息
            if int(current_time - start_time) % 5 == 0:  # 每5秒打印一次
                remaining_time = timeout - (current_time - start_time)
                rospy.loginfo("导航中... 距离: %.3f 米, 容差: %.3f 米, 剩余时间: %.1f 秒, 到达计数: %d/%d", 
                             distance, self.dead_zone_radius, remaining_time, 
                             arrival_count, required_arrival_count)
            
            rate.sleep()
        
        return False
    
    def navigate_waypoints(self):
        """执行逐点导航（带重试机制）"""
        rospy.loginfo("开始逐点导航")
        
        # 等待直线控制器就绪
        rospy.sleep(3.0)
        
        # 导航循环
        waypoint_index = 0
        while not rospy.is_shutdown() and waypoint_index < len(self.waypoints):
            current_waypoint = self.waypoints[waypoint_index]
            
            rospy.loginfo("导航到路径点 %d/%d: (%.2f, %.2f, %.2f)", 
                         waypoint_index + 1, len(self.waypoints),
                         current_waypoint.pose.position.x,
                         current_waypoint.pose.position.y,
                         current_waypoint.pose.position.z)
            
            # 重试机制
            max_retries = 3
            retry_count = 0
            arrived = False
            
            while retry_count < max_retries and not arrived:
                retry_count += 1
                
                if retry_count > 1:
                    rospy.logwarn("重试路径点 %d (第%d次重试)", 
                                 waypoint_index + 1, retry_count - 1)
                
                # 发布目标点
                current_waypoint.header.stamp = rospy.Time.now()
                rospy.loginfo("发布路径点 %d: (%.3f, %.3f)", 
                             waypoint_index + 1,
                             current_waypoint.pose.position.x,
                             current_waypoint.pose.position.y)
                self.goal_pub.publish(current_waypoint)
                
                # 等待到达
                arrived = self.wait_for_arrival(current_waypoint)
                
                if arrived:
                    rospy.loginfo("成功到达路径点 %d", waypoint_index + 1)
                    break
                else:
                    if retry_count < max_retries:
                        rospy.logwarn("路径点 %d 第%d次尝试失败，准备重试", 
                                     waypoint_index + 1, retry_count)
                        # 等待一下再重试
                        rospy.sleep(2.0)
            
            if arrived:
                waypoint_index += 1
                
                # 如果不是最后一个点，等待一下再发布下一个点
                if waypoint_index < len(self.waypoints):
                    rospy.loginfo("等待 %.1f 秒后前往下一个路径点", self.wait_between_points)
                    rospy.sleep(self.wait_between_points)
                else:
                    rospy.loginfo("所有路径点导航完成!")
            else:
                rospy.logwarn("路径点 %d 经过 %d 次尝试后仍未能到达，跳过", 
                             waypoint_index + 1, max_retries)
                waypoint_index += 1  # 跳过这个点，继续下一个
        
        rospy.loginfo("逐点导航完成!")
        
        # 如果启用了循环，重新开始
        if self.loop_path:
            rospy.loginfo("路径循环启用，重新开始导航")
            rospy.sleep(2.0)
            self.navigate_waypoints()
    
    def run(self):
        """运行节点"""
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass