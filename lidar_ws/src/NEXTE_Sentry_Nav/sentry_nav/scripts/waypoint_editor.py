#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式导航点编辑器节点
允许用户在RViz中通过鼠标拖动导航点来调整坐标
"""

import rospy
import tf
import json
import math
import os
import threading
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, InteractiveMarkerUpdate
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class WaypointEditor:
    def __init__(self):
        rospy.init_node('waypoint_editor')
        
        # 加载参数
        self.json_file_path = rospy.get_param('~json_file_path', 'nav_points.json')
        self.path_group_name = rospy.get_param('~path_group_name', 'default_path')
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.marker_size = rospy.get_param('~marker_size', 0.3)
        
        # 确保JSON文件路径存在
        if not os.path.isabs(self.json_file_path):
            # 尝试在sentry_nav包中查找
            try:
                from rospkg import RosPack
                rp = RosPack()
                pkg_path = rp.get_path('sentry_nav')
                self.json_file_path = os.path.join(pkg_path, self.json_file_path)
            except:
                rospy.logwarn("无法解析包路径，使用相对路径: %s", self.json_file_path)
        
        rospy.loginfo("导航点编辑器初始化")
        rospy.loginfo("JSON文件: %s", self.json_file_path)
        rospy.loginfo("路径组: %s", self.path_group_name)
        rospy.loginfo("世界坐标系: %s", self.world_frame)
        
        # 交互式标记服务器
        self.server = InteractiveMarkerServer("waypoint_editor")
        
        # 存储导航点数据
        self.waypoints = []  # 存储PoseStamped对象
        self.waypoint_names = []  # 存储标记名称
        self.lock = threading.Lock()
        
        # 加载导航点
        if not self.load_waypoints():
            rospy.logerr("无法加载导航点，编辑器将无法工作")
            return
        
        # 创建交互式标记
        self.create_interactive_markers()
        
        # 发布更新后的导航点到话题
        self.waypoint_pub = rospy.Publisher('/edited_waypoints', PoseStamped, queue_size=10, latch=True)
        self.waypoint_array_pub = rospy.Publisher('/edited_waypoints_array', Marker, queue_size=10, latch=True)
        
        # 定时保存
        self.save_timer = rospy.Timer(rospy.Duration(5.0), self.save_waypoints_callback)
        
        rospy.loginfo("导航点编辑器已就绪")
        rospy.loginfo("在RViz中可以通过拖动标记来调整导航点位置")
        rospy.loginfo("调整后的点会自动保存到文件")
        
    def load_waypoints(self):
        """从JSON文件加载导航点"""
        try:
            with open(self.json_file_path, 'r') as f:
                data = json.load(f)
            
            # 检查是否为带路径组的格式
            if isinstance(data, dict) and self.path_group_name in data:
                waypoints_data = data[self.path_group_name]
            else:
                # 假设直接是导航点数组
                waypoints_data = data
            
            self.waypoints = []
            for i, wp in enumerate(waypoints_data):
                if isinstance(wp, dict):
                    pose = PoseStamped()
                    pose.header.frame_id = self.world_frame
                    pose.pose.position.x = wp.get('position', {}).get('x', 0.0)
                    pose.pose.position.y = wp.get('position', {}).get('y', 0.0)
                    pose.pose.position.z = wp.get('position', {}).get('z', 0.0)
                    pose.pose.orientation.x = wp.get('orientation', {}).get('x', 0.0)
                    pose.pose.orientation.y = wp.get('orientation', {}).get('y', 0.0)
                    pose.pose.orientation.z = wp.get('orientation', {}).get('z', 0.0)
                    pose.pose.orientation.w = wp.get('orientation', {}).get('w', 1.0)
                    self.waypoints.append(pose)
                    rospy.logdebug("加载导航点 %d: (%.3f, %.3f, %.3f)", 
                                  i, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                else:
                    rospy.logwarn("忽略无效的导航点格式: %s", wp)
            
            rospy.loginfo("成功加载 %d 个导航点", len(self.waypoints))
            return True
            
        except Exception as e:
            rospy.logerr("加载导航点文件失败: %s", e)
            return False
    
    def create_interactive_markers(self):
        """为所有导航点创建交互式标记"""
        with self.lock:
            self.waypoint_names = []
            self.server.clear()
            
            for i, waypoint in enumerate(self.waypoints):
                marker_name = f"waypoint_{i}"
                self.waypoint_names.append(marker_name)
                
                # 调试信息：打印第一个标记的详细信息
                if i == 0:
                    rospy.loginfo(f"创建导航点0标记 - 位置: ({waypoint.pose.position.x:.3f}, {waypoint.pose.position.y:.3f}, {waypoint.pose.position.z:.3f})")
                    rospy.loginfo(f"导航点0方向: ({waypoint.pose.orientation.x:.3f}, {waypoint.pose.orientation.y:.3f}, {waypoint.pose.orientation.z:.3f}, {waypoint.pose.orientation.w:.3f})")
                
                # 创建交互式标记
                int_marker = InteractiveMarker()
                int_marker.header.frame_id = self.world_frame
                int_marker.pose = waypoint.pose
                int_marker.scale = self.marker_size
                int_marker.name = marker_name
                int_marker.description = f"导航点 {i}\n位置: ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})"
                
                # 创建控制部件 - 6自由度控制（平移+旋转）
                control = InteractiveMarkerControl()
                control.orientation.w = 1
                control.orientation.x = 1
                control.orientation.y = 0
                control.orientation.z = 0
                control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
                control.always_visible = True
                
                # 添加箭头标记
                arrow_marker = Marker()
                arrow_marker.type = Marker.ARROW
                arrow_marker.scale.x = self.marker_size * 0.8
                arrow_marker.scale.y = self.marker_size * 0.1
                arrow_marker.scale.z = self.marker_size * 0.1
                arrow_marker.color.r = 0.0
                arrow_marker.color.g = 1.0
                arrow_marker.color.b = 0.0
                arrow_marker.color.a = 0.8
                control.markers.append(arrow_marker)
                
                # 添加球体标记
                sphere_marker = Marker()
                sphere_marker.type = Marker.SPHERE
                sphere_marker.scale.x = self.marker_size * 0.3
                sphere_marker.scale.y = self.marker_size * 0.3
                sphere_marker.scale.z = self.marker_size * 0.3
                sphere_marker.color.r = 0.0
                sphere_marker.color.g = 0.7
                sphere_marker.color.b = 1.0
                sphere_marker.color.a = 0.6
                sphere_marker.pose.position.z = self.marker_size * 0.15  # 稍微抬高
                control.markers.append(sphere_marker)
                
                # 为导航点0添加更大的选择区域（如果z坐标接近0）
                if i == 0 and abs(waypoint.pose.position.z) < 0.01:
                    # 添加一个更大的半透明球体作为选择区域
                    selection_marker = Marker()
                    selection_marker.type = Marker.SPHERE
                    selection_marker.scale.x = self.marker_size * 0.8
                    selection_marker.scale.y = self.marker_size * 0.8
                    selection_marker.scale.z = self.marker_size * 0.8
                    selection_marker.color.r = 1.0
                    selection_marker.color.g = 0.5
                    selection_marker.color.b = 0.0
                    selection_marker.color.a = 0.3
                    control.markers.append(selection_marker)
                    rospy.loginfo("为导航点0添加了扩展选择区域")
                
                # 添加文本标记
                text_marker = Marker()
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.text = str(i)
                text_marker.scale.z = self.marker_size * 0.3
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.pose.position.z = self.marker_size * 0.5  # 在球体上方
                control.markers.append(text_marker)
                
                int_marker.controls.append(control)
                
                # 添加额外的控制：平面内移动
                control_xy = InteractiveMarkerControl()
                control_xy.orientation.w = 1
                control_xy.orientation.x = 0
                control_xy.orientation.y = 1
                control_xy.orientation.z = 0
                control_xy.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
                int_marker.controls.append(control_xy)
                
                # 添加旋转控制
                control_rot = InteractiveMarkerControl()
                control_rot.orientation.w = 1
                control_rot.orientation.x = 0
                control_rot.orientation.y = 1
                control_rot.orientation.z = 0
                control_rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
                int_marker.controls.append(control_rot)
                
                # 将标记添加到服务器并设置反馈回调
                self.server.insert(int_marker, self.marker_feedback_callback)
                if i == 0:
                    rospy.loginfo(f"导航点0标记 '{marker_name}' 已添加到服务器")
            
            # 应用所有更改
            self.server.applyChanges()
            
            rospy.loginfo("创建了 %d 个交互式标记", len(self.waypoints))
    
    def marker_feedback_callback(self, feedback):
        """交互式标记反馈回调"""
        # 只处理姿势更新事件
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # 提取标记索引
            try:
                marker_index = int(feedback.marker_name.replace("waypoint_", ""))
            except:
                rospy.logwarn("无法解析标记名称: %s", feedback.marker_name)
                return
            
            # 调试信息：特别是对于标记0
            if marker_index == 0:
                rospy.loginfo(f"收到导航点0的POSE_UPDATE事件")
                rospy.loginfo(f"标记名称: {feedback.marker_name}")
                rospy.loginfo(f"新位置: ({feedback.pose.position.x:.3f}, {feedback.pose.position.y:.3f}, {feedback.pose.position.z:.3f})")
            
            with self.lock:
                if 0 <= marker_index < len(self.waypoints):
                    # 更新导航点位置
                    self.waypoints[marker_index].pose = feedback.pose
                    
                    # 发布更新后的导航点
                    updated_pose = PoseStamped()
                    updated_pose.header.frame_id = self.world_frame
                    updated_pose.header.stamp = rospy.Time.now()
                    updated_pose.pose = feedback.pose
                    self.waypoint_pub.publish(updated_pose)
                    
                    # 更新标记姿势
                    self.server.setPose(feedback.marker_name, feedback.pose)
                    self.server.applyChanges()
                    
                    rospy.loginfo("更新导航点 %d: (%.3f, %.3f, %.3f)", 
                                 marker_index, 
                                 feedback.pose.position.x,
                                 feedback.pose.position.y,
                                 feedback.pose.position.z)
                    
                    # 发布可视化数组
                    self.publish_waypoint_array()
                else:
                    rospy.logwarn("无效的标记索引: %d", marker_index)
        else:
            # 忽略其他事件类型
            rospy.logdebug("忽略事件类型 %d 的反馈: %s", feedback.event_type, feedback.marker_name)
    
    def save_waypoints_callback(self, event=None):
        """保存导航点到JSON文件"""
        with self.lock:
            try:
                # 读取现有文件
                if os.path.exists(self.json_file_path):
                    with open(self.json_file_path, 'r') as f:
                        data = json.load(f)
                else:
                    data = {}
                
                # 准备导航点数据
                waypoints_data = []
                for waypoint in self.waypoints:
                    wp_dict = {
                        'position': {
                            'x': waypoint.pose.position.x,
                            'y': waypoint.pose.position.y,
                            'z': waypoint.pose.position.z
                        },
                        'orientation': {
                            'x': waypoint.pose.orientation.x,
                            'y': waypoint.pose.orientation.y,
                            'z': waypoint.pose.orientation.z,
                            'w': waypoint.pose.orientation.w
                        }
                    }
                    waypoints_data.append(wp_dict)
                
                # 更新数据
                if isinstance(data, dict):
                    data[self.path_group_name] = waypoints_data
                else:
                    # 如果原文件不是字典，创建新结构
                    data = {self.path_group_name: waypoints_data}
                
                # 保存文件
                with open(self.json_file_path, 'w') as f:
                    json.dump(data, f, indent=2)
                
                rospy.logdebug("导航点已保存到: %s", self.json_file_path)
                
            except Exception as e:
                rospy.logwarn("保存导航点失败: %s", e)
    
    def publish_waypoint_array(self):
        """发布所有导航点的可视化数组"""
        if not self.waypoints:
            return
        
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edited_waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # 点大小
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # 添加所有导航点
        for i, waypoint in enumerate(self.waypoints):
            point = Point()
            point.x = waypoint.pose.position.x
            point.y = waypoint.pose.position.y
            point.z = waypoint.pose.position.z
            marker.points.append(point)
            
            # 设置颜色（渐变）
            color = ColorRGBA()
            hue = i / max(1, len(self.waypoints) - 1)
            color.r = 1.0 - hue
            color.g = hue
            color.b = 0.5
            color.a = 0.8
            marker.colors.append(color)
        
        self.waypoint_array_pub.publish(marker)
    
    def run(self):
        """运行节点"""
        # 初始发布可视化
        self.publish_waypoint_array()
        
        rospy.loginfo("导航点编辑器运行中...")
        rospy.loginfo("在RViz中添加InteractiveMarkers显示（话题: /waypoint_editor/update）")
        rospy.loginfo("或添加Marker显示（话题: /edited_waypoints_array）")
        rospy.spin()
    
    def shutdown(self):
        """关闭节点时的清理工作"""
        rospy.loginfo("保存导航点...")
        self.save_waypoints_callback()
        rospy.loginfo("导航点编辑器关闭")

if __name__ == '__main__':
    try:
        editor = WaypointEditor()
        rospy.on_shutdown(editor.shutdown)
        editor.run()
    except rospy.ROSInterruptException:
        pass