#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
动态目标点直线控制器 - 带车轮旋转90度的平移运动
扩展自增强版控制器，添加了完整的平移状态机：
1. 检测到平移运动时，首先旋转车轮90度
2. 等待旋转完成（时间或反馈）
3. 开始平移运动
4. 到达目标点后，旋转车轮回0度（复位）
5. 等待复位完成
订阅RViz中的2D Nav Goal目标点，从当前位置直线运动到目标点

备份文件在/home/ubuntu/lidar_ws/src/NEXTE_Sentry_Nav/sentry_nav/scripts/dynamic_goal_line_controller_enhanced_copy.py
"""

import rospy
import tf
import math
import actionlib
import threading
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from vel_pkg.srv import SetKinematicMode, SetKinematicModeRequest
from sentry_nav.srv import PauseNavigation, PauseNavigationResponse


class DynamicGoalLineControllerWithWheelRotation:
    def __init__(self):
        """初始化带车轮旋转的控制器"""
        rospy.init_node('dynamic_goal_line_controller_with_wheel_rotation', anonymous=True)
        
        # 加载参数
        self.load_parameters()
        
        # 直线参数（动态更新）
        self.line_angle = 0.0
        self.is_vertical = False
        self.k = 0.0  # 斜率
        self.b = 0.0  # 截距
        self.x_const = 0.0
        self.start_point = (0.0, 0.0)  # 起点（机器人当前位置）
        self.end_point = (0.0, 0.0)    # 终点（目标点）
        self.has_goal = False          # 是否有有效目标点
        self.goal_received_time = rospy.get_time()
        
        # 平移运动状态机
        self.translation_state = 'IDLE'  # IDLE, ROTATE_TO_90, TRANSLATING, ROTATE_TO_ZERO
        self.translation_type = None     # 'vertical', 'horizontal', None
        self.state_start_time = rospy.get_time()
        self.vertical_line_y_error = 0.0
        self.vertical_line_y_sign = 1
        
        # 防止过早检测到达目标点的安全机制
        self.just_started_translating = False
        self.translating_start_time = rospy.get_time()
        
        # 平移起始位置记录
        self.translation_start_position = None  # (x, y)
        self.min_travel_distance = 0.1  # 最小移动距离后才开始检测目标点
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 速度发布器
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        
        # 订阅RViz中的目标点话题
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
 
        # 可选：发布当前目标点用于可视化
        self.goal_pub = rospy.Publisher('/simple_line_goal', PoseStamped, queue_size=1)
        
        # 发布起点到终点的连线（Marker）
        self.line_pub = rospy.Publisher('/simple_line_path', Marker, queue_size=1)
        
        # 记录和发布实际路径
        self.actual_path_pub = rospy.Publisher('/actual_path_marker', Marker, queue_size=1)
        self.path_points = []  # 存储路径点
        self.last_recorded_point = None  # 上一个记录点
        self.min_record_distance = rospy.get_param('~min_record_distance', 0.05)
        self.max_path_points = rospy.get_param('~max_path_points', 500)
        
        # 控制变量初始化
        self.prev_lateral_error = 0.0
        self.prev_heading_error = 0.0
        self.prev_time = rospy.get_time()

        
        # 积分项
        self.lateral_error_integral = 0.0
        self.heading_error_integral = 0.0
        
        # 暂停功能相关变量
        self.is_paused = False
        self.pause_lock = threading.Lock()
        
        # 创建暂停导航服务
        self.pause_service = rospy.Service('/pause_navigation', PauseNavigation, self.handle_pause_navigation)
        rospy.loginfo("已创建暂停导航服务: /pause_navigation")
        
        # 尝试连接twist_to_carry服务（用于切换运动学模式）
        self.set_kinematic_mode_service = None
        try:
            # 使用相对服务名称，ROS会自动解析为完整名称
            rospy.wait_for_service('twist_to_carry/set_kinematic_mode', timeout=5.0)
            self.set_kinematic_mode_service = rospy.ServiceProxy(
                'twist_to_carry/set_kinematic_mode', SetKinematicMode)
            rospy.loginfo("已连接到twist_to_carry服务")
        except rospy.ROSException as e:
            rospy.logwarn("无法连接到twist_to_carry服务: %s", e)
            rospy.logwarn("将继续运行，但平移模式切换功能将不可用")
        
        # 控制定时器
        control_rate = rospy.Rate(self.control_frequency)
        rospy.loginfo("带车轮旋转的平移运动控制器已启动")
        rospy.loginfo("等待RViz中的2D Nav Goal目标点...")
        
        # 注册关闭钩子，确保节点停止时发送零速度
        rospy.on_shutdown(self.shutdown_hook)
        
        # 主控制循环
        while not rospy.is_shutdown():
            self.control_loop()
            control_rate.sleep()
    
    def load_parameters(self):
        """加载参数"""
        # 控制模式
        self.control_mode = rospy.get_param('~control_mode', 'holonomic')
        
        # 机器人参数
        self.robot_length = rospy.get_param('~robot_length', 1.5)
        self.robot_width = rospy.get_param('~robot_width', 1.0)
        
        # 控制频率
        self.control_frequency = rospy.get_param('~control_frequency', 20.0)
        
        # 速度限制
        self.max_vel_x = rospy.get_param('~max_vel_x', 0.8)
        self.max_vel_y = rospy.get_param('~max_vel_y', 0.5)
        self.max_vel_theta = rospy.get_param('~max_vel_theta', 0.4)
        
        # PID参数
        self.kp_lateral = rospy.get_param('~kp_lateral', 3.5)
        self.kd_lateral = rospy.get_param('~kd_lateral', 0.3)
        self.kp_heading = rospy.get_param('~kp_heading', 1.2)
        self.kd_heading = rospy.get_param('~kd_heading', 0.2)
        
        # 积分控制参数
        self.ki_lateral = rospy.get_param('~ki_lateral', 0.2)
        self.ki_heading = rospy.get_param('~ki_heading', 0.1)
        self.integral_limit = rospy.get_param('~integral_limit', 0.08)
        
        # 控制精度阈值
        self.navigation_precision = rospy.get_param('~navigation_precision', 0.02)
        self.max_lateral_error = rospy.get_param('~max_lateral_error', 0.02)  # 2cm偏差限制
        self.max_heading_error = rospy.get_param('~max_heading_error', 0.05)
        
        # 偏差纠正增强参数 - 当偏差接近或超过限制时使用更强的控制
        self.critical_lateral_error = self.max_lateral_error * 1.2  # 超过1.8cm触发紧急纠正（原1.2倍）
        self.emergency_kp_lateral = self.kp_lateral * 3.0  # 紧急情况下的增强P增益
        self.emergency_kd_lateral = self.kd_lateral * 2.5  # 紧急情况下的增强D增益
        # 初始阶段增强参数 - 解决机器人刚开始运动时反应缓慢的问题
        self.initial_phase_duration = 2.0  # 初始阶段持续时间（秒）
        self.initial_phase_kp_multiplier = 1.8  # 初始阶段P增益倍增系数
        self.initial_phase_kd_multiplier = 1.5  # 初始阶段D增益倍增系数
        self.initial_phase_ki_multiplier = 1.2  # 初始阶段I增益倍增系数
        self.initial_phase_start_time = None  # 初始阶段开始时间
        
        # 平移运动检测参数
        self.enable_translation_detection = rospy.get_param('~enable_translation_detection', True)
        self.translation_threshold = rospy.get_param('~translation_threshold', 0.01)
        self.min_translation_distance = rospy.get_param('~min_translation_distance', 0.1)
        
        # 平移控制参数
        self.translation_kp = rospy.get_param('~translation_kp', 1.5)
        self.translation_min_speed = rospy.get_param('~translation_min_speed', 0.1)
        self.translation_max_speed = rospy.get_param('~translation_max_speed', 0.5)
        
        # 车轮旋转参数
        self.wheel_rotation_time = rospy.get_param('~wheel_rotation_time', 2.0)  # 车轮旋转90度所需时间
        self.wheel_reset_time = rospy.get_param('~wheel_reset_time', 2.0)  # 车轮复位所需时间
        
        # 倒车参数
        self.backward_speed_ratio = rospy.get_param('~backward_speed_ratio', 0.8)  # 倒车速度比例
        
        # 话题名称
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        
        # TF帧名称
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'body_foot')
        
        # 目标容差 - 与launch文件中的dead_zone_radius保持一致
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.06)
        
        # 目标点超时检查
        self.goal_timeout = rospy.get_param('~goal_timeout', 30.0)
        
        rospy.loginfo("控制器参数: goal_tolerance=%.3f, goal_timeout=%.1f", 
                     self.goal_tolerance, self.goal_timeout)
        
    
    
    def goal_callback(self, msg):
        """RViz目标点回调函数"""
      
        
        # 状态检查：如果当前状态不是IDLE且不是ROTATE_TO_ZERO，先重置状态
        if self.translation_state not in ['IDLE', 'ROTATE_TO_ZERO']:
            rospy.logwarn("接收到新目标点，但当前状态为%s，类型为%s，先重置状态到IDLE", 
                         self.translation_state, self.translation_type)
            self.translation_state = 'IDLE'
            self.translation_type = None
            # 确保切换到差速模式
            if self.set_kinematic_mode_service is not None:
                try:
                    req = SetKinematicModeRequest()
                    req.mode = 'differential'
                    resp = self.set_kinematic_mode_service(req)
                    if resp.success:
                        rospy.loginfo("已重置并切换到差速模式 (differential)")
                    else:
                        rospy.logwarn("重置并切换到差速模式失败: %s", resp.message)
                except rospy.ServiceException as e:
                    rospy.logerr("调用set_kinematic_mode服务失败: %s", e)
        
        # 如果正在复位车轮，忽略新目标点，等待复位完成
        if self.translation_state == 'ROTATE_TO_ZERO':
            rospy.loginfo("正在复位车轮中，忽略新目标点，等待复位完成后再接收新目标点")
            return
        
        # 获取机器人当前位置
        x, y, yaw = self.get_robot_pose()
        if x is None:
            rospy.logwarn("无法获取机器人当前位置，忽略目标点")
            return
        
        # 设置起点（机器人当前位置）
        self.start_point = (x, y)
        
        # 设置终点（RViz中的目标点）
        self.end_point = (msg.pose.position.x, msg.pose.position.y)
        
        rospy.loginfo("新目标点设置: 起点=(%.3f, %.3f), 终点=(%.3f, %.3f), 机器人朝向=%.2f°", 
                     x, y, self.end_point[0], self.end_point[1], math.degrees(yaw))
        
        # 计算直线参数
        self.calculate_line_parameters()
        
        # 标记有有效目标点
        self.has_goal = True
        self.goal_received_time = rospy.get_time()
        
        # 重置积分项
        self.lateral_error_integral = 0.0
        self.heading_error_integral = 0.0
        
        # 设置初始阶段开始时间（解决机器人刚开始运动时反应缓慢的问题）
        self.initial_phase_start_time = rospy.get_time()
        rospy.loginfo("设置初始阶段开始时间，增强前 %.1f 秒内的控制响应", self.initial_phase_duration)
        
        
        # 检测是否为平移运动
        self.detect_translation_motion()
        
        # 如果是平移运动，启动状态机
        if self.translation_type:
            rospy.loginfo("检测到平移运动: %s，启动车轮旋转状态机", self.translation_type)
            self.start_translation_state_machine()
        else:
            rospy.loginfo("一般直线运动: dx=%.3f, dy=%.3f", 
                         self.end_point[0] - self.start_point[0], 
                         self.end_point[1] - self.start_point[1])
        
        # 发布目标点用于可视化
        self.goal_pub.publish(msg)
        
        # 发布起点到终点的连线
        self.publish_line_marker()
        
    def detect_translation_motion(self):
        """检测平移运动类型（增强版）"""
        if not self.enable_translation_detection:
            self.translation_type = None
            return

        # 获取机器人当前朝向
        x, y, yaw = self.get_robot_pose()
        if x is None:
            self.translation_type = None
            return
            
        dx_world = self.end_point[0] - self.start_point[0]
        dy_world = self.end_point[1] - self.start_point[1]
        
        # 计算目标方向（世界坐标系）
        target_angle_world = math.atan2(dy_world, dx_world)
        
        # 将世界坐标系中的位移转换到机器人坐标系
        # 机器人坐标系：X轴向前，Y轴向左
        dx_robot = dx_world * math.cos(yaw) + dy_world * math.sin(yaw)
        dy_robot = -dx_world * math.sin(yaw) + dy_world * math.cos(yaw)
        
        # 计算世界坐标系下的路径角度与机器人朝向的绝对差值
        angle_diff = target_angle_world - yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 重置平移类型
        self.translation_type = None

        # 增强的平移检测逻辑
        # 对于垂直平移运动，我们关注以下特征：
        # 1. 在世界坐标系中，路径主要是Y方向运动（|dy_world| >> |dx_world|）
        # 2. 总距离足够长（> min_translation_distance）
        # 3. 路径角度与机器人当前朝向的差接近90度或-90度（对于侧向平移）
        
        # 计算世界坐标系的位移特征
        world_distance = math.sqrt(dx_world**2 + dy_world**2)
        world_dx_ratio = abs(dx_world) / world_distance if world_distance > 0 else 0
        world_dy_ratio = abs(dy_world) / world_distance if world_distance > 0 else 0
        
        rospy.loginfo("平移检测分析:")
        rospy.loginfo("  世界坐标系: dx=%.3f m, dy=%.3f m, 距离=%.3f m, 角度=%.1f°", 
                     dx_world, dy_world, world_distance, math.degrees(target_angle_world))
        rospy.loginfo("  机器人坐标系: dx_r=%.3f m, dy_r=%.3f m", dx_robot, dy_robot)
        rospy.loginfo("  机器人朝向: %.1f°, 角度差=%.1f°", math.degrees(yaw), math.degrees(angle_diff))
        rospy.loginfo("  世界坐标系位移比例: |dx|/距离=%.2f, |dy|/距离=%.2f", world_dx_ratio, world_dy_ratio)
        
        # 方法1：基于世界坐标系的检测（更稳定，不受机器人当前朝向影响）
        # 检查是否为垂直平移的典型特征（主要Y方向运动）
        is_world_vertical_motion = (
            world_distance > self.min_translation_distance and
            world_dy_ratio > 0.8 and  # Y方向占主导（>80%）
            world_dx_ratio < 0.3      # X方向运动很小（<30%）
        )
        
        # 方法2：基于机器人坐标系的检测
        robot_distance = math.sqrt(dx_robot**2 + dy_robot**2)
        robot_dx_ratio = abs(dx_robot) / robot_distance if robot_distance > 0 else 0
        robot_dy_ratio = abs(dy_robot) / robot_distance if robot_distance > 0 else 0
        
        is_robot_vertical_motion = (
            robot_distance > self.min_translation_distance and
            robot_dy_ratio > 0.8 and   # 机器人坐标系的Y方向占主导
            robot_dx_ratio < 0.3
        )
        
        # 方法3：检查角度特征（对于侧向平移，路径角度与机器人朝向差接近±90度）
        is_sideways_motion = (
            world_distance > self.min_translation_distance and
            (abs(abs(angle_diff) - math.pi/2) < 0.35)  # 角度差接近90度（±20度范围内）
        )
        
        rospy.loginfo("平移检测结果:")
        rospy.loginfo("  世界坐标系检测: %s (dy比例=%.2f>0.8? dx比例=%.2f<0.3? 距离=%.3f>%.3f?)", 
                     is_world_vertical_motion, world_dy_ratio, world_dx_ratio, 
                     world_distance, self.min_translation_distance)
        rospy.loginfo("  机器人坐标系检测: %s (dy比例=%.2f>0.8? dx比例=%.2f<0.3? 距离=%.3f>%.3f?)", 
                     is_robot_vertical_motion, robot_dy_ratio, robot_dx_ratio,
                     robot_distance, self.min_translation_distance)
        rospy.loginfo("  侧向运动检测: %s (角度差=%.1f°, 接近90度?)", 
                     is_sideways_motion, math.degrees(angle_diff))
        
        # 如果满足任一条件，检测为垂直平移
        if is_world_vertical_motion or is_robot_vertical_motion or is_sideways_motion:
            self.translation_type = 'vertical'
            
            # 确定平移方向（基于机器人坐标系的dy_robot符号）
            if abs(dy_robot) > 0.01:  # 有显著的Y方向运动
                direction = "左移" if dy_robot > 0 else "右移"
            else:
                # 如果没有明显的机器人坐标系Y运动，使用世界坐标系的dy_world符号
                direction = "正Y方向" if dy_world > 0 else "负Y方向"
            
            rospy.loginfo("检测到垂直平移运动！类型: vertical, 方向: %s", direction)
            rospy.loginfo("详细参数:")
            rospy.loginfo("  世界位移: (%.3f, %.3f) m, 距离: %.3f m", dx_world, dy_world, world_distance)
            rospy.loginfo("  机器人位移: (%.3f, %.3f) m, 距离: %.3f m", dx_robot, dy_robot, robot_distance)
            rospy.loginfo("  机器人朝向: %.1f°, 路径角度: %.1f°, 角度差: %.1f°", 
                         math.degrees(yaw), math.degrees(target_angle_world), math.degrees(angle_diff))
        else:
            # 不是垂直平移运动，使用一般直线控制
            rospy.loginfo("使用一般直线控制: 距离=%.3f m, 角度=%.1f°", 
                         world_distance, math.degrees(target_angle_world))
            rospy.loginfo("不满足平移条件:")
            if world_distance <= self.min_translation_distance:
                rospy.loginfo("  - 距离(%.3f) ≤ 最小平移距离(%.3f)", world_distance, self.min_translation_distance)
            if world_dy_ratio <= 0.8:
                rospy.loginfo("  - Y方向比例(%.2f) ≤ 0.8", world_dy_ratio)
            if world_dx_ratio >= 0.3:
                rospy.loginfo("  - X方向比例(%.2f) ≥ 0.3", world_dx_ratio)
            if abs(abs(angle_diff) - math.pi/2) >= 0.35:
                rospy.loginfo("  - 角度差(%.1f°)不接近90度", math.degrees(angle_diff))
    
    def start_translation_state_machine(self):
        """启动平移状态机"""
        if self.translation_type:
            self.translation_state = 'ROTATE_TO_90'
            self.state_start_time = rospy.get_time()
            
            # 切换到平移模式（如果服务可用）
            if self.set_kinematic_mode_service is not None:
                try:
                    req = SetKinematicModeRequest()
                    req.mode = 'lateral'
                    resp = self.set_kinematic_mode_service(req)
                    if resp.success:
                        rospy.loginfo("已切换到平移模式 (lateral)")
                    else:
                        rospy.logwarn("切换到平移模式失败: %s", resp.message)
                except rospy.ServiceException as e:
                    rospy.logerr("调用set_kinematic_mode服务失败: %s", e)
            else:
                rospy.logwarn("twist_to_carry服务不可用，跳过模式切换")
            
            rospy.loginfo("开始旋转车轮到90度...")
    
    def finish_translation_state_machine(self):
        """完成平移状态机，复位车轮"""
        rospy.loginfo("完成平移状态机，开始复位车轮。当前状态: %s, 类型: %s", 
                     self.translation_state, self.translation_type)
        
        # 立即清除平移类型，因为平移已经完成
        self.translation_type = None
        self.translation_state = 'ROTATE_TO_ZERO'
        self.state_start_time = rospy.get_time()
        
        # 切换到四轮差速模式（车轮会自动复位到0度）
        try:
            req = SetKinematicModeRequest()
            req.mode = 'four_wheel_diff'
            resp = self.set_kinematic_mode_service(req)
            if resp.success:
                rospy.loginfo("已切换回四轮差速模式，车轮将复位")
            else:
                rospy.logwarn("切换回四轮差速模式失败: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("调用set_kinematic_mode服务失败: %s", e)
        
        rospy.loginfo("开始复位车轮到0度，预计时间: %.1f秒", self.wheel_reset_time)
    
    def calculate_line_parameters(self):
        """计算直线方程参数"""
        dx = self.end_point[0] - self.start_point[0]
        dy = self.end_point[1] - self.start_point[1]
        
        # 直线方向角
        self.line_angle = math.atan2(dy, dx)
        
        # 处理垂直直线特殊情况
        if abs(dx) < 1e-6:
            self.is_vertical = True
            self.x_const = self.start_point[0]
        else:
            self.is_vertical = False
            self.k = dy / dx
            self.b = self.start_point[1] - self.k * self.start_point[0]
        
        # 计算直线长度
        self.line_length = math.sqrt(dx**2 + dy**2)
    
    def get_robot_pose(self):
        """获取机器人在世界坐标系中的位置和姿态"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.world_frame, self.robot_frame, rospy.Time(0))
            
            x = trans[0]
            y = trans[1]
            (roll, pitch, yaw) = euler_from_quaternion(rot)
            
            return x, y, yaw
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF异常: %s", e)
            return None, None, None
    
    def calculate_lateral_error(self, x, y):
        """计算横向位置偏差"""
        if self.is_vertical:
            lateral_error = x - self.x_const
            error_sign = 1 if lateral_error > 0 else -1
            lateral_error = abs(lateral_error)
            
            # 对于垂直直线运动，计算Y方向误差
            dy_to_end = self.end_point[1] - y
            self.vertical_line_y_error = dy_to_end
            self.vertical_line_y_sign = 1 if dy_to_end > 0 else -1
        else:
            numerator = abs(self.k * x - y + self.b)
            denominator = math.sqrt(self.k**2 + 1)
            lateral_error = numerator / denominator
            
            line_y = self.k * x + self.b
            error_sign = 1 if y > line_y else -1
            
            # 重要修复：对于垂直平移运动，需要正确计算Y方向误差
            # 即使不是完全垂直的直线，如果是垂直平移，也需要计算Y误差
            if self.translation_type == 'vertical' and self.translation_state == 'TRANSLATING':
                # 对于垂直平移，Y方向误差是到目标点的Y距离
                dy_to_end = self.end_point[1] - y
                self.vertical_line_y_error = dy_to_end
                self.vertical_line_y_sign = 1 if dy_to_end > 0 else -1
            else:
                self.vertical_line_y_error = 0.0
                self.vertical_line_y_sign = 1
        
        # 调试信息：记录垂直平移的误差计算
        if self.translation_type == 'vertical' and self.translation_state == 'TRANSLATING':
            if rospy.get_time() % 1.0 < 0.05:
                rospy.loginfo("垂直平移误差计算: 位置=(%.3f,%.3f), 目标=(%.3f,%.3f), Y误差=%.3f, 符号=%d", 
                             x, y, self.end_point[0], self.end_point[1], 
                             self.vertical_line_y_error, self.vertical_line_y_sign)
        
        return lateral_error, error_sign
    
    def calculate_remaining_distance(self, x, y):
        """计算到终点的剩余距离"""
        dx = self.end_point[0] - x
        dy = self.end_point[1] - y
        return math.sqrt(dx**2 + dy**2)
    
    def is_goal_reached(self, x, y):
        """检查是否到达目标点"""
        remaining_dist = self.calculate_remaining_distance(x, y)
        
        # 添加详细调试信息
        if rospy.get_time() % 0.5 < 0.05:
            rospy.loginfo("目标点检测: 剩余距离=%.3f, 容差=%.3f, 位置=(%.3f,%.3f), 目标=(%.3f,%.3f), 状态=%s, 类型=%s", 
                         remaining_dist, self.goal_tolerance, x, y, 
                         self.end_point[0], self.end_point[1],
                         self.translation_state, self.translation_type)
        
        # 安全机制：如果刚进入平移状态，需要移动一定距离后才开始检测是否到达目标点
        if self.translation_type and self.translation_state == 'TRANSLATING':
            if self.just_started_translating:
                current_time = rospy.get_time()
                translating_duration = current_time - self.translating_start_time
                
                # 如果还没有记录起始位置，记录起始位置
                if self.translation_start_position is None:
                    self.translation_start_position = (x, y)
                    rospy.loginfo("记录平移起始位置: (%.3f, %.3f)", x, y)
                    return False
                
                # 计算从起始位置移动的距离
                start_x, start_y = self.translation_start_position
                travel_distance = math.sqrt((x - start_x)**2 + (y - start_y)**2)
                
                # 如果移动距离小于最小要求，强制返回未到达
                if travel_distance < self.min_travel_distance:
                    rospy.loginfo("安全机制：平移刚开始，移动距离=%.3f/%.3f 米，不检测是否到达目标点", 
                                 travel_distance, self.min_travel_distance)
                    return False
                else:
                    # 已经移动足够距离，清除安全标志
                    rospy.loginfo("安全机制：已移动 %.3f 米 > %.3f 米，开始正常检测目标点", 
                                 travel_distance, self.min_travel_distance)
                    self.just_started_translating = False
                    # 重置起始位置
                    self.translation_start_position = None
            else:
                # 不是刚进入平移状态，正常检测
                pass
        
        # 对于平移运动，使用更严格的容差
        if self.translation_type and self.translation_state == 'TRANSLATING':
            # 对于垂直平移，检查Y方向误差和X方向误差
            if self.translation_type == 'vertical':
                y_error = abs(self.vertical_line_y_error)
                # 对于垂直直线，还需要检查X方向是否在线上
                if self.is_vertical:
                    x_error = abs(x - self.x_const)
                    if y_error < self.goal_tolerance and x_error < self.goal_tolerance:
                        rospy.loginfo("垂直平移到达目标点! Y误差=%.3f, X误差=%.3f < 容差=%.3f", 
                                     y_error, x_error, self.goal_tolerance)
                        return True
                    else:
                        # 记录详细的误差信息
                        if rospy.get_time() % 1.0 < 0.05:
                            rospy.loginfo("垂直平移未到达: Y误差=%.3f, X误差=%.3f, 容差=%.3f", 
                                         y_error, x_error, self.goal_tolerance)
                        return False
                else:
                    # 不是垂直直线，使用Y误差
                    if y_error < self.goal_tolerance:
                        rospy.loginfo("垂直平移到达目标点! Y误差=%.3f < 容差=%.3f", 
                                     y_error, self.goal_tolerance)
                        return True
                    else:
                        if rospy.get_time() % 1.0 < 0.05:
                            rospy.loginfo("垂直平移未到达: Y误差=%.3f, 容差=%.3f", 
                                         y_error, self.goal_tolerance)
                        return False
            else:
                # 其他类型的平移，使用一般距离检查
                if remaining_dist < self.goal_tolerance:
                    rospy.loginfo("平移运动到达目标点! 剩余距离=%.3f < 容差=%.3f", 
                                 remaining_dist, self.goal_tolerance)
                    return True
                else:
                    if rospy.get_time() % 1.0 < 0.05:
                        rospy.loginfo("平移运动未到达: 剩余距离=%.3f, 容差=%.3f", 
                                     remaining_dist, self.goal_tolerance)
                    return False
        else:
            # 一般直线运动
            if remaining_dist < self.goal_tolerance:
                rospy.loginfo("一般直线运动到达目标点! 剩余距离=%.3f < 容差=%.3f", 
                             remaining_dist, self.goal_tolerance)
                return True
            else:
                if rospy.get_time() % 1.0 < 0.05:
                    rospy.loginfo("一般直线运动未到达: 剩余距离=%.3f, 容差=%.3f", 
                                 remaining_dist, self.goal_tolerance)
                return False
    
    def update_state_machine(self):
        """更新状态机"""
        current_time = rospy.get_time()
        state_duration = current_time - self.state_start_time
        
        if self.translation_state == 'ROTATE_TO_90':
            # 旋转车轮阶段：发布零速度，等待旋转完成
            if state_duration > self.wheel_rotation_time:
                rospy.loginfo("车轮旋转90度完成，开始平移运动")
                self.translation_state = 'TRANSLATING'
                self.state_start_time = current_time
                # 设置安全机制：刚进入平移状态，防止过早检测到达目标点
                self.just_started_translating = True
                self.translating_start_time = current_time
                rospy.loginfo("安全机制：刚进入平移状态，设置最小运动时间3.0秒")
                return self.create_zero_velocity()
            else:
                # 等待旋转，发布零速度
                rospy.loginfo("旋转车轮中... %.1f/%.1f秒", 
                            state_duration, self.wheel_rotation_time)
                return self.create_zero_velocity()
        
        elif self.translation_state == 'ROTATE_TO_ZERO':
            # 复位车轮阶段：发布零速度，等待复位完成
            if state_duration > self.wheel_reset_time:
                rospy.loginfo("车轮复位完成，状态机结束")
                self.translation_state = 'IDLE'
                self.translation_type = None
                self.has_goal = False
                # 确保切换到差速模式
                if self.set_kinematic_mode_service is not None:
                    try:
                        req = SetKinematicModeRequest()
                        req.mode = 'differential'
                        resp = self.set_kinematic_mode_service(req)
                        if resp.success:
                            rospy.loginfo("已确保切换到差速模式 (differential)")
                        else:
                            rospy.logwarn("确保切换到差速模式失败: %s", resp.message)
                    except rospy.ServiceException as e:
                        rospy.logerr("调用set_kinematic_mode服务失败: %s", e)
                return self.create_zero_velocity()
            else:
                # 等待复位，发布零速度
                rospy.loginfo("复位车轮中... %.1f/%.1f秒", 
                            state_duration, self.wheel_reset_time)
                return self.create_zero_velocity()
        
        elif self.translation_state == 'TRANSLATING':
            # 平移运动中，返回None表示使用正常控制
            return None
        
        # IDLE状态，返回None
        return None
    
    def create_zero_velocity(self):
        """创建零速度命令"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        return cmd_vel
    
    def holonomic_control(self, lateral_error, heading_error, 
                         lateral_error_rate, heading_error_rate, error_sign):
        """全向移动控制策略"""
        cmd_vel = Twist()
        
        # 安全防护：状态一致性检查
        # 1. 如果状态是TRANSLATING但translation_type为空，状态不一致
        # 2. 如果translation_type不为空但状态不是TRANSLATING，状态不一致
        if (self.translation_state == 'TRANSLATING' and self.translation_type is None) or \
           (self.translation_type is not None and self.translation_state != 'TRANSLATING'):
            rospy.logwarn("状态不一致：type=%s, state=%s，发布零速度并重置状态", 
                         self.translation_type, self.translation_state)
            # 强制重置状态
            if self.translation_state == 'TRANSLATING':
                self.translation_state = 'IDLE'
                # 尝试切换到差速模式
                if self.set_kinematic_mode_service is not None:
                    try:
                        req = SetKinematicModeRequest()
                        req.mode = 'differential'
                        resp = self.set_kinematic_mode_service(req)
                        if resp.success:
                            rospy.loginfo("已强制切换到差速模式 (differential)")
                        else:
                            rospy.logwarn("强制切换到差速模式失败: %s", resp.message)
                    except rospy.ServiceException as e:
                        rospy.logerr("调用set_kinematic_mode服务失败: %s", e)
            return cmd_vel
        
        # 如果是平移运动且处于平移状态
        if self.translation_type and self.translation_state == 'TRANSLATING':
            if self.translation_type == 'vertical':
                # 垂直平移：使用Y方向控制
                y_error = abs(self.vertical_line_y_error)
                y_error_sign = self.vertical_line_y_sign
                
                # 添加减速控制：接近目标点时减速
                remaining_distance = abs(self.vertical_line_y_error)
                if remaining_distance < 0.5:  # 0.5米开始减速
                    slowdown_factor = max(0.1, remaining_distance / 0.5)
                    y_speed = self.translation_kp * y_error * slowdown_factor
                else:
                    y_speed = self.translation_kp * y_error
                
                y_speed = self.clamp(y_speed, self.translation_min_speed, self.translation_max_speed) * y_error_sign
                
                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = y_speed
                cmd_vel.angular.z = 0.0
                
                if rospy.get_time() % 1.0 < 0.05:
                    rospy.loginfo("垂直平移控制: Y速度=%.3f m/s, Y误差=%.3f m, 剩余距离=%.3f, 状态=%s, 类型=%s", 
                                 y_speed, self.vertical_line_y_error, remaining_distance, 
                                 self.translation_state, self.translation_type)
            else:
                # 如果不是vertical类型，但状态是TRANSLATING，这可能是状态不一致
                rospy.logwarn("状态异常: state=TRANSLATING但type=%s，发布零速度并重置状态", self.translation_type)
                # 重置状态
                self.translation_state = 'IDLE'
                self.translation_type = None
                # 确保切换到差速模式
                if self.set_kinematic_mode_service is not None:
                    try:
                        req = SetKinematicModeRequest()
                        req.mode = 'differential'
                        resp = self.set_kinematic_mode_service(req)
                        if resp.success:
                            rospy.loginfo("已重置到差速模式 (differential)")
                        else:
                            rospy.logwarn("重置到差速模式失败: %s", resp.message)
                    except rospy.ServiceException as e:
                        rospy.logerr("调用set_kinematic_mode服务失败: %s", e)
                return cmd_vel
        else:
            # 一般运动控制 - 支持前进和倒车
            # 判断目标点是否在机器人后方（航向误差绝对值大于90度）
            if abs(heading_error) > math.pi / 2:
                # 目标在后方，直接倒车，优化控制参数以避免震荡
                rospy.loginfo("目标点在后方 (航向误差=%.2f°)，倒车模式，优化控制参数避免震荡", 
                             math.degrees(heading_error))
                
                # 重要：当目标点在后方时，我们需要调整控制逻辑以增强路径跟踪
                # 1. 倒车速度设置为负值，根据偏差调整速度，增强路径跟踪能力
                # 初始速度适中，根据横向误差调整
                if lateral_error > self.max_lateral_error:
                    base_speed = -self.max_vel_x * self.backward_speed_ratio * 0.4  # 大偏差时降低速度以专注于纠正
                elif lateral_error > self.max_lateral_error * 0.7:
                    base_speed = -self.max_vel_x * self.backward_speed_ratio * 0.5  # 中等偏差时中等速度
                else:
                    base_speed = -self.max_vel_x * self.backward_speed_ratio * 0.6  # 小偏差时正常速度，提高效率

                # 2. 倒车时增强横向控制和航向控制，确保严格按规划路径运动
                # 增强横向控制增益以快速纠正位置偏差
                backward_kp_lateral = self.kp_lateral * 1.8  # 增强横向P增益以提高响应速度
                backward_kd_lateral = self.kd_lateral * 2.2  # 增强横向D增益以抑制超调
                backward_ki_lateral = self.ki_lateral * 1.0  # 标准积分增益

                if lateral_error > self.max_lateral_error:
                    rospy.logwarn("倒车模式：横向偏差 %.3f 米超过2cm限制，启动增强紧急纠正！", lateral_error)
                    # 显著增强控制增益以快速回归路径
                    backward_kp_lateral = self.kp_lateral * 3.0  # 显著增强横向P增益
                    backward_kd_lateral = self.kd_lateral * 2.5  # 增强横向D增益
                    backward_ki_lateral = self.ki_lateral * 1.3  # 增强积分增益
                    # 降低速度以专注纠正偏差
                    base_speed = -self.max_vel_x * self.backward_speed_ratio * 0.3  # 低速度高增益纠正
                    rospy.logwarn("紧急纠正：降低速度，显著增强控制以快速回归路径！增益：P=%.1f, D=%.1f, I=%.1f",
                                 backward_kp_lateral, backward_kd_lateral, backward_ki_lateral)
                elif lateral_error > self.max_lateral_error * 0.5:  # 降低触发阈值（原0.7）
                    rospy.loginfo("倒车模式：横向偏差 %.3f 米超过1cm，增强控制", lateral_error)
                    # 增强控制增益以保持精度
                    backward_kp_lateral = self.kp_lateral * 2.2  # 增强横向P增益
                    backward_kd_lateral = self.kd_lateral * 1.8  # 增强横向D增益
                    backward_ki_lateral = self.ki_lateral * 1.1  # 适度增强积分增益
                    # 中等速度
                    base_speed = -self.max_vel_x * self.backward_speed_ratio * 0.45
                
                # 添加死区：当横向误差很小时，不进行控制调整（进一步增大死区以减少微小震荡和过度纠正）
                lateral_error_deadzone = 0.025  # 2.5厘米死区（原0.015米，进一步增大以减少微小调整和过度纠正）
                if lateral_error < lateral_error_deadzone:
                    lateral_error = 0.0
                    lateral_error_rate = 0.0
                    self.lateral_error_integral = 0.0  # 重置积分项
                
                lateral_correction = (backward_kp_lateral * lateral_error + 
                                     backward_kd_lateral * lateral_error_rate +
                                     backward_ki_lateral * self.lateral_error_integral)
                lateral_correction = self.clamp(lateral_correction, -self.max_vel_y * 0.5, self.max_vel_y * 0.5)  # 限制横向调整幅度
                
                # 3. 增强航向控制参数以提高路径跟踪能力
                # 倒车时需要更强的航向控制来保持路径跟踪
                backward_kp_heading = self.kp_heading * 0.8  # 增加角度P增益（原0.6），提高响应速度
                backward_kd_heading = self.kd_heading * 1.2  # 增加角度D增益（原1.0），增强阻尼

                # 对于航向误差，需要特别处理：倒车时航向误差可能接近π，需要规范化到[-π/2, π/2]范围
                # 当航向误差接近π时，实际上机器人是正对后方，这是理想的倒车方向
                # 我们需要将航向误差映射到更适合控制的范围
                normalized_heading_error = heading_error
                if abs(normalized_heading_error) > math.pi / 2:
                    # 如果航向误差大于90度，调整到[-90, 90]度范围内
                    if normalized_heading_error > 0:
                        normalized_heading_error = math.pi - normalized_heading_error
                    else:
                        normalized_heading_error = -math.pi - normalized_heading_error

                # 重要修复：对于四轮差速机器人，不能直接横向移动，需要将横向误差转换为额外的航向调整
                # 倒车时需要更强的航向调整来保持路径跟踪
                # 横向误差到航向调整的转换系数（增强以提高路径跟踪能力）
                lateral_to_heading_gain = 1.2  # 增强横向误差转换为航向调整的增益（原0.8），提高路径跟踪响应
                # 倒车时误差符号可能需要反转：正误差需要负航向调整
                additional_heading_from_lateral = lateral_error * lateral_to_heading_gain * -error_sign  # 添加负号反转方向

                # 确保附加的航向调整不会过大但足够有效
                max_additional_heading = 0.2  # 增加最大附加航向调整（原0.1），增强纠正能力
                additional_heading_from_lateral = self.clamp(additional_heading_from_lateral, -max_additional_heading, max_additional_heading)

                # 合并航向误差和横向误差转换的航向调整
                total_heading_adjustment = normalized_heading_error + additional_heading_from_lateral

                # 添加航向误差死区（适度减小死区以提高路径跟踪精度）
                heading_error_deadzone = 0.05  # 约2.9度死区（原4.6度），提高敏感性
                if abs(total_heading_adjustment) < heading_error_deadzone:
                    total_heading_adjustment = 0.0
                    heading_error_rate = 0.0

                angular_correction = (backward_kp_heading * total_heading_adjustment +
                                     backward_kd_heading * heading_error_rate)
                max_backward_angular = 0.2  # 增加最大角度调整幅度（原0.15），增强纠正能力
                angular_correction = self.clamp(angular_correction, -max_backward_angular, max_backward_angular)
                
                # 4. 发布倒车命令
                # 重要：对于四轮差速机器人，不能直接横向移动，将linear.y设为0
                cmd_vel.linear.x = base_speed
                cmd_vel.linear.y = 0.0  # 四轮差速模式不支持横向移动
                cmd_vel.angular.z = angular_correction
                
                # 记录详细的调试信息
                rospy.loginfo("倒车纠正控制（四轮差速）: 横向误差=%.3f m, 误差符号=%d, 附加航向调整=%.3f rad, 总航向调整=%.3f rad, 速度=%.3f m/s", 
                             lateral_error, error_sign, additional_heading_from_lateral, total_heading_adjustment, base_speed)
                
                rospy.loginfo("倒车控制: 速度=%.3f m/s, 横向校正=%.3f, 角度校正=%.3f rad/s, 规范化航向误差=%.2f°, 原始航向误差=%.2f°", 
                             base_speed, lateral_correction, angular_correction, 
                             math.degrees(normalized_heading_error), math.degrees(heading_error))
                return cmd_vel
            else:
                # 目标在前方，正常前进 - 优化控制参数以增强稳定性
                base_speed = self.max_vel_x
                
                # 偏差检测与紧急纠正：当横向偏差接近或超过2cm限制时增强控制
                # 检查当前横向偏差是否超过或接近最大允许偏差
                if lateral_error > self.max_lateral_error:
                    rospy.logwarn("横向偏差 %.3f 米超过2cm限制，启动紧急纠正！", lateral_error)
                    # 显著增强控制增益以快速纠正偏差
                    forward_kp_lateral = self.emergency_kp_lateral  # 使用紧急P增益
                    forward_kd_lateral = self.emergency_kd_lateral  # 使用紧急D增益
                    forward_ki_lateral = self.ki_lateral * 1.5  # 增加积分增益
                    # 降低前进速度以专注于偏差纠正
                    base_speed = self.max_vel_x * 0.5
                elif lateral_error > self.max_lateral_error * 0.7:
                    rospy.loginfo("横向偏差 %.3f 米接近2cm限制，增强控制", lateral_error)
                    # 适度增强控制增益
                    forward_kp_lateral = self.kp_lateral * 2.2
                    forward_kd_lateral = self.kd_lateral * 2.0
                    forward_ki_lateral = self.ki_lateral * 1.5
                    # 稍微降低速度
                    base_speed = self.max_vel_x * 0.7
                else:
                    # 正常情况，使用标准优化参数
                    forward_kp_lateral = self.kp_lateral * 0.8  # 稍减小P增益
                    forward_kd_lateral = self.kd_lateral * 1.2  # 稍增加D增益
                    forward_ki_lateral = self.ki_lateral * 0.7  # 减小积分增益
                    
                    # 检查是否处于初始阶段（前2秒）- 如果是，应用增强增益
                    if self.initial_phase_start_time is not None:
                        current_time = rospy.get_time()
                        initial_phase_elapsed = current_time - self.initial_phase_start_time
                        if initial_phase_elapsed < self.initial_phase_duration:
                            phase_factor = 1.0 - (initial_phase_elapsed / self.initial_phase_duration)  # 从1到0
                            forward_kp_lateral *= (1.0 + (self.initial_phase_kp_multiplier - 1.0) * phase_factor)
                            forward_kd_lateral *= (1.0 + (self.initial_phase_kd_multiplier - 1.0) * phase_factor)
                            forward_ki_lateral *= (1.0 + (self.initial_phase_ki_multiplier - 1.0) * phase_factor)
                            rospy.loginfo("初始阶段增强: 已过 %.1f/%.1f 秒, P增益增强因子: %.2f, D增益增强因子: %.2f, I增益增强因子: %.2f", 
                                         initial_phase_elapsed, self.initial_phase_duration,
                                         1.0 + (self.initial_phase_kp_multiplier - 1.0) * phase_factor,
                                         1.0 + (self.initial_phase_kd_multiplier - 1.0) * phase_factor,
                                         1.0 + (self.initial_phase_ki_multiplier - 1.0) * phase_factor)
                
                # 根据剩余距离调整速度，平滑减速
                remaining_distance = getattr(self, 'remaining_distance', float('inf'))
                if remaining_distance < 2.0:  # 2米开始减速
                    slowdown_factor = max(0.3, remaining_distance / 2.0)  # 保持最小速度30%
                    base_speed *= slowdown_factor
                elif remaining_distance < 0.5:  # 0.5米内进一步减速
                    slowdown_factor = max(0.15, remaining_distance / 0.5)
                    base_speed *= slowdown_factor
                
                # 添加横向误差死区，减少微小震荡（更小的死区以确保2cm精度）
                forward_lateral_error_deadzone = 0.008  # 0.8厘米死区（原1.5厘米）
                if lateral_error < forward_lateral_error_deadzone:
                    lateral_error = 0.0
                    lateral_error_rate = 0.0
                    self.lateral_error_integral = 0.0  # 重置积分项
                
                # 添加航向误差死区，减少角度微小调整
                forward_heading_error_deadzone = 0.01  # 约0.57度死区（减小以提高敏感性）
                if abs(heading_error) < forward_heading_error_deadzone:
                    heading_error = 0.0
                    heading_error_rate = 0.0
                    self.heading_error_integral = 0.0  # 重置积分项
                
                # 重要修复：对于四轮差速机器人，不能直接横向移动，需要将横向误差转换为额外的航向调整
                # 横向误差越大，需要的航向调整也越大，以便机器人通过曲线运动纠正位置
                # 关键修复：前进时横向误差到航向调整的转换需要正确符号
                # 如果机器人在路径右侧（正横向误差），需要向左转（负航向调整）才能回到路径
                # 如果机器人在路径左侧（负横向误差），需要向右转（正航向调整）才能回到路径
                # 因此需要添加负号：additional_heading_from_lateral = -lateral_error * lateral_to_heading_gain * error_sign
                # 但是注意：lateral_error总是正数（绝对值），error_sign表示方向
                # 所以实际应该是：additional_heading_from_lateral = -lateral_to_heading_gain * lateral_error * error_sign
                lateral_to_heading_gain = 1.0  # 降低横向误差转换为航向调整的增益，减少超调（原1.5）
                additional_heading_from_lateral = -lateral_to_heading_gain * lateral_error * error_sign  # 添加负号反转方向

                # 确保附加的航向调整不会过大
                max_additional_heading = 0.25  # 最大附加航向调整（弧度）
                additional_heading_from_lateral = self.clamp(additional_heading_from_lateral, -max_additional_heading, max_additional_heading)

                # 合并航向误差和横向误差转换的航向调整
                total_heading_adjustment = heading_error + additional_heading_from_lateral
                rospy.loginfo("前进航向调整：航向误差=%.3f rad, 横向误差=%.3f m, 误差符号=%d, 附加航向调整=%.3f rad, 总航向调整=%.3f rad",
                             heading_error, lateral_error, error_sign, additional_heading_from_lateral, total_heading_adjustment)
                
                # 航向控制参数（增强航向控制，提高对角度偏差的敏感性）
                if lateral_error > self.max_lateral_error:
                    # 当横向偏差较大时，显著增强航向控制以快速纠正方向
                    forward_kp_heading = self.kp_heading * 0.9  # 显著增加角度P增益
                    forward_kd_heading = self.kd_heading * 1.2  # 显著增加角度D增益
                    forward_ki_heading = self.ki_heading * 0.8  # 适当增加积分增益
                    rospy.logwarn("前进模式紧急纠正：横向误差%.3f米超过2cm，增强航向控制增益！", lateral_error)
                elif lateral_error > self.max_lateral_error * 0.7:
                    forward_kp_heading = self.kp_heading * 0.7  # 增加角度P增益
                    forward_kd_heading = self.kd_heading * 1.0  # 增加角度D增益
                    forward_ki_heading = self.ki_heading * 0.6  # 适当增加积分增益
                    rospy.loginfo("前进模式增强纠正：横向误差%.3f米接近2cm，适度增强航向控制", lateral_error)
                else:
                    forward_kp_heading = self.kp_heading * 0.5  # 标准角度P增益
                    forward_kd_heading = self.kd_heading * 0.8  # 标准角度D增益
                    forward_ki_heading = self.ki_heading * 0.4  # 减小角度积分增益
                
                # 对于四轮差速机器人，计算横向误差作为监控用途，但不用于直接控制
                # 横向误差将被转换为航向调整
                lateral_correction = (forward_kp_lateral * lateral_error + 
                                     forward_kd_lateral * lateral_error_rate +
                                     forward_ki_lateral * self.lateral_error_integral)
                lateral_correction = self.clamp(lateral_correction, -self.max_vel_y * 0.8, self.max_vel_y * 0.8)  # 限制横向调整幅度（仅用于监控）
                
                angular_correction = (forward_kp_heading * total_heading_adjustment + 
                                     forward_kd_heading * heading_error_rate +
                                     forward_ki_heading * self.heading_error_integral)
                angular_correction = self.clamp(angular_correction, -self.max_vel_theta * 0.8, self.max_vel_theta * 0.8)  # 增加角度调整幅度限制
                
                # 重要：对于四轮差速机器人，不能直接横向移动，将linear.y设为0
                cmd_vel.linear.x = base_speed
                cmd_vel.linear.y = 0.0  # 四轮差速模式不支持横向移动
                cmd_vel.angular.z = angular_correction
                
                # 记录详细的调试信息
                rospy.loginfo("前进纠正控制（四轮差速）: 横向误差=%.3f m, 误差符号=%d, 附加航向调整=%.3f rad, 总航向调整=%.3f rad, 角度校正=%.3f rad/s", 
                             lateral_error, error_sign, additional_heading_from_lateral, total_heading_adjustment, angular_correction)
                
                # 调试日志
                if rospy.get_time() % 1.0 < 0.05:
                    rospy.loginfo("前进控制: 速度=%.3f m/s, 横向校正=%.3f, 角度校正=%.3f rad/s, 横向误差=%.3f, 航向误差=%.2f°, 剩余距离=%.2f", 
                                 base_speed, lateral_correction, angular_correction, 
                                 lateral_error, math.degrees(heading_error), remaining_distance)
        
        return cmd_vel
    
    def publish_line_marker(self):
        """发布起点到终点的连线（Marker）- 只有当起点和终点不同时才发布"""
        # 检查起点和终点是否相同
        if abs(self.start_point[0] - self.end_point[0]) < 1e-6 and abs(self.start_point[1] - self.end_point[1]) < 1e-6:
            rospy.logdebug("起点和终点相同，跳过发布LINE_STRIP标记")
            return
        
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "simple_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        from geometry_msgs.msg import Point
        start_point = Point()
        start_point.x = self.start_point[0]
        start_point.y = self.start_point[1]
        start_point.z = 0.0
        
        end_point = Point()
        end_point.x = self.end_point[0]
        end_point.y = self.end_point[1]
        end_point.z = 0.0
        
        marker.points.append(start_point)
        marker.points.append(end_point)
        
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0)
        
        self.line_pub.publish(marker)
    
    def clamp(self, value, min_val, max_val):
        """限制值在[min_val, max_val]范围内"""
        return max(min_val, min(max_val, value))
    
    def control_loop(self):
        """主控制循环"""
        # 获取机器人位姿
        x, y, yaw = self.get_robot_pose()
        if x is None:
            # 发布零速度
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        # 状态一致性检查：如果translation_type为None但状态是TRANSLATING，强制重置
        if self.translation_type is None and self.translation_state == 'TRANSLATING':
            rospy.logwarn("状态不一致：type=None但state=TRANSLATING，强制重置状态到IDLE")
            self.translation_state = 'IDLE'
            # 确保切换到差速模式
            if self.set_kinematic_mode_service is not None:
                try:
                    req = SetKinematicModeRequest()
                    req.mode = 'differential'
                    resp = self.set_kinematic_mode_service(req)
                    if resp.success:
                        rospy.loginfo("已强制切换到差速模式 (differential)")
                    else:
                        rospy.logwarn("强制切换到差速模式失败: %s", resp.message)
                except rospy.ServiceException as e:
                    rospy.logerr("调用set_kinematic_mode服务失败: %s", e)
        
        # 如果没有目标点，停止运动
        if not self.has_goal:
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            self.clear_path()
            return
        
        # 检查是否到达目标
        goal_reached = self.is_goal_reached(x, y)
        
        # 添加详细调试信息
        if rospy.get_time() % 0.5 < 0.05:
            rospy.loginfo("目标检查: 到达=%s, 类型=%s, 状态=%s, Y误差=%.3f, 位置=(%.3f,%.3f), 目标=(%.3f,%.3f)", 
                         goal_reached, self.translation_type, self.translation_state,
                         self.vertical_line_y_error, x, y, self.end_point[0], self.end_point[1])
        
        if goal_reached:
            rospy.loginfo("到达目标点！当前位置: (%.3f, %.3f), 目标点: (%.3f, %.3f)", 
                         x, y, self.end_point[0], self.end_point[1])
            rospy.loginfo("平移状态: type=%s, state=%s, y_error=%.3f", 
                         self.translation_type, self.translation_state, self.vertical_line_y_error)
            
            # 如果是平移运动（无论当前状态），启动复位流程
            if self.translation_type:
                # 额外检查：如果刚进入平移状态（安全期内），不应该检测到达目标点
                if self.just_started_translating:
                    rospy.logwarn("安全机制：刚进入平移状态时不应检测到达目标点，忽略此次到达检测")
                    # 清除刚进入标志，避免一直忽略
                    self.just_started_translating = False
                    # 不执行复位，继续控制
                else:
                    rospy.loginfo("平移运动到达目标点，启动车轮复位流程。类型: %s, 当前状态: %s", 
                                 self.translation_type, self.translation_state)
                    self.finish_translation_state_machine()
                    cmd_vel = Twist()
                    self.cmd_vel_pub.publish(cmd_vel)
                    return
            else:
                rospy.loginfo("非平移运动到达目标点，直接清除目标")
                self.has_goal = False
                # 确保状态重置
                if self.translation_state != 'IDLE':
                    rospy.loginfo("重置状态到IDLE: %s -> IDLE", self.translation_state)
                    self.translation_state = 'IDLE'
                    # 确保切换到差速模式
                    if self.set_kinematic_mode_service is not None:
                        try:
                            req = SetKinematicModeRequest()
                            req.mode = 'differential'
                            resp = self.set_kinematic_mode_service(req)
                            if resp.success:
                                rospy.loginfo("已确保切换到差速模式 (differential)")
                            else:
                                rospy.logwarn("确保切换到差速模式失败: %s", resp.message)
                        except rospy.ServiceException as e:
                            rospy.logerr("调用set_kinematic_mode服务失败: %s", e)
                
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                return
        
        # 更新状态机
        state_cmd_vel = self.update_state_machine()
        if state_cmd_vel is not None:
            # 状态机返回特定速度命令（如旋转等待阶段）
            self.cmd_vel_pub.publish(state_cmd_vel)
            return
        
        # 记录路径点
        self.record_path_point(x, y)
        
        # 计算偏差
        lateral_error, error_sign = self.calculate_lateral_error(x, y)
        
        # 对于平移运动，航向误差设为0
        if self.translation_type and self.translation_state == 'TRANSLATING':
            heading_error = 0.0
        else:
            heading_error = self.line_angle - yaw
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
        
        # 计算剩余距离
        self.remaining_distance = self.calculate_remaining_distance(x, y)
        
        # 计算偏差变化率
        current_time = rospy.get_time()
        dt = current_time - self.prev_time
        if dt > 0:
            lateral_error_rate = (lateral_error - self.prev_lateral_error) / dt
            heading_error_rate = (heading_error - self.prev_heading_error) / dt
            
            # 更新积分项
            self.lateral_error_integral += lateral_error * dt
            self.heading_error_integral += heading_error * dt
            
            self.lateral_error_integral = self.clamp(self.lateral_error_integral, 
                                                    -self.integral_limit, self.integral_limit)
            self.heading_error_integral = self.clamp(self.heading_error_integral,
                                                    -self.integral_limit, self.integral_limit)
        else:
            lateral_error_rate = 0.0
            heading_error_rate = 0.0
        
        # 保存当前偏差和时间
        self.prev_lateral_error = lateral_error
        self.prev_heading_error = heading_error
        self.prev_time = current_time
      
        
        # 检查是否处于暂停状态
        with self.pause_lock:
            is_paused = self.is_paused
        
        if is_paused:
            # 如果暂停，发布零速度
            cmd_vel = Twist()
            rospy.loginfo_throttle(1.0, "导航暂停中，发送零速度命令")
        else:
            # 根据控制模式选择控制策略
            if self.control_mode == 'holonomic':
                cmd_vel = self.holonomic_control(lateral_error, heading_error,
                                               lateral_error_rate, heading_error_rate,
                                               error_sign)
            else:
                # 简化处理，只实现holonomic模式
                cmd_vel = Twist()
        
        # 发布速度指令
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 记录日志
        if current_time % 1.0 < 0.05:
            state_info = f"状态: {self.translation_state}" if self.translation_type else "状态: 一般运动"
            rospy.loginfo("%s, 位置: (%.2f, %.2f), 偏航: %.2f, 横向偏差: %.3f, 航向偏差: %.3f, 剩余距离: %.2f",
                         state_info, x, y, yaw, lateral_error, heading_error, self.remaining_distance)
    
    def record_path_point(self, x, y):
        """记录路径点"""
        from geometry_msgs.msg import Point
        
        current_point = Point()
        current_point.x = x
        current_point.y = y
        current_point.z = 0.0
        
        if self.last_recorded_point is None:
            self.path_points.append(current_point)
            self.last_recorded_point = current_point
            return True
        
        dx = current_point.x - self.last_recorded_point.x
        dy = current_point.y - self.last_recorded_point.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance > self.min_record_distance:
            self.path_points.append(current_point)
            self.last_recorded_point = current_point
            
            if len(self.path_points) > self.max_path_points:
                self.path_points = self.path_points[-self.max_path_points:]
            
            return True
        
        return False
    
    def clear_path(self):
        """清空路径记录"""
        self.path_points = []
        self.last_recorded_point = None
    
    def handle_pause_navigation(self, req):
        """处理暂停导航服务请求"""
        with self.pause_lock:
            old_state = self.is_paused
            self.is_paused = req.pause
            
            if self.is_paused:
                # 暂停时，立即发送零速度命令
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.loginfo("导航已暂停，发送零速度命令")
            else:
                rospy.loginfo("导航已继续")
            
            message = f"导航已{'暂停' if self.is_paused else '继续'}"
            rospy.loginfo(f"暂停服务响应: {message}")
            return PauseNavigationResponse(success=True, message=message)
    
    def shutdown_hook(self):
        """关闭钩子：节点停止时发送零速度"""
        rospy.loginfo("控制器正在关闭，发送零速度命令...")
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        # 短暂等待确保命令发送出去
        rospy.sleep(0.1)
        rospy.loginfo("零速度命令已发送，控制器关闭完成")

def main():
    """主函数"""
    try:
        controller = DynamicGoalLineControllerWithWheelRotation()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
