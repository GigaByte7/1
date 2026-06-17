#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 与6-11相同

import rospy
import math
from geometry_msgs.msg import Twist
from ldl_msg.msg import RemoteControl
from vel_pkg.srv import SetKinematicMode, SetKinematicModeResponse

class TwistToCarry:
    def __init__(self):
        rospy.init_node('twist_to_carry', anonymous=True)
        
        # 订阅cmd_vel话题，通常由导航栈发布
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.twist_callback)
        
        # 发布到自定义话题/RemoteControl，使用RemoteControl消息类型
        self.pub = rospy.Publisher('/RemoteControl', RemoteControl, queue_size=10)
        
        # 转换参数
        # 更新缩放因子以支持3.0m/s线速度和1.5rad/s角速度（匹配base_local_planner_params.yaml中大幅提高后的设置）
        self.linear_scale = rospy.get_param('~linear_scale', 1667.0)  # 大幅提高线速度缩放因子，匹配新的最大速度3.0m/s，支持高速运动
        self.angular_scale = rospy.get_param('~angular_scale', 1250.0)  # 大幅提高角速度缩放因子，匹配新的最大角速度1.5rad/s，支持高速转向
        # 调整死区阈值，优化高速运动性能
        self.deadzone_threshold = rospy.get_param('~deadzone_threshold', 0.003)  # 进一步降低线速度死区阈值，提高高速运动精度
        self.min_speed_threshold = rospy.get_param('~min_speed_threshold', 0.002)  # 进一步降低最小速度阈值，提高低速控制精度
        self.angular_deadzone_threshold = rospy.get_param('~angular_deadzone_threshold', 0.03)  # 进一步降低角速度死区阈值，提高高速转向精度
        
        # 运动学模式：four_wheel_diff（四轮差速/阿克曼）或 lateral（左右平移）
        # 兼容旧参数：differential -> four_wheel_diff，omni -> lateral
        default_mode = rospy.get_param('~kinematic_mode', 'four_wheel_diff')
        self.kinematic_mode = self._normalize_mode(default_mode)
        
        # 默认模式保持阿克曼(0)；可通过参数覆盖
        self.default_mode = rospy.get_param('~default_mode', 0)
        # SW6：0=遥控/运行模式；2=导航模式（platform_control 中 navigation_flag 会屏蔽 conMontor）
        # 若底盘不动，请尝试将该参数设为 0，使 conMontor 生效。
        # 根据问题描述，底盘在目标点附近旋转但不运动，尝试将SW6设为0使conMontor生效
        # 默认使用0（运行模式），如果导航栈需要，可以通过参数设置为2
        self.default_state = rospy.get_param('~default_state', 0)  # 设置为0以启用conMontor
        
        # 平滑滤波参数 - 针对高速运动优化
        self.filter_cutoff_freq = rospy.get_param('~filter_cutoff_freq', 8.0)  # 进一步提高截止频率，大幅减少滤波效果，提高高速响应性
        # 大幅提高加速度限制，支持快速加速到高速
        self.max_accel_linear = rospy.get_param('~max_accel_linear', 6.0)  # 大幅提高线加速度限制，支持快速加速到3.0m/s
        self.max_accel_angular = rospy.get_param('~max_accel_angular', 3.0)  # 大幅提高角加速度限制，支持快速转向到1.5rad/s
        self.enable_lowpass_filter = rospy.get_param('~enable_lowpass_filter', False)  # 禁用低通滤波，减少延迟，提高高速响应性
        
        # 车辆参数（与platform_control保持一致）
        self.wheelbase = rospy.get_param('~wheelbase', 1.47)    # m，对应 DoubleAckermanSolver 长度
        self.track = rospy.get_param('~track', 0.83)            # m，对应 DoubleAckermanSolver 宽度（未直接使用）
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.175)  # m，来源platform_control

        # 发布频率 (Hz)
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)
        
        # 当前滤波后的速度
        self.filtered_linear_x = 0.0
        self.filtered_linear_y = 0.0
        self.filtered_angular_z = 0.0
        
        # 转向角平滑（大幅提高转向响应速度以支持高速运动）
        self.filtered_steering_angle = 0.0
        self.max_steering_rate = rospy.get_param('~max_steering_rate', 0.35)  # 大幅提高最大转向角变化率，使转向响应更快，支持高速转向
        
        # 目标速度（来自最新Twist消息）
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        # 上一次更新时间
        self.last_time = rospy.Time.now()
        
        # 低通滤波器系数计算
        # 一阶低通滤波器: alpha = dt / (dt + 1/(2*pi*fc))
        # 我们将在每次迭代中根据实际dt计算alpha
        self.filter_fc = self.filter_cutoff_freq
        
        # 动态参数服务
        self.dynamic_param_service = rospy.Service('~set_kinematic_mode', SetKinematicMode, self.handle_set_kinematic_mode)
        
        # 使用定时器以固定频率发布
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        
        rospy.loginfo("Twist to Carry converter started (with smoothing)")
        rospy.loginfo("Kinematic mode: %s", self.kinematic_mode)
        rospy.loginfo("Linear scale: %f, Angular scale: %f", self.linear_scale, self.angular_scale)
        rospy.loginfo("Deadzone threshold: %f, Min speed threshold: %f", self.deadzone_threshold, self.min_speed_threshold)
        rospy.loginfo("Filter cutoff: %f Hz, Publish rate: %f Hz, Lowpass enabled: %s", 
                     self.filter_cutoff_freq, self.publish_rate, self.enable_lowpass_filter)
        rospy.loginfo("Max acceleration - linear: %f m/s^2, angular: %f rad/s^2", self.max_accel_linear, self.max_accel_angular)
        rospy.loginfo("Max steering rate: %f rad/s, Default state (SW6): %d", self.max_steering_rate, self.default_state)
        
        # 注册关闭钩子，确保节点停止时发送零速度
        rospy.on_shutdown(self.shutdown_hook)
        
    def shutdown_hook(self):
        """关闭钩子：节点停止时发送零速度"""
        rospy.loginfo("TwistToCarry 正在关闭，发送零速度命令...")
        # 创建零速度消息
        remote_msg = RemoteControl()
        remote_msg.SW1 = 1000  # 转向中立
        remote_msg.SW2 = 1000  # 驱动停止
        remote_msg.SW3 = 1000
        remote_msg.SW4 = 0
        remote_msg.SW5 = 0     # 差速模式
        remote_msg.SW6 = self.default_state
        remote_msg.SW7 = 0
        remote_msg.SW8 = 0
        
        # 发布零速度
        self.pub.publish(remote_msg)
        # 短暂等待确保命令发送出去
        rospy.sleep(0.1)
        rospy.loginfo("零速度命令已发送，TwistToCarry 关闭完成")
        
    def handle_set_kinematic_mode(self, req):
        """处理动态模式切换服务请求"""
        normalized = self._normalize_mode(req.mode)
        if normalized:
            self.kinematic_mode = normalized
            rospy.loginfo("Kinematic mode changed to: %s", self.kinematic_mode)
            return SetKinematicModeResponse(True, "Mode changed successfully")
        else:
            return SetKinematicModeResponse(False, "Invalid mode. Use 'four_wheel_diff', 'lateral', or 'omnidirectional'")
        
    def twist_callback(self, msg):
        """接收并存储目标速度，应用死区处理"""
        # 死区处理：如果速度绝对值小于阈值，则视为0
        # 线速度使用通用死区阈值
        self.target_linear_x = msg.linear.x if abs(msg.linear.x) > self.deadzone_threshold else 0.0
        self.target_linear_y = msg.linear.y if abs(msg.linear.y) > self.deadzone_threshold else 0.0
        # 角速度使用专用死区阈值，过滤微小转向，减少频繁转动
        self.target_angular_z = msg.angular.z if abs(msg.angular.z) > self.angular_deadzone_threshold else 0.0
        
        rospy.logdebug("Target speeds: linear_x=%.3f, linear_y=%.3f, angular_z=%.3f", 
                      self.target_linear_x, self.target_linear_y, self.target_angular_z)
    
    def timer_callback(self, event):
        """定时器回调，执行滤波和发布"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            dt = 1.0 / self.publish_rate
        self.last_time = current_time
        
        # 应用加速度限制（主要平滑方法）
        self._apply_acceleration_limits(dt)
        
        # 可选：应用低通滤波（如果启用）
        if self.enable_lowpass_filter:
            self._apply_lowpass_filter(dt)
        
        # 根据运动学模式生成RemoteControl消息
        remote_msg = self._generate_remote_control_msg(dt)
        
        # 发布消息
        self.pub.publish(remote_msg)
        
        # 添加调试信息，帮助诊断不运动问题
        rospy.logdebug("Published RemoteControl: SW1=%d, SW2=%d, SW3=%d, SW5=%d, SW6=%d", 
                      remote_msg.SW1, remote_msg.SW2, remote_msg.SW3, remote_msg.SW5, remote_msg.SW6)
        
        # 如果目标速度不为0但实际速度接近0，发出警告
        if abs(self.target_linear_x) > 0.05 and abs(self.filtered_linear_x) < 0.01:
            rospy.logwarn_throttle(2.0, "警告：目标速度=%.3f m/s但实际速度=%.3f m/s（接近0），可能导致停止！SW2=%d", 
                                  self.target_linear_x, self.filtered_linear_x, remote_msg.SW2)
        
        # 如果速度不为0但SW2接近1000（无驱动），发出警告
        if abs(self.filtered_linear_x) > 0.01 and abs(remote_msg.SW2 - 1000) < 100:
            rospy.logwarn_throttle(2.0, "警告：线速度=%.3f m/s但SW2=%d（接近1000），可能导致无法运动！目标速度=%.3f", 
                                  self.filtered_linear_x, remote_msg.SW2, self.target_linear_x)
        
        # 如果角速度不为0但SW1接近1000（无转向），发出警告
        if abs(self.filtered_angular_z) > 0.01 and abs(remote_msg.SW1 - 1000) < 50:
            rospy.logwarn_throttle(2.0, "警告：角速度=%.3f rad/s但SW1=%d（接近1000），可能导致无法转向！目标角速度=%.3f", 
                                  self.filtered_angular_z, remote_msg.SW1, self.target_angular_z)
    
    def _apply_acceleration_limits(self, dt):
        """应用加速度限制，更新滤波后的速度（作为中间步骤）"""
        # 线速度x加速度限制 - 支持前进和倒车
        error_x = self.target_linear_x - self.filtered_linear_x
        max_change_x = self.max_accel_linear * dt
        if abs(error_x) > max_change_x:
            self.filtered_linear_x += math.copysign(max_change_x, error_x)
        else:
            self.filtered_linear_x = self.target_linear_x
        
        # 如果目标速度绝对值足够大，则不要轻易停止 - 支持前进和倒车
        if abs(self.target_linear_x) > 0.02:  # 如果目标速度绝对值大于0.02m/s
            # 确保滤波后的速度至少为最小速度阈值，保持方向
            if self.target_linear_x > 0:  # 前进
                if self.filtered_linear_x < 0.03:
                    self.filtered_linear_x = max(self.filtered_linear_x, 0.03)
            else:  # 倒车
                if self.filtered_linear_x > -0.03:
                    self.filtered_linear_x = min(self.filtered_linear_x, -0.03)
        # 如果目标速度很小，且当前速度也很小，直接停止（避免频繁微调）
        elif abs(self.target_linear_x) < self.min_speed_threshold * 0.5 and abs(self.filtered_linear_x) < self.min_speed_threshold * 0.5:
            self.filtered_linear_x = 0.0
            
        # 线速度y加速度限制
        error_y = self.target_linear_y - self.filtered_linear_y
        max_change_y = self.max_accel_linear * dt
        if abs(error_y) > max_change_y:
            self.filtered_linear_y += math.copysign(max_change_y, error_y)
        else:
            self.filtered_linear_y = self.target_linear_y
        
        # 如果目标速度很小，且当前速度也很小，直接停止
        if abs(self.target_linear_y) < self.min_speed_threshold * 0.5 and abs(self.filtered_linear_y) < self.min_speed_threshold * 0.5:
            self.filtered_linear_y = 0.0
            
        # 角速度z加速度限制 - 减少微小转向调整
        error_z = self.target_angular_z - self.filtered_angular_z
        max_change_z = self.max_accel_angular * dt
        if abs(error_z) > max_change_z:
            self.filtered_angular_z += math.copysign(max_change_z, error_z)
        else:
            self.filtered_angular_z = self.target_angular_z
        
        # 如果目标角速度很小，且当前角速度也很小，直接停止
        if abs(self.target_angular_z) < self.angular_deadzone_threshold * 0.8 and abs(self.filtered_angular_z) < self.angular_deadzone_threshold * 0.8:
            self.filtered_angular_z = 0.0
    
    def _apply_lowpass_filter(self, dt):
        """应用一阶低通滤波器（可选，如果已经用加速度限制可以不再滤波）"""
        # 注意：此滤波器应用于已经经过加速度限制的速度值
        # 使用目标速度与当前滤波速度的差值进行平滑
        alpha = dt / (dt + 1.0 / (2.0 * math.pi * self.filter_fc))
        
        # 对已经经过加速度限制的速度进行额外的低通滤波
        # 这里使用target值作为目标，但实际应该基于加速度限制后的值
        # 为了简化，我们直接对当前滤波值进行轻微平滑
        self.filtered_linear_x = self.filtered_linear_x + alpha * (self.target_linear_x - self.filtered_linear_x)
        self.filtered_linear_y = self.filtered_linear_y + alpha * (self.target_linear_y - self.filtered_linear_y)
        self.filtered_angular_z = self.filtered_angular_z + alpha * (self.target_angular_z - self.filtered_angular_z)
    
    def _generate_remote_control_msg(self, dt):
        """根据当前滤波后的速度生成RemoteControl消息"""
        remote_msg = RemoteControl()
        
        linear_x = self.filtered_linear_x
        linear_y = self.filtered_linear_y
        angular_z = self.filtered_angular_z
        
        if self.kinematic_mode == 'four_wheel_diff':
            # 四轮差速/阿克曼：使用 linear.x 和 angular.z
            delta_rad, wheel_omega = self._ackermann_from_twist(linear_x, angular_z)
            
            # 对转向角进行平滑处理，避免频繁转向
            delta_rad = self._smooth_steering_angle(delta_rad, dt)
            
            remote_msg.SW1 = self._encode_steering(delta_rad)   # 转向角，匹配 platform_control 公式
            remote_msg.SW2 = self._encode_wheel_omega(wheel_omega)  # 驱动角速度
            remote_msg.SW3 = 1000  # 未使用
            remote_msg.SW5 = 0     # <300 -> turn_flag = 1 (阿克曼)
        elif self.kinematic_mode == 'lateral':
            # 左右平移：使用 linear.y
            # 在lateral模式下，无论是否有速度，都要确保车轮旋转到90度位置
            # 如果速度很小，仍然需要设置车轮角度为90度
            sw1, sw2 = self._lateral_command(linear_y)
            remote_msg.SW1 = sw1
            remote_msg.SW2 = sw2
            remote_msg.SW3 = 1000
            remote_msg.SW5 = 1700  # >1300 -> turn_flag = 3 (平移模式)
            # 调试信息：记录平移模式下的命令
            rospy.logdebug("Lateral mode: linear_y=%.3f, SW1=%d, SW2=%d", linear_y, sw1, sw2)
        elif self.kinematic_mode == 'omni':
            # 全向模式：使用 linear.x, linear.y 和 angular.z
            # 所有车轮旋转到61度，然后根据速度方向驱动
            sw1, sw2 = self._omni_command(linear_x, linear_y, angular_z)
            remote_msg.SW1 = sw1
            remote_msg.SW2 = sw2
            remote_msg.SW3 = 1000
            remote_msg.SW5 = 900   # 600-1200 -> turn_flag = 2 (全向模式)
            # 调试信息：记录全向模式下的命令
            rospy.logdebug("Omni mode: linear_x=%.3f, linear_y=%.3f, angular_z=%.3f, SW1=%d, SW2=%d", 
                          linear_x, linear_y, angular_z, sw1, sw2)
        else:
            rospy.logwarn_throttle(1.0, "Unknown kinematic mode: %s, fallback four_wheel_diff", self.kinematic_mode)
            delta_rad, wheel_omega = self._ackermann_from_twist(linear_x, angular_z)
            remote_msg.SW1 = self._encode_steering(delta_rad)
            remote_msg.SW2 = self._encode_wheel_omega(wheel_omega)
            remote_msg.SW3 = 1000
            remote_msg.SW5 = 0

        # 其他字段根据底盘控制逻辑设置
        remote_msg.SW4 = 0
        remote_msg.SW6 = self.default_state  # 2 -> 导航模式，platform_control 中 SW6==2 开启导航
        remote_msg.SW7 = 0
        remote_msg.SW8 = 0
        
        return remote_msg
        
    def _clamp(self, value, min_val, max_val):
        """限制值在[min_val, max_val]范围内"""
        return max(min_val, min(value, max_val))

    def _normalize_mode(self, mode_str):
        """兼容旧模式名并返回规范化模式"""
        if mode_str in ['four_wheel_diff', 'ackermann', 'differential']:
            return 'four_wheel_diff'
        if mode_str in ['lateral', 'side', 'omni']:
            # 注意：'omni' 原来映射到 lateral，保持向后兼容
            return 'lateral'
        if mode_str in ['omnidirectional', 'all_wheel_steer']:
            # 新增全向模式，使用新名称避免冲突
            return 'omni'
        return None

    def _ackermann_from_twist(self, linear_x, angular_z):
        """
        将 cmd_vel 转换为阿克曼模型输入:
        - delta: 前轮转角 (rad)
        - omega_wheel: 驱动轮角速度 (rad/s)
        对应 platform_control 中：
            input_delta = -((SW1 - 1000) / 500 / 3 * pi)
            input_omega = (SW2 - 1000) / 15
        """
        v = linear_x
        w = angular_z

        # 若线速度过小但有角速度，为避免除零，给一个最小虚拟速度
        # 【修复】使用线速度的符号而不是角速度的符号，避免角速度为负时错误触发倒车
        # 原bug: v = math.copysign(min_v, w) 会导致机器人向后运动
        min_v = 0.15  # 最小虚拟速度
        if abs(v) < min_v and abs(w) > 0.06:  # 线速度太小但需要转向
            # 如果线速度接近零但方向未知，默认保持向前
            if abs(v) < 0.01:
                v = min_v  # 默认向前
            else:
                v = math.copysign(min_v, v)  # 保持原线速度方向（前进或倒退）


        # 转角计算：delta = atan(L * w / v)
        delta = 0.0
        if abs(v) > 1e-3 and abs(w) > 1e-3:  # 当速度和角速度都不为0时
            delta = math.atan2(self.wheelbase * w, v)
        elif abs(w) > 0.06:  # 进一步降低纯旋转阈值，提高高速转向灵敏度
            # 纯旋转时，根据角速度计算合理的转向角
            max_steering = rospy.get_param('~max_steering_angle', 0.52)
            # 根据角速度大小按比例设置转向角，提高高速转向响应
            delta = math.copysign(min(abs(w) * 0.4, max_steering), w)  # 调整比例系数，优化高速转向
        # 注意：当角速度w=0时，无论速度方向如何，转向角都应为0

        # 驱动轮角速度（近似）：线速度 -> 轮角速度
        omega_wheel = v / self.wheel_radius
        
        # 限制最大转向角度，避免在目标点附近过度转向
        max_steering_angle = rospy.get_param('~max_steering_angle', 0.52)  # 默认30度（约0.52弧度）
        delta = self._clamp(delta, -max_steering_angle, max_steering_angle)

        return delta, omega_wheel
    
    def _smooth_steering_angle(self, target_delta, dt):
        """对转向角进行平滑处理，避免频繁转向"""
        if dt <= 0:
            dt = 1.0 / self.publish_rate
        
        # 计算转向角变化率限制
        max_change = self.max_steering_rate * dt
        error = target_delta - self.filtered_steering_angle
        
        if abs(error) > max_change:
            self.filtered_steering_angle += math.copysign(max_change, error)
        else:
            self.filtered_steering_angle = target_delta
        
        # 如果目标转向角很小，且当前转向角也很小，直接归零
        if abs(target_delta) < 0.01 and abs(self.filtered_steering_angle) < 0.02:
            self.filtered_steering_angle = 0.0
        
        return self.filtered_steering_angle

    def _encode_steering(self, delta_rad):
        """
        将前轮转角编码到 SW1 (0-2000)
        platform_control 解码：input_delta = -((SW1-1000)/500/3*pi)
        反推：SW1 = 1000 - delta_rad * 500 * 3 / pi
        """
        raw = 1000 - delta_rad * 500.0 * 3.0 / math.pi
        return self._clamp(int(raw), 0, 2000)

    def _encode_wheel_omega(self, omega):
        """
        将驱动轮角速度编码到 SW2 (0-2000)
        platform_control 解码：input_omega = (SW2 - 1000) / 15
        关键：platform_control的死区为990-1010，必须确保非零速度映射到死区外
        根据警告，SW2在950-1050范围内可能导致驱动力不足，因此需要更远离1000
        """
        # 如果速度非常小，直接返回停止信号（1000）
        if abs(omega) < 0.001:
            return 1000
        
        # 计算原始编码值
        raw = 1000 + omega * 15.0
        
        # 确保非零速度能够突破死区并远离1000
        # 正向速度：确保SW2 >= 1050，负向速度：确保SW2 <= 950
        if omega > 0 and raw < 1050:
            raw = 1050
        elif omega < 0 and raw > 950:
            raw = 950
        
        # 限制在有效范围内
        return self._clamp(int(raw), 0, 2000)

    def _lateral_command(self, linear_y):
        """
        生成左右平移模式的 SW1/SW2
        platform_control 逻辑：
          - SW5>1300 -> turn_flag=3
          - SW1>1050: turn_dir=0, angle_turnmodel3=90*(SW1-1050)/450 （右向）
          - SW1<950 : turn_dir=1, angle_turnmodel3=90*(950-SW1)/450 （左向）
          - SW2>1010: wheel_flag_R=1, wheel_flag_L=2 (正向)
          - SW2<990 : wheel_flag_R=2, wheel_flag_L=1 (反向)
        
        重要修改：在lateral模式下，即使速度为0，也要确保车轮旋转到90度位置。
        否则车轮不会旋转，只会停留在0度位置。
        """
        # 关键修改：在lateral模式下，无论速度大小，都要设置车轮角度
        # 根据线性速度方向设置车轮角度为90度
        if linear_y > 0:
            # 左移：设成 turn_dir=1 -> SW1 < 950
            # 500对应大约90度 (90*(950-500)/450 = 90度)
            sw1 = 500  # 约 90 度
            rospy.loginfo("设置左移平移模式：SW1=500 (车轮左转90度)")
        elif linear_y < 0:
            # 右移：turn_dir=0 -> SW1 > 1050
            # 1500对应大约90度 (90*(1500-1050)/450 = 90度)
            sw1 = 1500  # 约 90 度
            rospy.loginfo("设置右移平移模式：SW1=1500 (车轮右转90度)")
        else:
            # 速度为0，但仍然需要设置车轮角度为90度（保持平移状态）
            # 默认设置为500（左转90度），因为通常左移更常见
            sw1 = 500
            rospy.loginfo("平移模式零速度：SW1=500 (保持车轮90度位置)")

        # 速度映射到 SW2
        # 如果速度很小（在死区范围内），设置SW2=1000（停止但不改变车轮角度）
        if abs(linear_y) < self.deadzone_threshold:
            sw2 = 1000  # 停止驱动，但车轮保持在90度位置
        else:
            sw2 = 1000 + linear_y * self.linear_scale
            sw2 = self._clamp(int(sw2), 0, 2000)
            
            # 确保非零速度能够突破死区
            if linear_y > 0 and sw2 < 1010:
                sw2 = 1010
            elif linear_y < 0 and sw2 > 990:
                sw2 = 990

        rospy.loginfo("平移模式命令：linear_y=%.3f, SW1=%d, SW2=%d", linear_y, sw1, sw2)
        return sw1, sw2
    
    def _omni_command(self, linear_x, linear_y, angular_z):
        """
        生成全向模式的 SW1/SW2
        platform_control 逻辑：
          - SW5在600-1200之间 -> turn_flag=2 (全向模式)
          - 所有车轮旋转到61度（固定角度）
          - 根据速度方向驱动车轮
          
        全向模式分析：
          - 根据 main_2.cpp 代码，全向模式时：
            Srl.TurnMotor_PositionChange_package((float)61.0,0);
            Slq.TurnMotor_PositionChange_package((float)61.0,1);
            Sll.TurnMotor_PositionChange_package((float)61.0,0);
            Srq.TurnMotor_PositionChange_package((float)61.0,1);
            
            这意味着所有车轮都旋转到61度，但方向不同（0或1）
            
          - 驱动电机控制：
            Frq.DriveMotor_SpeedChange((float)Veloicty,wheel_flag_R);
            Flq.DriveMotor_SpeedChange((float)Veloicty,wheel_flag_R);
            Frl.DriveMotor_SpeedChange((float)Veloicty,wheel_flag_R);
            Fll.DriveMotor_SpeedChange((float)Veloicty,wheel_flag_R);
            
            所有车轮使用相同的速度和方向标志
        """
        # 全向模式下，SW1用于控制车轮旋转角度
        # 根据 platform_control 代码，全向模式时 SW1 应该控制角度
        # 但实际代码中全向模式是固定61度，所以我们需要设置一个合适的SW1值
        
        # 分析：在 main_2.cpp 中，全向模式时 SW1 用于控制 omiga_car 和 angle_turnmodel3
        # 但全向模式时 turn_flag=2，不会使用 angle_turnmodel3
        # 所以我们可以设置一个默认值，比如 1000（中立位置）
        sw1 = 1000
        
        # 全向模式下，我们需要根据 linear_x, linear_y, angular_z 计算综合速度
        # 简化处理：使用 linear_x 作为主要速度，linear_y 和 angular_z 作为辅助
        # 实际上，全向模式应该支持任意方向移动，但这里简化实现
        
        # 计算综合速度大小
        speed_magnitude = math.sqrt(linear_x**2 + linear_y**2)
        
        # 计算速度方向角度（相对于机器人前方）
        if speed_magnitude > 0.001:
            direction_angle = math.atan2(linear_y, linear_x)  # 弧度
            # 将方向角度转换为车轮控制需要的参数
            # 这里简化：使用 linear_x 作为主要控制
            # 实际上需要更复杂的全向运动学转换
            pass
        
        # 对于简化实现，我们主要使用 linear_x 控制前进/后退
        # linear_y 控制左右平移，angular_z 控制旋转
        
        # 计算综合速度：结合线速度和角速度
        # 角速度转换为等效线速度：v_angular = angular_z * 0.5（假设旋转半径0.5米）
        v_angular = angular_z * 0.5
        
        # 综合速度 = 线速度 + 角速度等效线速度
        # 这里简化：使用 linear_x 作为主要速度，加上角速度分量
        combined_speed = linear_x + v_angular
        
        # 映射到 SW2
        if abs(combined_speed) < self.deadzone_threshold:
            sw2 = 1000  # 停止
        else:
            sw2 = 1000 + combined_speed * self.linear_scale
            sw2 = self._clamp(int(sw2), 0, 2000)
            
            # 确保非零速度能够突破死区
            if combined_speed > 0 and sw2 < 1010:
                sw2 = 1010
            elif combined_speed < 0 and sw2 > 990:
                sw2 = 990
        
        rospy.loginfo("全向模式命令：linear_x=%.3f, linear_y=%.3f, angular_z=%.3f, SW1=%d, SW2=%d", 
                     linear_x, linear_y, angular_z, sw1, sw2)
        return sw1, sw2
        
if __name__ == '__main__':
    try:
        converter = TwistToCarry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

