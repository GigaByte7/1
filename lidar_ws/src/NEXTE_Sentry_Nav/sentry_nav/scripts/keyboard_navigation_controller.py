#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
键盘导航控制器
使用p键暂停导航，c键继续导航
需要现有的路径导航节点提供PauseNavigation服务
"""

import rospy
import sys
import select
import termios
import tty
from sentry_nav.srv import PauseNavigation, PauseNavigationRequest

class KeyboardNavigationController:
    def __init__(self):
        rospy.init_node('keyboard_navigation_controller', anonymous=True)
        
        # 服务名称参数
        self.pause_service_name = rospy.get_param('~pause_service', '/pause_navigation')
        
        # 等待暂停服务可用
        rospy.loginfo("等待暂停导航服务: %s", self.pause_service_name)
        try:
            rospy.wait_for_service(self.pause_service_name, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("无法连接到暂停导航服务: %s", self.pause_service_name)
            rospy.logerr("请确保路径导航节点正在运行并实现了PauseNavigation服务")
            sys.exit(1)
        
        # 创建服务代理
        self.pause_service = rospy.ServiceProxy(self.pause_service_name, PauseNavigation)
        rospy.loginfo("已连接到暂停导航服务")
        
        # 保存原始终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 状态变量
        self.is_paused = False
        self.running = True
        
        # 注册关闭钩子
        rospy.on_shutdown(self.shutdown_hook)
        
        rospy.loginfo("键盘导航控制器已启动")
        rospy.loginfo("按键说明:")
        rospy.loginfo("  p - 暂停导航")
        rospy.loginfo("  c - 继续导航")
        rospy.loginfo("  q - 退出")
        rospy.loginfo("当前状态: 运行中")
        
    def get_key(self):
        """获取单个按键输入（非阻塞）"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def call_pause_service(self, pause):
        """调用暂停/继续服务"""
        try:
            req = PauseNavigationRequest()
            req.pause = pause
            resp = self.pause_service(req)
            if resp.success:
                if pause:
                    rospy.loginfo("导航已暂停")
                    self.is_paused = True
                else:
                    rospy.loginfo("导航已继续")
                    self.is_paused = False
            else:
                rospy.logwarn("服务调用成功但返回失败: %s", resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: %s", e)
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown() and self.running:
            key = self.get_key()
            
            if key == 'p' or key == 'P':
                if not self.is_paused:
                    self.call_pause_service(True)
                else:
                    rospy.loginfo("导航已处于暂停状态")
                    
            elif key == 'c' or key == 'C':
                if self.is_paused:
                    self.call_pause_service(False)
                else:
                    rospy.loginfo("导航已处于运行状态")
                    
            elif key == 'q' or key == 'Q':
                rospy.loginfo("收到退出命令")
                self.running = False
                break
                
            elif key:
                rospy.loginfo("未知按键: '%s'，请使用 p/c/q", key)
                rospy.loginfo("p:暂停, c:继续, q:退出")
            
            rate.sleep()
        
        rospy.loginfo("键盘导航控制器正在关闭")
    
    def shutdown_hook(self):
        """关闭钩子：恢复终端设置"""
        rospy.loginfo("恢复终端设置...")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.running = False

def main():
    try:
        controller = KeyboardNavigationController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("发生异常: %s", e)
    finally:
        # 确保终端设置恢复
        import termios
        import sys
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, controller.settings)
        except:
            pass

if __name__ == '__main__':
    main()