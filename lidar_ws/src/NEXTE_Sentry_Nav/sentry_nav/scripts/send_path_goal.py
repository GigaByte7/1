#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import sys
from sentry_nav.msg import PathNavigationAction, PathNavigationGoal

def send_path_goal(path_group_name="tj5", dead_zone_radius=0.06):
    """发送路径导航目标"""
    
    rospy.init_node('send_path_goal', anonymous=True)
    
    # 等待Action服务器
    client = actionlib.SimpleActionClient('/track_points', PathNavigationAction)
    rospy.loginfo("等待Action服务器...")
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("Action服务器不可用")
        return False
    
    # 创建目标
    goal = PathNavigationGoal()
    goal.path_group_name = path_group_name
    goal.dead_zone_radius = dead_zone_radius
    
    rospy.loginfo("发送导航目标:")
    rospy.loginfo("  路径组: %s", goal.path_group_name)
    rospy.loginfo("  死区半径: %.3f", goal.dead_zone_radius)
    
    # 发送目标
    client.send_goal(goal)
    rospy.loginfo("目标已发送，等待动态目标点设置...")
    rospy.loginfo("请在RViz中使用2D Nav Goal工具设置新目标点")
    
    # 等待结果
    rospy.loginfo("等待导航完成...")
    client.wait_for_result(rospy.Duration(300.0))  # 5分钟超时
    
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        result = client.get_result()
        rospy.loginfo("导航成功: %s", result.message)
        return True
    else:
        state = client.get_state()
        rospy.logerr("导航失败，状态: %s", state)
        return False

if __name__ == '__main__':
    # 解析命令行参数
    path_group = "tj5"
    dead_zone = 0.06
    
    if len(sys.argv) > 1:
        path_group = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            dead_zone = float(sys.argv[2])
        except ValueError:
            rospy.logerr("死区半径必须是数字")
            sys.exit(1)
    
    try:
        success = send_path_goal(path_group, dead_zone)
        if success:
            rospy.loginfo("路径导航目标发送成功")
            sys.exit(0)
        else:
            rospy.logerr("路径导航目标发送失败")
            sys.exit(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
        sys.exit(1)
    except Exception as e:
        rospy.logerr("发生异常: %s", e)
        sys.exit(1)