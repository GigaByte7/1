# 导航系统

## 步骤一: 启动激光雷达

``` bash
cd lidar_ws/
roslaunch livox_ros_driver2 msg_MID360.launch
```

## 步骤二: 重定位
``` bash
roslaunch fast_lio_localization sentry_localize.launch
# 使用rviz发布初始位姿
```

## 步骤三: 导航
``` bash
roslaunch sentry_nav path_navigation_enhanced_dynamic.launch
# 添加可视化话题

# 使用命令朝 /track_points/goal 话题发布一个路径组名 path_group_name 以及死区半径 dead_zone_radius 可以让其自动沿着路径组 path_beta 进行逐点导航。本质是逐点发布导航点到 /move_base_simple/goal 话题中让 move_base 完成路径规划功能，期间实时监听 base_link 是否到达死区内，如果到达则发布下一个点
rostopic pub /track_points/goal sentry_nav/PathNavigationActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  path_group_name: 'path_a'
  dead_zone_radius: 0.2"

# 使用rviz给定一个目标点
