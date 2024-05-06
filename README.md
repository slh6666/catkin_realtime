# catkin_realtime
用于小黄车完整算法的规划包，使用的是最小碰撞风险算法

## 环境配置
sudo apt install ros-<ROS版本>-navigation（eg：sudo apt install ros-noetic-navigation）
sudo apt install ros-<ROS版本>-gmapping
sudo apt install ros-<ROS版本>-map-server
如果需要使用teb算法
sudo apt-get install ros-<ROS版本>-teb-local-planner  (安装包)
rosdep install  teb_local_planner（安装依赖）

## 建图与保存（waffle可以换成burger，waffle_pi）
1. 仿真环境
cd <workspace> (eg: cd catkin_turtlebot3)
export TURTLEBOT3_MODEL=waffle
source devel/setup.sh
roslaunch turtlebot3_gazebo turtlebot3_world.launch

2. slam
cd <workspace> 
export TURTLEBOT3_MODEL=waffle
source devel/setup.sh
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

3. 键控(键控时需要把鼠标点击在相应终端上)
cd <workspace> 
export TURTLEBOT3_MODEL=waffle
source devel/setup.sh
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

4. 地图保存（需要指定路径）
rosrun map_server map_saver -f ~/map 

## 导航

1. 仿真环境
cd <workspace> 
export TURTLEBOT3_MODEL=waffle
source devel/setup.sh
roslaunch turtlebot3_gazebo turtlebot3_world.launch

2. Move_Base (切换不同算法可以在turtlebot3_navigation.launch中更改move_base_dwa.launch为move_base.launch或move_base_teb.launch)
cd <workspace> 
export TURTLEBOT3_MODEL=waffle
source devel/setup.sh
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml（需要和地图保存的路径保存一致）

## 实时建图与导航（需要修改bash内路径）
cd <workspace> 
bash self_nav.bash

## 一键导航（需要修改bash内路径）
cd <workspace> 
bash run.bash
