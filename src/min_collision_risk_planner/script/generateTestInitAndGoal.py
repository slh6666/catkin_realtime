#! /usr/bin/python
#! -*- coding: utf-8 -*-

"""

为场景批量化测试生成起点和终点
author: flztiii

"""

import rospkg
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import numpy as np
import os
import shutil
from rospy.exceptions import ROSException

# 全局变量
COSTMAP_OBTAINED_FLAG_ = False  # 是否接收到地图
map_left_, map_right_ = 0.0, 0.0  # 地图左右边界
map_bottom_, map_top_ = 0.0, 0.0  # 地图上下边界

# 栅格地图回调函数
def costmapCallback(msg):
    global COSTMAP_OBTAINED_FLAG_, map_left_, map_right_, map_bottom_, map_top_
    # 获得地图边界
    map_left_ = msg.info.origin.position.x
    map_right_ = msg.info.origin.position.x + msg.info.width * msg.info.resolution
    map_bottom_ = msg.info.origin.position.y
    map_top_ = msg.info.origin.position.y + msg.info.height * msg.info.resolution
    # 修改标志位
    COSTMAP_OBTAINED_FLAG_ = True

def main():
    global COSTMAP_OBTAINED_FLAG_, map_left_, map_right_, map_bottom_, map_top_
    # 初始化ros
    rospy.init_node("random_init_goal_generator_node")
    # 订阅栅格地图消息
    costmap_topic = rospy.get_param("~costmap_topic")
    costmap_sub_ = rospy.Subscriber(costmap_topic, OccupancyGrid, costmapCallback, queue_size=1)
    # 订阅定位结果
    odom_topic = rospy.get_param("~odom_topic")
    # 订阅规划结果消息
    planning_result_topic = rospy.get_param("~planning_result_topic")
    # 发布起点和终点消息
    init_point_pub_ = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    goal_point_pub_ = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
    # 得到测试场景名称
    scene_name = rospy.get_param("~scene_name")
    # 生成数量
    test_num = rospy.get_param("~test_num", 100)
    # 得到文件保存目录
    root_path = rospkg.RosPack().get_path("min_collision_risk_planner")
    root_path += "/logs/" + scene_name + "_test_pairs/"
    # 判断文件夹是否存在
    if not os.path.exists(root_path):
        # 如果不存在则创建
        os.mkdir(root_path)
    else:
        # 如果存在则清空文件夹
        shutil.rmtree(root_path)
        os.mkdir(root_path)
    # 清空之前的文件
    # 等待获取栅格地图信息
    while not rospy.is_shutdown():
        if COSTMAP_OBTAINED_FLAG_:
            break
        print("wait for costmap")
        rospy.Rate(1).sleep()
    print("costmap obtained")
    # 得到栅格地图后,随机生成100组起点和终点
    count = 0
    while count < test_num and not rospy.is_shutdown():
        print("---------------one turn generation-------------")
        # 生成随机起点和终点
        init_x = np.random.uniform(map_left_, map_right_)
        init_y = np.random.uniform(map_bottom_, map_top_)
        goal_x = np.random.uniform(map_left_, map_right_)
        goal_y = np.random.uniform(map_bottom_, map_top_)
        # 保证起点和终点之间的距离
        distance_gap = (goal_x - init_x) ** 2 + (goal_y - init_y) ** 2
        if distance_gap < 2:
            continue
        # 格式转化
        init_point = PoseWithCovarianceStamped()
        init_point.header.frame_id = "odom"
        init_point.header.stamp = rospy.Time.now()
        init_point.pose.pose.position.x = init_x
        init_point.pose.pose.position.y = init_y
        goal_point = PoseStamped()
        goal_point.header.frame_id = "odom"
        goal_point.header.stamp = rospy.Time.now()
        goal_point.pose.position.x = goal_x
        goal_point.pose.position.y = goal_y
        print("start point:", init_x, init_y)
        print("goal point:", goal_x, goal_y)
        # 发布起点
        lost_times = 0
        while not rospy.is_shutdown():
            init_point_pub_.publish(init_point)
            odom_msg = rospy.wait_for_message(odom_topic, Odometry)
            distance_gap = np.sqrt((odom_msg.pose.pose.position.x - init_x) ** 2 + (odom_msg.pose.pose.position.y - init_y) ** 2)
            if distance_gap < 0.1 or lost_times > 10:
                break
            rospy.Rate(10).sleep()
            lost_times += 1
        if lost_times > 10:
            continue
        # 发布终点
        goal_point_pub_.publish(goal_point)
        # 接收规划结果
        try:
            planning_result = rospy.wait_for_message(planning_result_topic, Int8, 50)
        except ROSException:
            continue
        # 判断规划结果,如果成功则保存
        if int(planning_result.data) == 1:
            save_file = open(root_path + str(count) + '.txt', 'w')
            save_file.write("init point:" + str(init_x) + "," + str(init_y) + "\n")
            save_file.write("goal point:" + str(goal_x) + "," + str(goal_y) + "\n")
            save_file.close()
            count += 1

if __name__ == "__main__":
    main()
