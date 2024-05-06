#! /usr/bin/python
#! -*- coding: utf-8 -*-

"""

读取固定起点和终点进行规划
author: flztiii

"""

import rospkg
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import numpy as np
import os
from rospy.exceptions import ROSException

def main():
    # 初始化ros
    rospy.init_node("fixed_batch_init_and_goal_test_node")
    # 订阅定位结果
    odom_topic = rospy.get_param("~odom_topic")
    # 订阅规划结果消息
    planning_result_topic = rospy.get_param("~planning_result_topic")
    # 发布起点和终点消息
    init_point_pub_ = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    goal_point_pub_ = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
    # 得到测试场景名称
    scene_name = rospy.get_param("~scene_name")
    # 得到文件保存目录
    root_path = rospkg.RosPack().get_path("min_collision_risk_planner")
    root_path += "/logs/" + scene_name + "_test_pairs/"
    # 判断文件夹是否存在
    if not os.path.exists(root_path):
       print("file path not exist")
    # 遍历文件夹中的文件
    for file_name in os.listdir(root_path):
        print("---------------one turn generation-------------")
        file_path = root_path + file_name
        file_data = open(file_path, 'r').readlines()
        # 得到起点和终点
        init_x = float((file_data[0].strip().split(':')[1]).split(',')[0])
        init_y = float((file_data[0].strip().split(':')[1]).split(',')[1])
        init_yaw = np.random.uniform(-np.pi, np.pi)
        goal_x = float((file_data[1].strip().split(':')[1]).split(',')[0])
        goal_y = float((file_data[1].strip().split(':')[1]).split(',')[1])
        # 格式转化
        init_point = PoseWithCovarianceStamped()
        init_point.header.frame_id = "odom"
        init_point.header.stamp = rospy.Time.now()
        init_point.pose.pose.position.x = init_x
        init_point.pose.pose.position.y = init_y
        init_point.pose.pose.orientation.z = np.sin(0.5 * init_yaw)
        init_point.pose.pose.orientation.w = np.cos(0.5 * init_yaw)
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
        if lost_times > 100:
            continue
        # 发布终点
        goal_point_pub_.publish(goal_point)
        print("start planning")
        # 接收规划结果
        try:
            rospy.wait_for_message(planning_result_topic, Int8)
        except ROSException:
            continue

if __name__ == "__main__":
    # 等待按键输入
    s = input("input anything to start program: ")
    main()
