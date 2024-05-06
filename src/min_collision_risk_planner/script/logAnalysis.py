#! /usr/bin/python
#! -*- coding: utf-8 -*-

"""

对日志进行分析
author: flztiii

"""

import rospkg
import os
import numpy as np
import matplotlib.pyplot as plt


# 主函数
def main():
    # 得到路径
    root_path = rospkg.RosPack().get_path("min_collision_risk_planner")
    root_path += "/logs/optimized_path_info/"
    print(root_path)
    if not os.path.exists(root_path):
        print("aimed path does not exist")
        return
    # 进行文件解析
    curve_lengths, avg_curvatures, maximum_curvatures, avg_dis_to_obss, min_dis_to_obss, collsions, time_consumptions = list(), list(), list(), list(), list(), list(), list()
    count = 0
    for file_name in os.listdir(root_path):
        file_path = root_path + file_name
        file_data = open(file_path, 'r').readlines()
        # 读取路径长度
        curve_lengths.append(float((file_data[2].strip().split(":"))[1]))
        avg_curvatures.append(float((file_data[3].strip().split(":"))[1]))
        maximum_curvatures.append(abs(float((file_data[4].strip().split(":"))[1])))
        avg_dis_to_obss.append(float((file_data[5].strip().split(":"))[1]))
        min_dis_to_obss.append(float((file_data[6].strip().split(":"))[1]))
        collsions.append(int((file_data[7].strip().split(":"))[1]))
        time_consumptions.append(float((file_data[8].strip().split(":"))[1]))
        count += 1
    # 完成解析,进行统计
    print("valid test count: ", count)
    # 首先是路径长度平均值
    avg_curve_length = np.mean(curve_lengths)
    print("averge curve length: ", avg_curve_length)
    # 平均曲率
    avg_curvature = np.mean(avg_curvatures)
    print("averge curvature: ", avg_curvature)
    # 平均最大曲率
    avg_max_curvature = np.mean(maximum_curvatures)
    print("averge max curvature: ", avg_max_curvature)
    # 平均离障碍物距离
    avg_dis_to_obs = np.mean(avg_dis_to_obss)
    print("averge distance to obstacles: ", avg_dis_to_obs)
    # 平均离障碍物的最小距离
    avg_min_dis_to_obs = np.mean(min_dis_to_obss)
    print("averge min distance to obstacles: ", avg_min_dis_to_obs)
    # 发生碰撞的总次数
    sum_collisions = np.sum(collsions)
    print("total collsion times: ", sum_collisions)
    # 平均时间开销
    avg_time_consumption = np.mean(time_consumptions)
    print("average time consumption: ", avg_time_consumption)
    # 最大时间开销
    max_time_consumption = np.max(time_consumptions)
    print("max time consumption: ", max_time_consumption)

if __name__ == "__main__":
    main()