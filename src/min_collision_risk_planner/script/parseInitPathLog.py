#! /usr/bin/python
#! -*- coding:utf-8 -*-

"""

解析初始路径日志文件
author: flztiii

"""

import os
import rospkg
import rospy
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # 得到日志文件根目录
    root_path = rospkg.RosPack().get_path("min_collision_risk_planner")
    root_path += "/logs/init_path_info/"
    file_list = os.listdir(root_path)
    for i in range(0, len(file_list)):
        # 得到日志文件路径
        file_path = root_path + file_list[i]
        print(file_path)
        # 读取文件内容
        log_data = list()
        log_file = open(file_path, 'r')
        for line in log_file.readlines():
            log_data.append(float(line.strip()))
        # 进行内容的可视化
        plt.cla()
        plt.plot(range(0, len(log_data)), log_data)
        plt.show()