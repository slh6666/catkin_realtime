/*
    Copyright [2021] Jian ZhiQiang
*/
#pragma once
#ifndef CONFIG_H_
#define CONFIG_H_

// 定义全局变量
// define global parameters

namespace mcrp_global_planner {

class Config {
 public:
    // 构造函数
    Config (){}

    // 析构函数
    ~Config () {}

    // --------------------------------基础常量-----------------------------

    // 用于A星碰撞判断的机器人半径
    static double robot_size_;

    // 真实的机器人长度
    static double robot_length_;

    // 真实机器人宽度
    static double robot_width_;

    // 规划点在机器人的比例(0为车头,1为车尾)
    static double robot_rear_axis_center_scale_;

    // -------------------------------全局导航常量--------------------------

    // 到达终点判断
    static double destination_tolerance_;

    // 栅格分辨率
    static double grid_resolution_;

    // 关键点提取偏移
    static double key_vertices_obtain_tolerance_;

    // 关键点增加最大迭代次数
    static int key_vertices_increase_max_iteration_;

    // 采样点间隔
    static double sample_gap_;

    // 最大优化迭代次数
    static int max_optimization_iteration_;
    
    // 优化终止条件
    static double optimization_terminate_condition_;

    // optimize time terminate condition
    static double optimization_time_terminate_condition_;

    // 优化初始步长
    static double optimization_init_step_;

    // 安全性损失权重
    static double safety_cost_weight_;

    // 安全损失考虑百分比
    static double safety_cost_consider_rate_;

    // 平滑性损失权重
    static double smooth_cost_weight_;

    // 平滑损失考虑百分比
    static double smooth_cost_consider_rate_;

    // start offset tolarence
    static double start_offset_tolarence_;

    // goal offset tolarence
    static double goal_offset_tolarence_;

    // 偏移损失
    static double offset_cost_weight_;

    // 关键点的最小距离
    static double key_vertices_min_distance_;
};

};

#endif