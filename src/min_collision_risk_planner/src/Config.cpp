#include "common/Config.hpp"

namespace mcrp_global_planner {

// --------------------------------基础常量-----------------------------

// 用于A星碰撞判断的机器人半径
double Config::robot_size_ = 0.26;

// 真实的机器人长度
double Config::robot_length_ = 0.455;

// 真实机器人宽度
double Config::robot_width_ = 0.381;

// 规划点在机器人的比例(0为车头,1为车尾)
double Config::robot_rear_axis_center_scale_ = 0.43;

// -------------------------------全局导航常量--------------------------

// 到达终点距离判断
double Config::destination_tolerance_ = 1.0;

// 栅格分辨率
double Config::grid_resolution_ = 0.05;

// 关键点提取偏移
double Config::key_vertices_obtain_tolerance_ = 0.1;

// 关键点增加最大迭代次数
int Config::key_vertices_increase_max_iteration_ = 100;

// 采样点间隔
double Config::sample_gap_ = 0.1;

// 最大优化迭代次数
int Config::max_optimization_iteration_ = 10;

// 优化终止条件
double Config::optimization_terminate_condition_ = 1e-2;

// optimize time terminate condition
double Config::optimization_time_terminate_condition_ = 0.5;

// 优化初始步长
double Config::optimization_init_step_ = 1e-1;

// 安全性损失权重
double Config::safety_cost_weight_ = 5.0;

// 安全损失考虑百分比
double Config::safety_cost_consider_rate_ = 0.2;

// 平滑性损失权重
double Config::smooth_cost_weight_ = 1.0;

// 平滑损失考虑百分比
double Config::smooth_cost_consider_rate_ = 0.2;

// start offset tolarence
double Config::start_offset_tolarence_ = 0.2;

// goal offset tolarence
double Config::goal_offset_tolarence_ = 0.2;

// 偏移损失
double Config::offset_cost_weight_ = 0.1;

// 关键点的最小距离
// double Config::key_vertices_min_distance_ = 0.2;
double Config::key_vertices_min_distance_ = 0.1;

};
