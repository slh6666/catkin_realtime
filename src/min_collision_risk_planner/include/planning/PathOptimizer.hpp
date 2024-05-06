/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef PATH_OPTIMIZER_HPP_
#define PATH_OPTIMIZER_HPP_

#include <vector>
#include <algorithm>
#include <numeric>
#include <nlopt.hpp>
#include <functional>
#include "utilities/KDTree.hpp"
#include "utilities/CubicSpline.hpp"
#include "common/Config.hpp"

namespace mcrp_global_planner {

// 定义目标函数类型
using ObjectiveFuncType = boost::function<double(const std::vector<double> &, std::vector<double> &)>;

// 路径优化器
class PathOptimizer {
 public:
    // 构造函数
    PathOptimizer(const std::vector<double> &init_param, const KDTree &kdtree, const PathPlanningUtilities::Point2f &current_point, const PathPlanningUtilities::Point2f &destination_point) {
        // 设置参数
        this->param_ = init_param;
        this->kdtree_ = kdtree;
        this->current_point_ = current_point;
        this->destination_point_ = destination_point;
    };

    // 析构函数
    ~PathOptimizer() {};

    // 进行参数优化
    nlopt::result optimize() {
        if (this->param_.size() == 0) {
            return nlopt::result::SUCCESS;
        }
        // 优化结果
        nlopt::result result = nlopt::result::FAILURE;
        // 构造优化器对象(给出优化算法和优化参数维度)
        nlopt::opt optimizer(nlopt::LN_COBYLA, this->param_.size());
        // 设置优化目标函数
        ObjectiveFuncType objective_function = boost::bind(&PathOptimizer::objectiveFunction, this, _1, _2);
        optimizer.set_min_objective(PathOptimizer::objectiveFunctionWrapper, &objective_function);
        // 设置优化终止条件
        optimizer.set_xtol_rel(Config::optimization_terminate_condition_);
        // 设置优化最大时间开销
        double max_time_consumption = (static_cast<double>(this->param_.size()) * Config::optimization_time_terminate_condition_) * (static_cast<double>(this->param_.size()) * Config::optimization_time_terminate_condition_) * 0.001;
        optimizer.set_maxtime(max_time_consumption);
        // 设置初始步长
        optimizer.set_initial_step(Config::optimization_init_step_);
        // 进行优化
        double min_value;
        try{
            result = optimizer.optimize(this->param_, min_value);
        }
        catch(std::exception &e) {
            std::cout << "optimize failed: " << e.what() << std::endl;
        }
        return result;
    };

    // 得到当前关键点
    PathPlanningUtilities::Path getKeyVertices() const {
        PathPlanningUtilities::Path control_vertices;
        for (size_t i = 0; i < this->param_.size(); i+= 2) {
            PathPlanningUtilities::Point2f control_vertex = {this->param_[i], this->param_[i+1]};
            control_vertices.push_back(control_vertex);
        }
        return control_vertices;
    };

 private:
    // 包装目标函数
    static double objectiveFunctionWrapper(const std::vector<double> &x, std::vector<double> &grad, void *data) {
        return (*((ObjectiveFuncType*)(data)))(x, grad);
    };

    // 优化目标函数
    double objectiveFunction(const std::vector<double> &x, std::vector<double> &grad) {
        // 根据参数构造曲线
        CubicSpline cubic_spline = this->unwarpParam(x);
        // 对曲线进行采样
        // 得到采样
        int sample_num = static_cast<int>(cubic_spline.getMaxArcLength() / Config::sample_gap_);
        std::vector<double >samples = Tools::linspace(0.0, cubic_spline.getMaxArcLength(), sample_num);
        PathPlanningUtilities::Path current_path;
        for (auto sample: samples) {
            current_path.push_back(cubic_spline.getPoint(sample));
        }
        // 计算安全性损失
        double safety_cost = 0.0;
        if (!this->kdtree_.isEmpty()) {
            // 计算每个点到障碍物的距离
            std::vector<double> min_distances_to_obstacle;
            for (auto path_point: current_path) {
                std::vector<std::pair<float, float>> neighbors;
                std::vector<float> sq_distances;
                int result = this->kdtree_.findKNeighbor(path_point.x_, path_point.y_, &neighbors, &sq_distances, 1);
                if (result > -1) {  
                    min_distances_to_obstacle.push_back(1.0 / sq_distances[0]);
                } else {
                    min_distances_to_obstacle.push_back(std::numeric_limits<double>::max());
                }
            }
            sort(min_distances_to_obstacle.begin(), min_distances_to_obstacle.end());
            reverse(min_distances_to_obstacle.begin(), min_distances_to_obstacle.end());
            int safety_consider_nums = std::max(static_cast<int>(min_distances_to_obstacle.size() * Config::safety_cost_consider_rate_), 1);
            safety_cost = std::accumulate(min_distances_to_obstacle.begin(), min_distances_to_obstacle.begin() + safety_consider_nums, safety_cost) / static_cast<double>(safety_consider_nums);
        }

        // 计算平滑性损失
        double smooth_cost = 0.0;
        // 计算每个点的平滑性
        std::vector<double> smooth_evaluations;
        for (size_t i = 1; i < current_path.size() - 1; i++) {
            smooth_evaluations.push_back((current_path[i + 1].x_ + current_path[i - 1].x_ - 2 * current_path[i].x_) * (current_path[i + 1].x_ + current_path[i - 1].x_ - 2 * current_path[i].x_) + (current_path[i + 1].y_ + current_path[i - 1].y_ - 2 * current_path[i].y_) * (current_path[i + 1].y_ + current_path[i - 1].y_ - 2 * current_path[i].y_));
        }
        sort(smooth_evaluations.begin(), smooth_evaluations.end());
        reverse(smooth_evaluations.begin(), smooth_evaluations.end());
        int smooth_consider_nums = std::max(static_cast<int>(smooth_evaluations.size() * Config::smooth_cost_consider_rate_), 1);
        smooth_cost = std::accumulate(smooth_evaluations.begin(), smooth_evaluations.begin() + smooth_consider_nums, smooth_cost) / static_cast<double>(smooth_consider_nums);

        // calculate offset cost
        double offset_cost = 0.0;
        // start offset
        double start_offset = PathPlanningUtilities::calcDistance(current_path.front(), this->current_point_);
        offset_cost += tan(0.5 * M_PI * std::min(start_offset, Config::start_offset_tolarence_) / Config::start_offset_tolarence_);
        // goal offset
        double goal_offset = PathPlanningUtilities::calcDistance(current_path.back(), this->destination_point_);
        offset_cost += tan(0.5 * M_PI * std::min(goal_offset, Config::goal_offset_tolarence_) / Config::goal_offset_tolarence_);

        // 得到总损失
        double total_cost = Config::safety_cost_weight_ * safety_cost + Config::smooth_cost_weight_ * smooth_cost + Config::offset_cost_weight_ * offset_cost;
        // 输出损失信息
        // std::cout << "safety cost is " << safety_cost << ", smooth cost is " << smooth_cost << ", total cost is " << total_cost << std::endl;
        return total_cost;
    };

    // 根据参数得到曲线
    CubicSpline unwarpParam(const std::vector<double> &param) const {
        PathPlanningUtilities::Path control_vertices;
        for (size_t i = 0; i < param.size(); i+= 2) {
            PathPlanningUtilities::Point2f control_vertex = {param[i], param[i+1]};
            control_vertices.push_back(control_vertex);
        }
        return CubicSpline(control_vertices);
    };

    // 属性
    std::vector<double> param_;  // 优化参数
    KDTree kdtree_;  // 优化计算过程所需
    PathPlanningUtilities::Point2f current_point_;  // 优化计算过程所需
    PathPlanningUtilities::Point2f destination_point_;  // 优化计算过程所需
};

};

# endif  // PATH_OPTIMIZER_HPP_
