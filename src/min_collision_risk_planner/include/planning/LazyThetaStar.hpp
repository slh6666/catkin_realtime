/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef LAZY_THETA_STAR_H_
#define LAZY_THETA_STAR_H_

#include "common/Config.hpp"
#include "common/Common.hpp"
#include "utilities/Tools.hpp"
#include "common/Point.hpp"
#include "common/Path.hpp"
#include "utilities/PathGenerator.h"
#include "utilities/LineSegment.hpp"
#include "utilities/Visualization.hpp"
#include "utilities/KDTree.hpp"
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <typeinfo>
#include <thread>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <queue>
#include <set>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <ctime>

namespace mcrp_global_planner {

// 初始路径搜索
namespace InitPathSearch {

// Lazy Theta星规划器
class LazyThetaStarPlanner {
 public:
    // 构造函数
    LazyThetaStarPlanner(double robot_size){
        this->robot_size_ = robot_size;
        this->motions_.push_back(std::make_pair(-1, -1));
        this->motions_.push_back(std::make_pair(-1, 0));
        this->motions_.push_back(std::make_pair(-1, 1));
        this->motions_.push_back(std::make_pair(0, -1));
        this->motions_.push_back(std::make_pair(0, 1));
        this->motions_.push_back(std::make_pair(1, -1));
        this->motions_.push_back(std::make_pair(1, 0));
        this->motions_.push_back(std::make_pair(1, 1));
    };

    // 析构函数
    ~LazyThetaStarPlanner(){};

    // 进行规划
    int planning(const PathPlanningUtilities::Point2f &start_point, const PathPlanningUtilities::Point2f &goal_point, const GridMap &grid_map, const KDTree &kd_tree, PathPlanningUtilities::Path &path){
        // 进行搜索
        // 初始化节点记录器
        std::vector<Node> node_recorder;
        node_recorder.resize(grid_map.getWidth() * grid_map.getHeight());
        // 初始化搜索记录器
        std::priority_queue<Node, std::vector<Node>, std::greater<Node> > open_set;  // 未搜索节点
        // 构建起始节点和目标节点
        std::pair<int, int> start_in_grid = grid_map.getGridMapCoordinate(start_point.x_, start_point.y_);
        std::pair<int, int> goal_in_grid = grid_map.getGridMapCoordinate(goal_point.x_, goal_point.y_);
        // 验证起点和终点的有效性
        if(!grid_map.isVerify(start_in_grid.first, start_in_grid.second) || !grid_map.isVerify(goal_in_grid.first, goal_in_grid.second)) {
            // 起点或终点无效
            return -1;
        }
        // 验证起点和终点是否发生碰撞
        if(this->nodeCollisionJudgement(start_in_grid.first, start_in_grid.second, grid_map) || this->nodeCollisionJudgement(goal_in_grid.first, goal_in_grid.second, grid_map)) {
            // 起点或终点发生碰撞
            return -1;
        }
        Node start_node = Node(start_in_grid.first, start_in_grid.second);
        Node goal_node = Node(goal_in_grid.first, goal_in_grid.second);
        // 计算起点的代价和启发
        start_node.cost_ = 0.0;
        start_node.heuristic_ = this->calcHeuristic(start_node, goal_node);
        // 将起始节点加入搜索列表
        start_node.open();
        node_recorder[grid_map.getIndex(start_node.x_, start_node.y_)] = start_node;
        open_set.push(start_node);
        // 开始搜索
        while (!open_set.empty()) {
            // 得到搜索列表中最小的index作为当前节点,并从开集合中删除
            Node current_node = open_set.top();
            open_set.pop();
            // 计算当前节点下标
            int current_index = grid_map.getIndex(current_node.x_, current_node.y_);
            // 判断是否存在父节点
            if (current_node.pre_index_ != -1) {
                // 存在父节点,进行节点重置
                // 得到当前节点的父节点
                Node parent_node = node_recorder[current_node.pre_index_];
                // 判断当前节点与父节点之间的连线是否无碰撞
                // 得到当前节点和父节点的连线
                PathPlanningUtilities::LineSegment line_segment = PathPlanningUtilities::LineSegment(PathPlanningUtilities::Point2f{static_cast<double>(parent_node.x_), static_cast<double>(parent_node.y_)}, PathPlanningUtilities::Point2f{static_cast<double>(current_node.x_), static_cast<double>(current_node.y_)});
                // 判断连线是否发生碰撞
                bool is_line_segment_collision = false;
                for (double length = 1.0; length <= line_segment.length(); length += 1.0) {
                    PathPlanningUtilities::Point2f tmp_point = PathPlanningUtilities::Point2f{static_cast<double>(parent_node.x_), static_cast<double>(parent_node.y_)} + line_segment.getVector() * (length / line_segment.length());
                    // 判断是否发生碰撞
                    if (this->nodeCollisionJudgement(static_cast<int>(tmp_point.x_), static_cast<int>(tmp_point.y_), grid_map)) {
                        // 发生碰撞
                        is_line_segment_collision = true;
                        break;
                    }
                }
                // 如果发生碰撞
                if (is_line_segment_collision) {
                    // 进行节点重置
                    current_node.cost_ = std::numeric_limits<double>::max();
                    current_node.pre_index_ = -2;
                    // 遍历当前节点的邻居节点
                    // 如果当前节点不是终点,搜索其的邻居
                    for (auto motion: this->motions_) {
                        // 计算邻居节点
                        Node neighbor_node = Node(current_node.x_ + motion.first, current_node.y_ + motion.second, current_index);
                        // 得到邻居节点下标
                        int neighbor_index = grid_map.getIndex(neighbor_node.x_, neighbor_node.y_);
                        // 判断邻居是否超出边界
                        if(!grid_map.isVerify(neighbor_node.x_, neighbor_node.y_)) {
                            continue;
                        }
                        // 判断邻居节点是否在闭集合内
                        if (node_recorder[neighbor_index].is_closed_ == false) {
                            // 存在于闭集合内
                            continue;
                        }
                        // 计算新损失
                        double new_cost = node_recorder[neighbor_index].cost_ + sqrt(motion.first * motion.first + motion.second * motion.second);
                        // 判断是否更新当前节点
                        if (Tools::isSmall(new_cost, current_node.cost_)) {
                            current_node.cost_ = new_cost;
                            current_node.pre_index_ = neighbor_index;
                        }
                    }
                    // 判断更新结束
                    assert(current_node.pre_index_ > -2);
                    // 更新列表
                    node_recorder[current_index] = current_node;
                }
            }
            // 将当前节点加入闭集合,
            node_recorder[current_index].close();
            // 判断当前节点是否为终点
            if (current_node.x_ == goal_node.x_ && current_node.y_ == goal_node.y_) {
                // 当前节点为终点,结束搜索
                goal_node = current_node;
                break;
            }
            // 如果当前节点不是终点,搜索其的邻居
            for (auto motion: this->motions_) {
                // 计算邻居节点
                Node neighbor_node = Node(current_node.x_ + motion.first, current_node.y_ + motion.second, current_index);
                // 得到邻居节点下标
                int neighbor_index = grid_map.getIndex(neighbor_node.x_, neighbor_node.y_);
                // 判断邻居节点是否在闭集合内
                if (node_recorder[neighbor_index].is_closed_ == true) {
                    // 存在于闭集合内
                    continue;
                }
                // 判断邻居是否超出边界
                if(!grid_map.isVerify(neighbor_node.x_, neighbor_node.y_)) {
                    continue;
                }
                // 判断此邻居是否与障碍物碰撞
                bool is_neighbor_collide = this->nodeCollisionJudgement(neighbor_node.x_, neighbor_node.y_, grid_map);
                if (is_neighbor_collide) {
                    continue;
                }
                // 判断当前节点是否存在父节点
                if (current_node.pre_index_ != -1) {
                    // 存在父节点
                    Node parent_node = node_recorder[current_node.pre_index_];
                    // 将邻居节点的父节点从当前节点改为当前节点的父节点
                    neighbor_node.pre_index_ = current_node.pre_index_;
                    // 计算损失
                    double motion_cost = sqrt((neighbor_node.x_ - parent_node.x_) * (neighbor_node.x_ - parent_node.x_) + (neighbor_node.y_ - parent_node.y_) * (neighbor_node.y_ - parent_node.y_));
                    neighbor_node.cost_ = motion_cost + parent_node.cost_;
                    // 计算启发
                    neighbor_node.heuristic_ = this->calcHeuristic(neighbor_node, goal_node);
                } else {
                    // 不存在父节点
                    // 计算损失增量,包括两个部分,走过的距离和离障碍物的距离
                    double motion_cost = sqrt(motion.first * motion.first + motion.second * motion.second);
                    // 得到邻居的损失
                    neighbor_node.cost_ = current_node.cost_ + motion_cost;
                    // 得到邻居的启发
                    neighbor_node.heuristic_ = this->calcHeuristic(neighbor_node, goal_node);
                }
                // 判断邻居节点是否在开集合内
                if (node_recorder[neighbor_index].is_open_ == true) {
                    // 在开集合内
                    if (neighbor_node < node_recorder[neighbor_index]) {
                        neighbor_node.open();
                        node_recorder[neighbor_index] = neighbor_node;
                        open_set.push(neighbor_node);
                    }
                } else {
                    // 不在开集合内
                    // 加入开集合
                    neighbor_node.open();
                    node_recorder[neighbor_index] = neighbor_node;
                    open_set.push(neighbor_node);
                }
            }
        }
        // 开始生成最终路径
        // 判断是否生成了最终路径
        if (goal_node.pre_index_ == -1) {
            // 没有找到路径
            return -1;
        }
        // 生成了最终路径
        PathPlanningUtilities::Path raw_path;
        Node current_node = goal_node;
        while (true) {
            PathPlanningUtilities::Point2f point = grid_map.getCartesianCoordinate(current_node.x_, current_node.y_);
            raw_path.push_back(point);
            if (current_node.pre_index_ != -1) {
                current_node = node_recorder[current_node.pre_index_];
                assert(current_node.is_closed_);
            } else {
                break;
            }
        }
        // 反转路径后输出
        reverse(raw_path.begin(),raw_path.end());
        // 给raw_path补点
        bool interpolation_finished = false;
        while (!interpolation_finished) {
            interpolation_finished = true;
            for (size_t i = 1; i < raw_path.size(); i++) {
                // 计算两点之间的距离
                double distance = PathPlanningUtilities::calcDistance(raw_path[i - 1], raw_path[i]);
                if (Tools::isLarge(distance, 2.0 * Config::grid_resolution_)) {
                    // 在两点之间插入一个点
                    PathPlanningUtilities::Point2f new_point;
                    new_point.x_ = (raw_path[i - 1].x_ + raw_path[i].x_) * 0.5;
                    new_point.y_ = (raw_path[i - 1].y_ + raw_path[i].y_) * 0.5;
                    raw_path.insert(raw_path.begin() + i, new_point);
                    interpolation_finished = false;
                    break;
                }
            }
        }
        path = raw_path;
        return 0;
    };

 private:
    // 计算启发函数(使用的启发函数为到终点的距离)
    double calcHeuristic(const Node &current_node, const Node &goal_node, double weight= 1.0) const {
        return sqrt((current_node.x_ - goal_node.x_) * (current_node.x_ - goal_node.x_) + (current_node.y_ - goal_node.y_) * (current_node.y_ - goal_node.y_)) * weight;
    }

    // 判断节点是否发生碰撞
    bool nodeCollisionJudgement(int node_x, int node_y, const GridMap &grid_map) const {
        int range = static_cast<int>(this->robot_size_ / Config::grid_resolution_);
        for (int i = - range; i < range + 1; i++) {
            for (int j = -range; j < range + 1; j++) {
                if (!grid_map.isVerify(node_x + i, node_y + j)) {
                    return true;
                } else if (grid_map.isOccupied(grid_map.getIndex(node_x + i, node_y + j))) {
                    return true;
                }
            }
        }
        return false;
    }

    double robot_size_;  // 机器人大小
    std::vector<std::pair<int, int>> motions_;  // 行为
};

};

};

#endif
