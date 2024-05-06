/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef MIN_COLLISION_RISK_PLANNER_NODE_HPP_
#define MIN_COLLISION_RISK_PLANNER_NODE_HPP_

#include <limits>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <navigation_msgs/Path.h>
#include <navigation_msgs/PathPoint.h>
#include "planning/Astar.hpp"
#include "planning/ThetaStar.hpp"
#include "planning/LazyThetaStar.hpp"
#include "planning/KeyVerticesObtain.hpp"
#include "planning/PathOptimizer.hpp"
#include "utilities/CubicSpline.hpp"
#include "utilities/KDTree.hpp"
#include "utilities/Visualization.hpp"
#include "common/Common.hpp"
#include "common/Config.hpp"

namespace mcrp_global_planner {

// 最小化碰撞风险规划节点
class MinCollisionRiskPlannerNode {
 public:
    // 构造函数
    MinCollisionRiskPlannerNode(const ros::NodeHandle &nh){
        // 获取ros句柄
        this->nh_ = nh;
    };

    // 析构函数
    ~MinCollisionRiskPlannerNode() {};

    // 开始进行规划
    void startPlanning();

 private:

    // 初始化与ROS链接
    void initROSConnection();

    // 监听ros线程
    void listenRosMSGThread();

    // 开始规划线程
    void planningThread();

    // 获取关键点
    void keyVerticesObtain(const PathPlanningUtilities::Path &init_path, PathPlanningUtilities::Path &key_vertices);

    // 进行路径优化
    void pathOptimizing(PathPlanningUtilities::Curve &optimized_curve, PathPlanningUtilities::Path &optimized_key_vertices, const PathPlanningUtilities::Point2f &current_point, const PathPlanningUtilities::Point2f &destination_point, const GridMap &costmap, const KDTree &kdtree);

    // 定位回调函数
    void odomCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);

    // 目的地回调函数
    void destinationCallback(geometry_msgs::PoseStamped::ConstPtr pose_msg);

    // 地图回调函数
    void costmapCallback(nav_msgs::OccupancyGrid::ConstPtr map_msg);

    // ros句柄
    ros::NodeHandle nh_;
    // ros消息订阅器
    ros::Subscriber odom_sub_;  // 订阅定位信息
    ros::Subscriber destination_sub_;  // 订阅目的地信息
    ros::Subscriber costmap_sub_;  // 订阅全局地图信息
    // ros消息发布器
    ros::Publisher init_path_pub_;  // 初始路径发布
    ros::Publisher init_path_vis_pub_;  // 初始路径可视化发布
    ros::Publisher key_vertices_pub_;  // 关键点发布
    ros::Publisher processed_path_pub_;  // 后处理路径发布
    ros::Publisher processed_path_vis_pub_;  // 后处理路径可视化发布
    ros::Publisher optimized_path_pub_;  // 优化路径发布
    ros::Publisher optimized_path_vis_pub_;  // 优化路径可视化发布
    ros::Publisher optimized_key_vertices_pub_;  // 优化路径可视化发布
    ros::Publisher planning_result_flag_pub_;  // 发布规划结果(-1为规划失败,0为到达终点,1为规划成功)
    ros::Publisher navigation_path_pub_;  // 全局导航路径发布
    
    // 属性
    PathPlanningUtilities::Point2f destination_point_;  // 目的地
    PathPlanningUtilities::Point2f current_point_;  // 当前定位
    GridMap costmap_;  // 全局地图
    KDTree kdtree_;  // 地图中被占据栅格对应的kd树

    // mutex
    std::mutex destination_point_mutex_;
    std::mutex current_point_mutex_;
    std::mutex costmap_mutex_;
    std::mutex kdtree_mutex_;
    std::mutex destination_obtained_flag_mutex_;
    std::mutex current_point_obtained_flag_mutex_;
    std::mutex map_obtained_flag_mutex_;
    std::mutex planning_start_flag_mutex_;
    
    // 标志位
    bool destination_obtained_flag_ = false;  // 目的地获取成功标志位
    bool current_point_obtained_flag_ = false;  // 当前位置获取成功标志位
    bool map_obtained_flag_ = false;  // 地图获取成功标志位
    bool planning_start_flag_ = false;  // 启动规划标志位
    bool save_logs_flag_ = false;  // 是否保存日志信息标志位
};

};

#endif // MIN_COLLISION_RISK_PLANNER_NODE_HPP_
