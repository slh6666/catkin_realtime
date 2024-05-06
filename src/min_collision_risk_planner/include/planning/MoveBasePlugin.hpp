#ifndef MCRP_PLANNER_H_
#define MCRP_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include "planning/LazyThetaStar.hpp"
#include "planning/KeyVerticesObtain.hpp"
#include "planning/PathOptimizer.hpp"
#include "utilities/CubicSpline.hpp"
#include "utilities/KDTree.hpp"
#include "common/Common.hpp"
#include "common/Config.hpp"
#include <mutex>

namespace mcrp_global_planner {

class MCRPlanner : public nav_core::BaseGlobalPlanner {
 public:
    // construct
    MCRPlanner() {};

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    // init
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // start planning
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

 private:
    // 地图回调函数
    void costmapCallback(nav_msgs::OccupancyGrid::ConstPtr map_msg);

    // 获取关键点
    int keyVerticesObtain(const PathPlanningUtilities::Path &init_path, PathPlanningUtilities::Path &key_vertices);

    // 进行路径优化
    int pathOptimizing(PathPlanningUtilities::Curve &optimized_curve, PathPlanningUtilities::Path &optimized_key_vertices, const PathPlanningUtilities::Point2f &current_point, const PathPlanningUtilities::Point2f &destination_point, const GridMap &costmap, const KDTree &kdtree);

    // 路径与障碍物的平均距离
    bool pathAvgDistanceToObs(const PathPlanningUtilities::Curve &curve, const KDTree &kdtree, double &distance_to_obs);

    // parameters
    ros::Subscriber costmap_sub_;
    ros::Publisher global_path_pub_;
    ros::Publisher global_path_vis_pub_;
    bool initialized_ = false;
    GridMap costmap_;
    KDTree kdtree_;
    bool map_obtained_flag_ = false;  // 地图获取成功标志位
    std::shared_ptr<PathPlanningUtilities::Curve> historical_curve_ptr_ = nullptr;
    std::shared_ptr<geometry_msgs::PoseStamped> historical_goal_ptr_ = nullptr;

    // mutex
    std::mutex costmap_mutex_;
    std::mutex kdtree_mutex_;
    std::mutex map_obtained_flag_mutex_;
};

};
#endif
