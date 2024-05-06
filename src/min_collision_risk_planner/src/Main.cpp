#include "planning/MinCollisionRiskPlannerNode.hpp"

// 主函数
int main(int argc, char **argv) {
    // 初始化ros
    ros::init(argc, argv, "min_collision_risk_planner_node");
    ros::NodeHandle nh = ros::NodeHandle("~");
    // 规划程序运行
    std::unique_ptr<mcrp_global_planner::MinCollisionRiskPlannerNode> min_collision_risk_planner_node_ptr(new mcrp_global_planner::MinCollisionRiskPlannerNode(nh));
    min_collision_risk_planner_node_ptr->startPlanning();
    return 0;
}
