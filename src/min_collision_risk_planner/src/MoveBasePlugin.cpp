#include "planning/MoveBasePlugin.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mcrp_global_planner::MCRPlanner, nav_core::BaseGlobalPlanner);

namespace mcrp_global_planner {

// init
void MCRPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!this->initialized_) {
        ros::NodeHandle pnh("~/" + name);
        // subscribe
        this->costmap_sub_ = pnh.subscribe("/costmap_generation/costmap_2d", 1, &mcrp_global_planner::MCRPlanner::costmapCallback, this);
        // publisher 
        this->global_path_pub_ = pnh.advertise<nav_msgs::Path>("/mcrp/global_path", 10);
        this->global_path_vis_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("/mcrp/global_path_vis", 10);
        //reset flag
        this->initialized_ = true;
    } else {
        ROS_WARN("This planner has already been initialized... doing nothing");
    }
};

// start planning
bool MCRPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    // judge if costmap prepared
    this->map_obtained_flag_mutex_.lock();
    bool data_ready = this->map_obtained_flag_;
    this->map_obtained_flag_mutex_.unlock();
    if (!data_ready) {
        return false;
    }
    // data prepared, start planning
    // 获取定位、目的地和感知结果
    PathPlanningUtilities::Point2f current_point{start.pose.position.x, start.pose.position.y};
    PathPlanningUtilities::Point2f destination_point{goal.pose.position.x, goal.pose.position.y};
    this->costmap_mutex_.lock();
    GridMap costmap = this->costmap_;
    this->costmap_mutex_.unlock();
    this->kdtree_mutex_.lock();
    KDTree kdtree = this->kdtree_;
    this->kdtree_mutex_.unlock();

    // 计算当前点与目的地之间的距离
    double destination_distance = PathPlanningUtilities::calcDistance(current_point, destination_point);
    if (Tools::isSmall(destination_distance, Config::destination_tolerance_)) {
        ROS_WARN_STREAM("destination to near");
        this->historical_curve_ptr_ = nullptr;
        this->historical_goal_ptr_ = nullptr;
        return false;
    }

    // calculate time consuming
    clock_t start_planning_time = clock();

    // 初始路径生成
    std::shared_ptr<InitPathSearch::LazyThetaStarPlanner> lazy_theta_star_planner_ptr = std::make_shared<InitPathSearch::LazyThetaStarPlanner>(Config::robot_size_);
    PathPlanningUtilities::Path init_path;
    int init_path_search_result = lazy_theta_star_planner_ptr->planning(current_point, destination_point, costmap, kdtree, init_path);
    // 判断是否生成成功
    if (init_path_search_result == -1) {
        ROS_WARN_STREAM("init path searching failed");
        return false;
    }

    // 关键点提取
    PathPlanningUtilities::Path key_vertices;
    int key_vertices_obtain_result = this->keyVerticesObtain(init_path, key_vertices);
    if (key_vertices_obtain_result < 0) {
        return false;
    }

    // 进行曲线优化
    PathPlanningUtilities::Curve optimized_curve;
    PathPlanningUtilities::Path optimized_key_vertices = key_vertices;
    int optimize_result = this->pathOptimizing(optimized_curve, optimized_key_vertices, current_point, destination_point, costmap, kdtree);
    if (optimize_result < 0) {
        return false;
    }

    // // compared current optimized_curve with historical path
    // if (this->historical_curve_ptr_ != nullptr and this->historical_goal_ptr_ != nullptr) {
    //     // judge if goal changed
    //     double goal_change_gap = sqrt((goal.pose.position.x - this->historical_goal_ptr_->pose.position.x) * (goal.pose.position.x - this->historical_goal_ptr_->pose.position.x) + (goal.pose.position.y - this->historical_goal_ptr_->pose.position.y) * (goal.pose.position.y - this->historical_goal_ptr_->pose.position.y));
    //     if (Tools::isZero(goal_change_gap)) {
    //         // calculate avg distance to historical path
    //         double historical_curve_avg_distance_to_obs = 0.0;
    //         bool is_historical_path_collide = this->pathAvgDistanceToObs(*(this->historical_curve_ptr_), kdtree, historical_curve_avg_distance_to_obs);
    //         if (is_historical_path_collide) {
    //             this->historical_curve_ptr_ = std::make_shared<PathPlanningUtilities::Curve>(optimized_curve);
    //         } else {
    //             double otimized_curve_avg_distance_to_obs = 0.0;
    //             this->pathAvgDistanceToObs(optimized_curve, kdtree, otimized_curve_avg_distance_to_obs);
    //             if (Tools::isSmall(historical_curve_avg_distance_to_obs, otimized_curve_avg_distance_to_obs)) {
    //                 this->historical_curve_ptr_ = std::make_shared<PathPlanningUtilities::Curve>(optimized_curve);
    //             } else {
    //                 return false;
    //             }
    //         }
    //     } else {
    //         this->historical_curve_ptr_ = std::make_shared<PathPlanningUtilities::Curve>(optimized_curve);
    //         this->historical_goal_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(goal);
    //     }
    // } else {
    //    this->historical_curve_ptr_ = std::make_shared<PathPlanningUtilities::Curve>(optimized_curve);
    //     this->historical_goal_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(goal);
    // }

    // 进行格式转换和输出
    std::vector<geometry_msgs::PoseStamped> path_output;
    for (size_t i = 0; i < optimized_curve.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header = goal.header;
        pose.pose.position.x = optimized_curve[i].position_.x_;
        pose.pose.position.y = optimized_curve[i].position_.y_;
        pose.pose.orientation.z = std::sin(0.5 * optimized_curve[i].theta_);
        pose.pose.orientation.w = std::cos(0.5 * optimized_curve[i].theta_);
        path_output.push_back(pose);
    }
    plan = path_output;
    // publish viuslization
    // 进行格式转换和输出
    nav_msgs::Path path_output_vis;
    path_output_vis.header = goal.header;
    for (size_t i = 0; i < optimized_curve.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_output_vis.header;
        pose.pose.position.x = optimized_curve[i].position_.x_;
        pose.pose.position.y = optimized_curve[i].position_.y_;
        path_output_vis.poses.push_back(pose);
    }
    this->global_path_pub_.publish(path_output_vis);
    // 进行可视化
    // 清空之前的可视化
    visualization_msgs::MarkerArray delete_marker_array;
    delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
    this->global_path_vis_pub_.publish(delete_marker_array);
    // 可视化
    this->global_path_vis_pub_.publish(VisualizationMethods::visualizeCurveToMarker(path_output_vis, Config::robot_size_, VisualizationMethods::color(1, 0, 0, 0.1)));
    // calculate time consuming
    clock_t end_planning_time = clock();
    // ROS_INFO_STREAM("path searching finished, time consuming is " << static_cast<double>(end_planning_time - start_planning_time) * 1000.0 / CLOCKS_PER_SEC << " ms");
    return true;
};

// 获取关键点
int MCRPlanner::keyVerticesObtain(const PathPlanningUtilities::Path &init_path, PathPlanningUtilities::Path &key_vertices) {
    std::shared_ptr<KeyVerticesObtain::DouglasPeuckerSimplify> key_vertices_obtainer_ptr = std::make_shared<KeyVerticesObtain::DouglasPeuckerSimplify>();
    // 进行提取
    key_vertices = key_vertices_obtainer_ptr->simplifiedPath(init_path, Config::key_vertices_obtain_tolerance_);
    if (key_vertices.size() < 2) {
        return -1;
    }

    // 进行关键点扩增
    if (key_vertices.size() > 2) {
        int iteration_count = 0;
        while (true) {
            PathPlanningUtilities::Path updated_key_vetices;
            double processed_path_to_raw_path_max_gap = KeyVerticesObtain::keyVerticesIncrement(key_vertices, updated_key_vetices);
            iteration_count++;
            // 更新关键点
            key_vertices = updated_key_vetices;
            // 判断gap是否小于阈值
            if (Tools::isSmall(processed_path_to_raw_path_max_gap, Config::key_vertices_obtain_tolerance_)) {
                // 小于阈值,完成迭代
                // ROS_INFO_STREAM("key vertices increase finished: success");
                break;
            }
            // 判断迭代次数是否超越上限
            if (iteration_count > Config::key_vertices_increase_max_iteration_) {
                // ROS_INFO_STREAM("key vertices increase finished: out of max iteration");
                break;
            }
        }
    } else {
        PathPlanningUtilities::Point2f new_key_vertex;
        new_key_vertex.x_ = (key_vertices[0].x_ + key_vertices[1].x_) * 0.5;
        new_key_vertex.y_ = (key_vertices[0].y_ + key_vertices[1].y_) * 0.5;
        key_vertices.insert(key_vertices.begin() + 1, new_key_vertex);
    }
    return 0;
}

// 进行路径优化
int MCRPlanner::pathOptimizing(PathPlanningUtilities::Curve &optimized_curve, PathPlanningUtilities::Path &optimized_key_vertices, const PathPlanningUtilities::Point2f &current_point, const PathPlanningUtilities::Point2f &destination_point, const GridMap &costmap, const KDTree &kdtree) {
    int optimize_iteration = 0;  // 优化迭代次数
    while (true) {
        // 得到初始参数
        std::vector<double> init_param;
        for (size_t i = 0; i < optimized_key_vertices.size(); i++) {
            init_param.push_back(optimized_key_vertices[i].x_);
            init_param.push_back(optimized_key_vertices[i].y_);
        }
        // 初始化优化器
        std::shared_ptr<PathOptimizer> path_optimizer_ptr = std::make_shared<PathOptimizer>(init_param, kdtree, current_point, destination_point);
        // 进行路径优化
        path_optimizer_ptr->optimize();
        // 更新优化后的控制点
        optimized_key_vertices = path_optimizer_ptr->getKeyVertices();
        // 判断优化后的控制点之间的距离是否大于阈值,如果小于阈值则删除相应控制点,重新进行优化
        if (optimized_key_vertices.size() <= 2) {
            return -1;
        }
        while (true) {
            bool recheck = false;
            for (size_t i = 1; i < optimized_key_vertices.size(); i++) {
                if (optimized_key_vertices.size() <= 3) {
                    recheck = false;
                    break;
                }
                double distance = PathPlanningUtilities::calcDistance(optimized_key_vertices[i - 1], optimized_key_vertices[i]);
                if (Tools::isSmall(distance, Config::key_vertices_min_distance_)) {
                    if (i != optimized_key_vertices.size() - 1) {
                        optimized_key_vertices.erase(optimized_key_vertices.begin() + i);
                    } else {
                        optimized_key_vertices.erase(optimized_key_vertices.begin() + i - 1);
                    }
                    // ROS_INFO_STREAM("delete too close vertices, key vertices size: " << optimized_key_vertices.size());
                    recheck = true;
                    break;
                }
            }
            if (!recheck) {
                break;
            }
        }
        if (optimized_key_vertices.size() <= 2) {
            return -1;
        }
        // 得到新的样条曲线
        CubicSpline optimized_spline = CubicSpline(optimized_key_vertices);
        // 对曲线进行采样,得到每一段曲线
        std::vector<PathPlanningUtilities::Curve> optimized_curve_segments;
        for (size_t i = 0; i < optimized_spline.getArcLengths().size() - 1; i++) {
            PathPlanningUtilities::Curve optimized_curve_segment;
            int tmp_sample_num = static_cast<int>((optimized_spline.getArcLengths()[i+1] - optimized_spline.getArcLengths()[i]) / Config::sample_gap_);
            std::vector<double> tmp_samples = Tools::linspace(optimized_spline.getArcLengths()[i], optimized_spline.getArcLengths()[i+1], tmp_sample_num);
            if (tmp_sample_num < 2) {
                return -1;
            }
            for (auto sample: tmp_samples) {
                optimized_curve_segment.push_back(optimized_spline.getCurvePoint(sample));
            }
            optimized_curve_segments.push_back(optimized_curve_segment);
        }
        // 得到此时的总曲线
        for (size_t i = 0; i < optimized_curve_segments.size(); i++) {
            if (i == 0) {
                optimized_curve = optimized_curve_segments[i];
            } else {
                optimized_curve.insert(optimized_curve.end(), optimized_curve_segments[i].begin() + 1, optimized_curve_segments[i].end());
            }
        }
        // 判断是否停止优化
        if (optimize_iteration >= Config::max_optimization_iteration_) {
            // 停止优化
            break;
        }
        optimize_iteration++;
        // 得到需要插入的新关键点列表
        std::vector<std::pair<size_t, PathPlanningUtilities::Point2f>> insert_key_vertices;
        // 判断每一段是否发生碰撞
        for (size_t i = 0; i < optimized_curve_segments.size(); i++) {
            // 得到当前路径段
            PathPlanningUtilities::Curve optimized_curve_segment = optimized_curve_segments[i];
            // 计算路径每个点到障碍物的距离
            std::vector<double> min_distances_to_obstacle;
            for (auto curve_point: optimized_curve_segment) {
                std::vector<std::pair<float, float>> neighbors;
                std::vector<float> sq_distances;
                int result = kdtree.findKNeighbor(curve_point.position_.x_, curve_point.position_.y_, &neighbors, &sq_distances, 1);
                if (result > -1) {  
                    min_distances_to_obstacle.push_back(sqrt(sq_distances[0]));
                } else {
                    min_distances_to_obstacle.push_back(std::numeric_limits<double>::max());
                }
            }
            // 得到距离最小值
            auto min_distance_it = std::min_element(min_distances_to_obstacle.begin(), min_distances_to_obstacle.end());
            // 判断是否发生碰撞
            if (!Tools::isLarge(*min_distance_it, Config::robot_size_)) {
                // 发生碰撞,进行插入新关键点操作
                ROS_INFO("collision occur, new vertex insert");
                // 得到优化后路径与障碍物的最近点
                size_t insert_index = min_distance_it - min_distances_to_obstacle.begin();
                PathPlanningUtilities::Point2f nearest_point = optimized_curve_segment[insert_index].position_;
                // 计算路径段每一个点的里程
                std::vector<double> optimized_curve_segment_distances = Tools::calcCurveDistances(optimized_curve_segment);
                // 计算最近点离路径段起点的距离
                double nearest_point_distance_to_segment_start = optimized_curve_segment_distances[insert_index];
                // 计算最近点离路径段终点的距离
                double nearest_point_distance_to_segment_end = optimized_curve_segment_distances[optimized_curve_segment_distances.size() - 1] - optimized_curve_segment_distances[insert_index];
                // 判断距离是否大于阈值
                if (Tools::isLarge(optimized_curve_segment_distances[optimized_curve_segment_distances.size() - 1], 2.0 * Config::key_vertices_min_distance_)) {
                    // 路径段长度大于两倍的阈值
                    if (!Tools::isSmall(nearest_point_distance_to_segment_start, Config::key_vertices_min_distance_) && !Tools::isSmall(nearest_point_distance_to_segment_end, Config::key_vertices_min_distance_)) {
                        // 离前后都大于阈值,直接插入列表
                        insert_key_vertices.push_back(std::make_pair(i, nearest_point));
                    } else if (!Tools::isSmall(nearest_point_distance_to_segment_start, Config::key_vertices_min_distance_)) {
                        // 离前面大于阈值,离后面小于阈值
                        while (Tools::isSmall(nearest_point_distance_to_segment_end, Config::key_vertices_min_distance_)) {
                            insert_index--;
                            nearest_point_distance_to_segment_end = optimized_curve_segment_distances[optimized_curve_segment_distances.size() - 1] - optimized_curve_segment_distances[insert_index];
                        }
                        // 更新插入点
                        nearest_point = optimized_curve_segment[insert_index].position_;
                        // 将关键点加入插入列表
                        insert_key_vertices.push_back(std::make_pair(i, nearest_point));
                    } else {
                        // 离前面小于阈值,离后面大于阈值
                        while (Tools::isSmall(nearest_point_distance_to_segment_start, Config::key_vertices_min_distance_)) {
                            insert_index++;
                            nearest_point_distance_to_segment_start = optimized_curve_segment_distances[insert_index];
                        }
                        // 更新插入点
                        nearest_point = optimized_curve_segment[insert_index].position_;
                        // 将关键点加入插入列表
                        insert_key_vertices.push_back(std::make_pair(i, nearest_point));
                    }
                }
            }
        }
        // 判断新插入关键点的数量
        if (insert_key_vertices.size() == 0) {
            // 没有需要新插入的关键点,优化完成
            // ROS_INFO_STREAM("no new key vertices insert");
            break;
        } else {
            // ROS_INFO_STREAM("new key vertices insert");
            // 插入新的关键点
            int count = 1;
            for (auto new_key_vertex_pair: insert_key_vertices) {
                optimized_key_vertices.insert(optimized_key_vertices.begin() + new_key_vertex_pair.first + count, new_key_vertex_pair.second);
                count++;
            }
        }
    }
    return 0;
}

// 路径与障碍物的平均距离
bool MCRPlanner::pathAvgDistanceToObs(const PathPlanningUtilities::Curve &curve, const KDTree &kdtree, double &distance_to_obs) {
    // 计算路径每个点到障碍物的距离
    std::vector<double> min_distances_to_obstacle;
    for (auto curve_point: curve) {
        std::vector<std::pair<float, float>> neighbors;
        std::vector<float> sq_distances;
        int result = kdtree.findKNeighbor(curve_point.position_.x_, curve_point.position_.y_, &neighbors, &sq_distances, 1);
        if (result > -1) {  
            min_distances_to_obstacle.push_back(sqrt(sq_distances[0]));
        } else {
            min_distances_to_obstacle.push_back(std::numeric_limits<double>::max());
        }
    }
    // 计算距离平均值
    distance_to_obs = std::accumulate(min_distances_to_obstacle.begin(), min_distances_to_obstacle.end(), 0.0) / static_cast<double>(min_distances_to_obstacle.size());
    // 得到距离最小值
    auto min_distance_it = std::min_element(min_distances_to_obstacle.begin(), min_distances_to_obstacle.end());
    // 判断是否发生碰撞
    if (!Tools::isLarge(*min_distance_it, Config::robot_size_)) {
        return true;
    } else {
        return false;
    }
}

// 地图回调函数
void MCRPlanner::costmapCallback(nav_msgs::OccupancyGrid::ConstPtr map_msg) {
    // 解析地图的分辨率
    Config::grid_resolution_ = map_msg->info.resolution;
    // 进行格式转换
    this->costmap_mutex_.lock();
    this->costmap_ = GridMap(*map_msg);
    this->kdtree_mutex_.lock();
    this->kdtree_ = KDTree(this->costmap_);
    this->kdtree_mutex_.unlock();
    this->costmap_mutex_.unlock();
    // 修改标志位
    this->map_obtained_flag_mutex_.lock();
    if (!this->map_obtained_flag_) {
        this->map_obtained_flag_ = true;
    }
    this->map_obtained_flag_mutex_.unlock();
};

};
