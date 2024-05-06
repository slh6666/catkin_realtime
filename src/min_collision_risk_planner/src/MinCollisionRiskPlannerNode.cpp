#include "planning/MinCollisionRiskPlannerNode.hpp"

namespace mcrp_global_planner {

// 开始进行规划
void MinCollisionRiskPlannerNode::startPlanning() {
    // 初始化与ros链接
    this->initROSConnection();
    // 开启ros线程和规划线程
    std::thread ros_listener_thread(&MinCollisionRiskPlannerNode::listenRosMSGThread, this);
    std::thread planning_thread(&MinCollisionRiskPlannerNode::planningThread, this);
    ros_listener_thread.join();
    planning_thread.join();
};

// 监听ros线程
void MinCollisionRiskPlannerNode::listenRosMSGThread() {
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
};

// 开始规划线程
void MinCollisionRiskPlannerNode::planningThread() {
    // 等待数据准备完成
    bool data_ready = false;
    while (!data_ready) {
        this->destination_obtained_flag_mutex_.lock();
        this->current_point_obtained_flag_mutex_.lock();
        this->map_obtained_flag_mutex_.lock();
        data_ready = this->current_point_obtained_flag_ && this->destination_obtained_flag_ && this->map_obtained_flag_;
        this->map_obtained_flag_mutex_.unlock();
        this->current_point_obtained_flag_mutex_.unlock();
        this->destination_obtained_flag_mutex_.unlock();
        ros::Rate(1).sleep();
    }
    // 进行规划
    int count = 0;
    bool is_print = false;
    while (ros::ok()) {
        // 判断是否进行规划
        this->planning_start_flag_mutex_.lock();
        bool planning_start_flag = this->planning_start_flag_;
        this->planning_start_flag_mutex_.unlock();
        if (!planning_start_flag) {
            ros::Rate(1).sleep();
            continue;
        }
        if(count % 10 == 0){
            is_print = true;
            count = 0;
        }
        else{
            is_print = false;
        }
        count++;
        // 开始规划 only planning once after change goal
	//已注释 更新全局规划
	//this->planning_start_flag_mutex_.lock();
        //this->planning_start_flag_ = false;
        //this->planning_start_flag_mutex_.unlock();
	 
        if(is_print)
            ROS_INFO_STREAM("all messge obtained, planning begin");  //change

        // 获取定位、目的地和感知结果
        this->current_point_mutex_.lock();
        PathPlanningUtilities::Point2f current_point = this->current_point_;
        this->current_point_mutex_.unlock();
        this->destination_point_mutex_.lock();
        PathPlanningUtilities::Point2f destination_point = this->destination_point_;
        this->destination_point_mutex_.unlock();
        this->costmap_mutex_.lock();
        GridMap costmap = this->costmap_;
        this->costmap_mutex_.unlock();
        this->kdtree_mutex_.lock();
        KDTree kdtree = this->kdtree_;
        this->kdtree_mutex_.unlock();

        // 计算当前点与目的地之间的距离
        double destination_distance = PathPlanningUtilities::calcDistance(current_point, destination_point);

        if (Tools::isSmall(destination_distance, Config::destination_tolerance_)) {
            ROS_INFO_STREAM("destination reached");
            // 发布到达目的地规划信号
            std_msgs::Int8 planning_result_flag;
            planning_result_flag.data = 0;
            this->planning_result_flag_pub_.publish(planning_result_flag);
            continue;
        }

        // 初始路径生成
        clock_t start_init_path_search_time = clock();
        std::shared_ptr<InitPathSearch::LazyThetaStarPlanner> lazy_theta_star_planner_ptr(new InitPathSearch::LazyThetaStarPlanner(Config::robot_size_));
        PathPlanningUtilities::Path init_path;
        int init_path_search_result = lazy_theta_star_planner_ptr->planning(current_point, destination_point, costmap, kdtree, init_path);
        // 判断是否生成成功
        if (init_path_search_result == -1) {
            ROS_INFO_STREAM("init path searching failed");
            // 发布规划失败信号
            std_msgs::Int8 planning_result_flag;
            planning_result_flag.data = -1;
            this->planning_result_flag_pub_.publish(planning_result_flag);
            continue;
        }
        // 计算时间开销
        clock_t end_init_path_search_time = clock();
        if(is_print)
            ROS_INFO_STREAM("init path searching finished, time consuming is " << static_cast<double>(end_init_path_search_time - start_init_path_search_time) * 1000.0 / CLOCKS_PER_SEC << " ms");
        // 进行格式转换和输出  //change
        nav_msgs::Path init_path_output;
        init_path_output.header.frame_id = "map";
        init_path_output.header.stamp = ros::Time::now();
        for (size_t i = 0; i < init_path.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.header = init_path_output.header;
            pose.pose.position.x = init_path[i].x_;
            pose.pose.position.y = init_path[i].y_;
            init_path_output.poses.push_back(pose);
        }
        this->init_path_pub_.publish(init_path_output);
        // 进行可视化
        // 清空之前的可视化  //change
        visualization_msgs::MarkerArray delete_marker_array;
        delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
        // this->init_path_vis_pub_.publish(delete_marker_array);
        // // 可视化
        // this->init_path_vis_pub_.publish(VisualizationMethods::visualizeCurveToMarker(init_path_output, Config::robot_size_, VisualizationMethods::color(1, 0, 0, 0.1)));

        // 计算初始路径每一个点与障碍物的距离
        // 保存日志信息
        if (this->save_logs_flag_) {
            // 首先读取本工程的目录
            std::string file_path = ros::package::getPath("min_collision_risk_planner");
            // 创建数据文件夹
            file_path += "/logs/init_path_info/";
            Tools::resetLogFile(file_path);
            // 读取系统当前时钟作为文件名称
            file_path += Tools::returnCurrentTimeAndDate() + ".csv";
            // 打开文件
            std::ofstream file(file_path);
            // 写入文件
            if (file) {
                // 计算初始路径上每一采样点与障碍物的最小距离
                for (auto path_point: init_path) {
                    std::vector<std::pair<float, float>> neighbors;
                    std::vector<float> sq_distances;
                    int result = kdtree.findKNeighbor(path_point.x_, path_point.y_, &neighbors, &sq_distances, 1);
                    if (result > -1) {  
                        file << sqrt(sq_distances[0]) << "\n";
                    } else {
                        file << std::numeric_limits<double>::max() << "\n";
                    }
                }
            }
            file.close();
        }
        
        // 关键点提取
        clock_t start_key_vertices_obtain_time = clock();
        PathPlanningUtilities::Path key_vertices;
        this->keyVerticesObtain(init_path, key_vertices);
        // 计算时间开销
        clock_t end_key_vertices_obtain_time = clock();
        if(is_print)
            ROS_INFO_STREAM("key vertices obtain finished, time consuming is " << static_cast<double>(end_key_vertices_obtain_time - start_key_vertices_obtain_time) * 1000.0 / CLOCKS_PER_SEC << " ms");
        // 进行可视化
        // 清空之前的可视化  //change
        this->key_vertices_pub_.publish(delete_marker_array);
        // 可视化
        visualization_msgs::MarkerArray key_vertices_marker_array;
        for (size_t i = 0; i < key_vertices.size(); i++) {
            key_vertices_marker_array.markers.push_back(VisualizationMethods::visualizeSphere(key_vertices[i].x_, key_vertices[i].y_, 0.05, VisualizationMethods::color(1, 0, 0, 1), i));
        }
        this->key_vertices_pub_.publish(key_vertices_marker_array);

        nav_msgs::Path optimized_curve_output;
        optimized_curve_output.header.frame_id = "map";
        optimized_curve_output.header.stamp = ros::Time::now();
        for (size_t i = 0; i < key_vertices.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.header = optimized_curve_output.header;
            pose.pose.position.x = key_vertices[i].x_;
            pose.pose.position.y = key_vertices[i].y_;
            optimized_curve_output.poses.push_back(pose);
        }
        this->optimized_path_pub_.publish(optimized_curve_output);

        // // 关键点插值三次多项式曲线
        // CubicSpline cubic_spline = CubicSpline(key_vertices);
        // // 得到采样
        // int sample_num = static_cast<int>(cubic_spline.getMaxArcLength() / Config::sample_gap_);
        // std::vector<double> samples = Tools::linspace(0.0, cubic_spline.getMaxArcLength(), sample_num);
        // // 得到路径
        // PathPlanningUtilities::Curve processed_curve;
        // for (auto sample: samples) {
        //     processed_curve.push_back(cubic_spline.getCurvePoint(sample));
        // }
        // // 进行格式转换和输出  //change
        // // nav_msgs::Path processed_curve_output;
        // // processed_curve_output.header.frame_id = "map";
        // // processed_curve_output.header.stamp = ros::Time::now();
        // // for (size_t i = 0; i < processed_curve.size(); i++) {
        // //     geometry_msgs::PoseStamped pose;
        // //     pose.header = processed_curve_output.header;
        // //     pose.pose.position.x = processed_curve[i].position_.x_;
        // //     pose.pose.position.y = processed_curve[i].position_.y_;
        // //     processed_curve_output.poses.push_back(pose);
        // // }
        // // this->processed_path_pub_.publish(processed_curve_output);
        // // // 进行可视化输出
        // // // 删除之前的可视化
        // // this->processed_path_vis_pub_.publish(delete_marker_array);
        // // // 可视化
        // // this->processed_path_vis_pub_.publish(VisualizationMethods::visualizeCurveToMarker(processed_curve_output, Config::robot_size_, VisualizationMethods::color(0, 1, 0, 0.1)));

        // // 进行曲线优化
        // clock_t start_path_optimization_time = clock();
        // PathPlanningUtilities::Curve optimized_curve;
        // PathPlanningUtilities::Path optimized_key_vertices = key_vertices;
        // this->pathOptimizing(optimized_curve, optimized_key_vertices, current_point, destination_point, costmap, kdtree);
        // // 计算时间开销
        // clock_t end_path_optimization_time = clock();
        // if(is_print)
        //     ROS_INFO_STREAM("path optimization finished, time consuming is " << static_cast<double>(end_path_optimization_time - start_path_optimization_time) * 1000.0 / CLOCKS_PER_SEC << " ms");
        // // 进行格式转换和输出
        // nav_msgs::Path optimized_curve_output;
        // optimized_curve_output.header.frame_id = "map";
        // optimized_curve_output.header.stamp = ros::Time::now();
        // for (size_t i = 0; i < optimized_curve.size(); i++) {
        //     geometry_msgs::PoseStamped pose;
        //     pose.header = optimized_curve_output.header;
        //     pose.pose.position.x = optimized_curve[i].position_.x_;
        //     pose.pose.position.y = optimized_curve[i].position_.y_;
        //     optimized_curve_output.poses.push_back(pose);
        // }
        // this->optimized_path_pub_.publish(optimized_curve_output);
        // // debug
        // // 进行可视化输出
        // // 删除之前的可视化  //change
        // // this->optimized_path_vis_pub_.publish(delete_marker_array);
        // // // 可视化
        // // this->optimized_path_vis_pub_.publish(VisualizationMethods::visualizeCurveToMarker(optimized_curve_output, Config::robot_size_, VisualizationMethods::color(0, 0, 1, 0.1)));
        // // // 进行可视化输出
        // // this->optimized_key_vertices_pub_.publish(delete_marker_array);
        // // visualization_msgs::MarkerArray optimized_key_vertices_marker_array;
        // // for (size_t i = 0; i < optimized_key_vertices.size(); i++) {
        // //     optimized_key_vertices_marker_array.markers.push_back(VisualizationMethods::visualizeSphere(optimized_key_vertices[i].x_, optimized_key_vertices[i].y_, 0.05, VisualizationMethods::color(0, 0, 1, 1), i));
        // // }
        // // this->optimized_key_vertices_pub_.publish(optimized_key_vertices_marker_array);

        // // 发布导航路径信息  //change
        // // navigation_msgs::Path navigation_path_msg;
        // // navigation_path_msg.header.frame_id = "map";
        // // navigation_path_msg.header.stamp = ros::Time::now();
        // // double arc_length = 0.0;
        // // for (size_t i = 0; i < optimized_curve.size(); i++) {
        // //     navigation_msgs::PathPoint path_point;
        // //     path_point.x = optimized_curve[i].position_.x_;
        // //     path_point.y = optimized_curve[i].position_.y_;
        // //     path_point.yaw = optimized_curve[i].theta_;
        // //     path_point.kappa = optimized_curve[i].kappa_;
        // //     path_point.arc_length = arc_length;
        // //     if (i < optimized_curve.size() - 1) {
        // //         arc_length += PathPlanningUtilities::calcDistance(optimized_curve[i].position_, optimized_curve[i + 1].position_); 
        // //     }
        // //     navigation_path_msg.points.push_back(path_point);
        // // }
        // // this->navigation_path_pub_.publish(navigation_path_msg);
        
        // 发布规划成功信号
        std_msgs::Int8 planning_result_flag;
        planning_result_flag.data = 1;
        this->planning_result_flag_pub_.publish(planning_result_flag);

        // 保存日志信息
        // if (this->save_logs_flag_) {
        //     // 首先读取本工程的目录
        //     std::string file_path = ros::package::getPath("min_collision_risk_planner");
        //     // 创建数据文件夹
        //     file_path += "/logs/optimized_path_info/";
        //     Tools::resetLogFile(file_path);
        //     // 读取系统当前时钟作为文件名称
        //     file_path += Tools::returnCurrentTimeAndDate() + ".csv";
        //     // 打开文件
        //     std::ofstream file(file_path);
        //     // 写入文件
        //     if (file) {
        //         // 内容包括路径长度,路径最大曲率,路径平均曲率,路径与障碍物的平均距离,路径与障碍物的最小距离,是否发生碰撞
        //         // 计算曲线长度
        //         double optimized_curve_length = Tools::calcCurveLength(optimized_curve);
        //         // 计算曲线的最大曲率
        //         double optimized_curve_max_curvature = Tools::calcMaxKappaForCurve(optimized_curve);
        //         // 计算曲线的平均曲率
        //         double optimized_curve_avg_curvature = 0.0;
        //         for (auto curve_point: optimized_curve) {
        //             optimized_curve_avg_curvature += std::fabs(curve_point.kappa_);
        //         }
        //         optimized_curve_avg_curvature = optimized_curve_avg_curvature / static_cast<double>(optimized_curve.size());
        //         // 计算每个点到障碍物的距离
        //         std::vector<double> min_distances_to_obstacle;
        //         for (auto curve_point: optimized_curve) {
        //             std::vector<std::pair<float, float>> neighbors;
        //             std::vector<float> sq_distances;
        //             int result = kdtree.findKNeighbor(curve_point.position_.x_, curve_point.position_.y_, &neighbors, &sq_distances, 1);
        //             if (result > -1) {  
        //                 min_distances_to_obstacle.push_back(sqrt(sq_distances[0]));
        //             } else {
        //                 min_distances_to_obstacle.push_back(std::numeric_limits<double>::max());
        //             }
        //         }
        //         // 计算曲线到障碍物的平均距离
        //         double optimized_curve_avg_distance_to_obs = std::accumulate(min_distances_to_obstacle.begin(), min_distances_to_obstacle.end(), 0.0) / static_cast<double>(min_distances_to_obstacle.size());
        //         // 判断路径是否发生碰撞
        //         auto optimized_curve_min_distance_to_obs_ptr = std::min_element(min_distances_to_obstacle.begin(), min_distances_to_obstacle.end());
        //         double optimized_curve_min_distance_to_obs = *optimized_curve_min_distance_to_obs_ptr;
        //         ROS_INFO_STREAM("optimized_curve_min_distance_to_obs: " << optimized_curve_min_distance_to_obs);
        //         int collision_judge = 0;
        //         if (!Tools::isLarge(optimized_curve_min_distance_to_obs, Config::robot_size_)) {
        //             collision_judge = 1;
        //         }
        //         // 计算总时间开销
        //         double time_consumption = static_cast<double>(end_init_path_search_time - start_init_path_search_time) * 1000.0 / CLOCKS_PER_SEC + static_cast<double>(end_key_vertices_obtain_time - start_key_vertices_obtain_time) * 1000.0 / CLOCKS_PER_SEC + static_cast<double>(end_path_optimization_time - start_path_optimization_time) * 1000.0 / CLOCKS_PER_SEC;
        //         // 保存信息
        //         file << "start point:" << current_point.x_ << "," << current_point.y_ << "\n";
        //         file << "destination point:" << destination_point.x_ << "," << destination_point.y_ << "\n";
        //         file << "curve length:" << optimized_curve_length << "\n";
        //         file << "average curvature:" << optimized_curve_avg_curvature << "\n";
        //         file << "maximum curvature:" << optimized_curve_max_curvature << "\n";
        //         file << "average distance to obstacles:" << optimized_curve_avg_distance_to_obs << "\n";
        //         file << "minimum distance to obstacles:" << optimized_curve_min_distance_to_obs << "\n";
        //         file << "collision:" << collision_judge << "\n";
        //         file << "time consumption:" << time_consumption << "\n";
        //         // 保存路径信息
        //         file << "path info:\n";
        //         for (size_t i = 0; i < optimized_curve.size(); i++) {
        //             file << optimized_curve[i];
        //         }
        //         // 保存障碍物信息
        //         file << "distance to obstacls:\n";
        //         for (size_t i = 0; i < min_distances_to_obstacle.size(); i++) {
        //             file << min_distances_to_obstacle[i] << "\n";
        //         }
        //     }
        //     // 关闭文件
        //     file.close();
        // }
    }
};

// 获取关键点
void MinCollisionRiskPlannerNode::keyVerticesObtain(const PathPlanningUtilities::Path &init_path, PathPlanningUtilities::Path &key_vertices) {
    std::shared_ptr<KeyVerticesObtain::DouglasPeuckerSimplify> key_vertices_obtainer_ptr(new KeyVerticesObtain::DouglasPeuckerSimplify());
    // 进行提取
    key_vertices = key_vertices_obtainer_ptr->simplifiedPath(init_path, Config::key_vertices_obtain_tolerance_);
    assert(key_vertices.size() >= 2);

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
                // ROS_INFO_STREAM("key vertices increase finished: success");  //change
                break;
            }
            // 判断迭代次数是否超越上限
            if (iteration_count > Config::key_vertices_increase_max_iteration_) {
                // ROS_INFO_STREAM("key vertices increase finished: out of max iteration");  //change
                break;
            }
        }
    } else {
        PathPlanningUtilities::Point2f new_key_vertex;
        new_key_vertex.x_ = (key_vertices[0].x_ + key_vertices[1].x_) * 0.5;
        new_key_vertex.y_ = (key_vertices[0].y_ + key_vertices[1].y_) * 0.5;
        key_vertices.insert(key_vertices.begin() + 1, new_key_vertex);
    }

}

// 进行路径优化
void MinCollisionRiskPlannerNode::pathOptimizing(PathPlanningUtilities::Curve &optimized_curve, PathPlanningUtilities::Path &optimized_key_vertices, const PathPlanningUtilities::Point2f &current_point, const PathPlanningUtilities::Point2f &destination_point, const GridMap &costmap, const KDTree &kdtree) {
    int optimize_iteration = 0;  // 优化迭代次数
    while (true) {
        // 得到初始参数
        std::vector<double> init_param;
        for (size_t i = 0; i < optimized_key_vertices.size(); i++) {
            init_param.push_back(optimized_key_vertices[i].x_);
            init_param.push_back(optimized_key_vertices[i].y_);
        }
        // 初始化优化器
        std::shared_ptr<PathOptimizer> path_optimizer_ptr(new PathOptimizer(init_param, kdtree, current_point, destination_point));
        // 进行路径优化
        path_optimizer_ptr->optimize();
        // 更新优化后的控制点
        optimized_key_vertices = path_optimizer_ptr->getKeyVertices();
        // 判断优化后的控制点之间的距离是否大于阈值,如果小于阈值则删除相应控制点,重新进行优化
        assert(optimized_key_vertices.size() > 2);
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
                    // ROS_INFO_STREAM("delete too close vertices, key vertices size: " << optimized_key_vertices.size());  //change
                    recheck = true;
                    break;
                }
            }
            if (!recheck) {
                break;
            }
        }
        assert(optimized_key_vertices.size() > 2);
        // 得到新的样条曲线
        CubicSpline optimized_spline = CubicSpline(optimized_key_vertices);
        // 对曲线进行采样,得到每一段曲线
        std::vector<PathPlanningUtilities::Curve> optimized_curve_segments;
        for (size_t i = 0; i < optimized_spline.getArcLengths().size() - 1; i++) {
            PathPlanningUtilities::Curve optimized_curve_segment;
            int tmp_sample_num = static_cast<int>((optimized_spline.getArcLengths()[i+1] - optimized_spline.getArcLengths()[i]) / Config::sample_gap_);
            std::vector<double> tmp_samples = Tools::linspace(optimized_spline.getArcLengths()[i], optimized_spline.getArcLengths()[i+1], tmp_sample_num);
            assert(tmp_sample_num >= 2);
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
}

// 初始化与ROS链接
void MinCollisionRiskPlannerNode::initROSConnection() {
    // 初始化ROS消息订阅
    // 定位消息
    std::string odom_topic;
    this->nh_.getParam("odom_topic", odom_topic);
    this->odom_sub_ = this->nh_.subscribe(odom_topic, 1, &MinCollisionRiskPlannerNode::odomCallback, this);
    // 目的地消息
    std::string destination_topic;
    this->nh_.getParam("destination_topic", destination_topic);
    this->destination_sub_ = this->nh_.subscribe(destination_topic, 1, &MinCollisionRiskPlannerNode::destinationCallback, this);
    // 全局地图消息
    std::string costmap_topic;
    this->nh_.getParam("costmap_topic", costmap_topic);
    this->costmap_sub_ = this->nh_.subscribe(costmap_topic, 1, &MinCollisionRiskPlannerNode::costmapCallback, this);
    // 初始化ROS消息发布
    // 输出初始路径
    std::string init_path_topic;
    this->nh_.getParam("init_path_topic", init_path_topic);
    this->init_path_pub_ = this->nh_.advertise<nav_msgs::Path>(init_path_topic, 10);
    this->init_path_vis_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(init_path_topic.append(std::string("_vis")), 10);
    // 输出关键点
    std::string key_vertices_topic;
    this->nh_.getParam("key_vertices_topic", key_vertices_topic);
    this->key_vertices_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(key_vertices_topic, 10);
    // 输出关键点插值路径
    std::string processed_path_topic;
    this->nh_.getParam("processed_path_topic", processed_path_topic);
    this->processed_path_pub_ = this->nh_.advertise<nav_msgs::Path>(processed_path_topic, 10);
    this->processed_path_vis_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(processed_path_topic.append(std::string("_vis")), 10);
    // 输出优化后路径
    std::string optimized_path_topic;
    this->nh_.getParam("optimized_path_topic", optimized_path_topic);
    this->optimized_path_pub_ = this->nh_.advertise<nav_msgs::Path>(optimized_path_topic, 10);
    this->optimized_path_vis_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(optimized_path_topic.append(std::string("_vis")), 10);
    // 输出优化后的控制点
    std::string optimized_key_vertices_topic;
    this->nh_.getParam("optimized_key_vertices_topic", optimized_key_vertices_topic);
    this->optimized_key_vertices_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(optimized_key_vertices_topic, 10);
    // 规划结果标志位
    std::string planning_result_flag_topic;
    this->nh_.getParam("planning_result_flag_topic", planning_result_flag_topic);
    this->planning_result_flag_pub_ = this->nh_.advertise<std_msgs::Int8>(planning_result_flag_topic, 10);
    // 导航消息输出
    std::string navigation_path_topic;
    this->nh_.getParam("navigation_path_topic", navigation_path_topic);
    this->navigation_path_pub_ = this->nh_.advertise<navigation_msgs::Path>(navigation_path_topic, 10);
    // 初始化常量
    // 是否保存日志信息
    this->nh_.getParam("save_logs_flag", this->save_logs_flag_);
};

// 定位回调函数
void MinCollisionRiskPlannerNode::odomCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {
    // 进行位置信息的转换
    PathPlanningUtilities::Point2f current_point;
    current_point.x_ = msg->pose.pose.position.x;
    current_point.y_ = msg->pose.pose.position.y;
    // 保存信息
    this->current_point_mutex_.lock();
    this->current_point_ = current_point;
    this->current_point_mutex_.unlock();
    // 修改标志位
    this->current_point_obtained_flag_mutex_.lock();
    if (!this->current_point_obtained_flag_) {
        this->current_point_obtained_flag_ = true;
        // ROS_INFO_STREAM("position updated");
    }
    this->current_point_obtained_flag_mutex_.unlock();
};

// 目的地回调函数
void MinCollisionRiskPlannerNode::destinationCallback(geometry_msgs::PoseStamped::ConstPtr pose_msg) {
    // 保存信息
    this->destination_point_mutex_.lock();
    this->destination_point_.x_ = pose_msg->pose.position.x;
    this->destination_point_.y_ = pose_msg->pose.position.y;
    this->destination_point_mutex_.unlock();
    // this->destination_point_.x_ = -4.442267512159894;
    // this->destination_point_.y_ = 6.1834673448622155;
    // 修改标志位
    this->destination_obtained_flag_mutex_.lock();
    if (!this->destination_obtained_flag_) {
        this->destination_obtained_flag_ = true;
        // ROS_INFO_STREAM("destination updated");
    }
    this->destination_obtained_flag_mutex_.unlock();
    // 开启规划
    this->planning_start_flag_mutex_.lock();
    this->planning_start_flag_ = true;
    this->planning_start_flag_mutex_.unlock();
};

// 地图回调函数
void MinCollisionRiskPlannerNode::costmapCallback(nav_msgs::OccupancyGrid::ConstPtr map_msg) {
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
        // ROS_INFO_STREAM("map updated");
    }
    this->map_obtained_flag_mutex_.unlock();
};

};
