/*
    Copyright [2021] Jian ZhiQiang
*/

#ifndef KEY_VERTICES_OBTAIN_HPP_
#define KEY_VERTICES_OBTAIN_HPP_

#include "common/Point.hpp"
#include "common/Path.hpp"
#include "utilities/CubicSpline.hpp"
#include "utilities/LineSegment.hpp"

namespace mcrp_global_planner {

namespace KeyVerticesObtain {

// 道格拉斯-普客路径简化算法
class DouglasPeuckerSimplify{
 public:
    // 构造函数
    DouglasPeuckerSimplify() {};

    // 析构函数
    ~DouglasPeuckerSimplify() {};

    // 进行简化(获得路径)
    PathPlanningUtilities::Path simplifiedPath(const PathPlanningUtilities::Path &path, double tolerance) {
        // 初始化数据
        this->raw_path_ = path;
        this->tolerance_ = tolerance;
        this->tags_.resize(path.size(), 0);
        // 开始简化
        this->douglasPeuckerReduction(0, path.size() - 1);
        PathPlanningUtilities::Path result;
        for (size_t i = 0; i < this->tags_.size(); i++) {
            if (this->tags_[i]) {
                result.push_back(this->raw_path_[i]);
            }
        }
        return result;
    }

    // 进行简化(获得关键点下标)
    std::vector<size_t> keyPointsIndex(const PathPlanningUtilities::Path &path, double tolerance) {
        // 初始化数据
        this->raw_path_ = path;
        this->tolerance_ = tolerance;
        this->tags_.resize(path.size(), 0);
        // 开始简化
        this->douglasPeuckerReduction(0, path.size() - 1);
        std::vector<size_t> result;
        for (size_t i = 0; i < this->tags_.size(); i++) {
            if (this->tags_[i]) {
                result.push_back(i);
            }
        }
        return result;
    }

 private:
    // 迭代函数
    void douglasPeuckerReduction(size_t first_index, size_t last_index) {
        // std::cout << "first_index " << first_index << ", last_index " << last_index << std::endl;
        // 得到分段中的最远距离
        double max_dist = 0.0;
        size_t max_index = first_index;
        for (size_t index = first_index; index < last_index; index++) {
            double distance = this->perpendicularDistance(this->raw_path_[first_index], this->raw_path_[last_index], this->raw_path_[index]);
            if (Tools::isLarge(distance, max_dist)) {
                max_dist = distance;
                max_index = index;
            }
        }
        // std::cout << "max index " << max_index << ", max dist " << max_dist << std::endl;
        // 判断最优距离是否大于阈值
        if (Tools::isLarge(max_dist, this->tolerance_) && max_index != first_index) {
            // 如果大于阈值,则进行分段,再次进行迭代
            this->tags_[max_index] = true;
            this->douglasPeuckerReduction(first_index, max_index);
            this->douglasPeuckerReduction(max_index, last_index);
        } else {
            // 如果小于阈值则去除中间所有点,只留起点和终点
            this->tags_[first_index] = true;
            this->tags_[last_index] = true;
        }
    }

    // 计算点到直线的距离
    double perpendicularDistance (const PathPlanningUtilities::Point2f &point_start, const PathPlanningUtilities::Point2f &point_end, const PathPlanningUtilities::Point2f &point) {
        // 点到直线的距离公式法
        PathPlanningUtilities::LineSegment line = PathPlanningUtilities::LineSegment(point_start, point_end);
        double dist = line.pointToLineSegmentDistance(point.x_, point.y_);
        return dist;
    }

    PathPlanningUtilities::Path raw_path_;  // 输入未简化路径
    std::vector<bool> tags_;  // 保留的点下标
    double tolerance_;  // 偏移忍受程度
};

// 关键点扩增
double keyVerticesIncrement(const PathPlanningUtilities::Path &key_vertices, PathPlanningUtilities::Path &updated_key_vertices) {
    updated_key_vertices = key_vertices;
    // 关键点插值三次多项式曲线
    CubicSpline cubic_spline = CubicSpline(key_vertices);
    // 得到多项式曲线每一段的里程
    std::vector<double> arc_lengths = cubic_spline.getArcLengths();
    // 验证曲线段数与关键点的关系
    size_t count = 0;  // 增加的元素的数量
    double max_distance = 0.0;  // 全部路径段的最大间隔
    for (size_t i = 1; i < key_vertices.size(); i++) {
        // 构建直线段
        PathPlanningUtilities::LineSegment line_segment = PathPlanningUtilities::LineSegment(key_vertices[i - 1], key_vertices[i]);
        // 得到直线段对应的曲线段
        int sample_num = static_cast<int>((arc_lengths[i] - arc_lengths[i - 1]) / Config::sample_gap_);
        std::vector<double> samples = Tools::linspace(arc_lengths[i - 1], arc_lengths[i], sample_num);
        // 计算曲线与直线的距离
        double segment_max_distance = 0.0;
        PathPlanningUtilities::Point2f segment_max_distance_point = key_vertices[i - 1];
        for (auto sample: samples) {
            PathPlanningUtilities::Point2f sampled_point = cubic_spline.getPoint(sample);
            double distance = line_segment.pointToLineSegmentDistance(sampled_point.x_, sampled_point.y_);
            if (Tools::isLarge(distance, segment_max_distance)) {
                segment_max_distance = distance;
                segment_max_distance_point = sampled_point;
            }
        }
        // 更新最大距离
        if (Tools::isLarge(segment_max_distance, max_distance)) {
            max_distance = segment_max_distance;
        }
        // 判断距离是否大于阈值,如果大于阈值则插入新点
        if (Tools::isLarge(segment_max_distance, Config::key_vertices_obtain_tolerance_)) {
            // 计算新的关键点
            PathPlanningUtilities::Point2f new_key_vertice = line_segment.nearestPointOnLineSegment(segment_max_distance_point.x_, segment_max_distance_point.y_);
            assert (Tools::isEqual(segment_max_distance, PathPlanningUtilities::calcDistance(new_key_vertice, segment_max_distance_point)));
            // 插入原来的列表
            updated_key_vertices.insert(updated_key_vertices.begin() + i + count, new_key_vertice);
            count++;
        }
    }
    return max_distance;
}

};

};

#endif  // KEY_VERTICES_OBTAIN_HPP_