/**
 * @file search_frontier.hpp
 * @brief
 *
 * @author ZhiHang_FU (fu_zhihang@126.com)
 * @version 1.0
 * @date 2024-04-02
 * @copyright Copyright (c) 2024  the author of this program
 */
#pragma once

#include <costmap_2d/costmap_2d.h>
#include <mutex>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <set>
#include <cmath>
#include <tf/transform_datatypes.h>

constexpr double R_TO_A = 180 / M_PI; // 弧度转角度比例 在编译阶段就完成了运算

namespace frontier_exploration_ns
{
    struct FrontierStruct
    {
        /* data */
        std::uint32_t size;
        double min_distance; // 所有有效栅格中距离robot最近的距离
        double cost;         // 代价
        double delta_angle;
        bool danger_flag;
        // point的默认构造函数就是0 0 0
        geometry_msgs::Point initial;
        geometry_msgs::Point centroid;
        geometry_msgs::Point middle;
        geometry_msgs::Point centroid_replace;
        std::vector<geometry_msgs::Point> points_vec;
        FrontierStruct() : size(1), min_distance(std::numeric_limits<double>::infinity()), delta_angle(0), danger_flag(false){};
    };
    /**
     * @brief 寻找前沿点的类
     */
    class FrontierSearchClass
    {
    public:
        FrontierSearchClass() = default;

        /**
         * @brief Construct a new Frontier Search Class object
         */
        FrontierSearchClass(costmap_2d::Costmap2D *costmap, double potential_scale, double gain_scale, double orientation_scale, double min_frontier_size);

        /**
         * @brief 从这个位置开始搜寻边界点
         */
        std::vector<FrontierStruct> SearchFrom(geometry_msgs::Pose pose);

    protected:
        /**
         * @brief 建立新的完整前沿边界结构体
         */
        FrontierStruct
        BuildNewFrontierStruct(unsigned int initial_cell_index, unsigned int robot_index, std::vector<bool> &frontier_flag);

        /**
         * @brief 判断是否是一个未访问的地图栅格
         */
        bool IsNewFrontierCell(unsigned int index, const std::vector<bool> &frontier_flag);

        double CalculateFrontierCost(const FrontierStruct &frontier);
        void CalculateFrontierCost(std::vector<FrontierStruct> &frontiers_vec, geometry_msgs::Pose);

    private:
        costmap_2d::Costmap2D *costmap_;
        unsigned char *map_;
        double gain_scale_, potential_scale_, orientation_scale_;
        unsigned int size_x_, size_y_;
        double min_frontier_size_;
    };
} // end of frontier_exploration_ns
