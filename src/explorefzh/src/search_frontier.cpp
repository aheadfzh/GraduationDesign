/**
 * @file search_frontier.cpp
 * @brief
 *
 * @author ZhiHang_FU (fu_zhihang@126.com)
 * @version 1.0
 * @date 2024-04-02
 * @copyright Copyright (c) 2024  the author of this program
 */
#include "../include/search_frontier.hpp"
#include "../include/costmap_tools.h" // 因为在这头文件中定义了存在函数定义所以不能放在另一个头文件 不然报重复定义错误

// 调用函数是必须考虑是定义在哪个命名空间内的

namespace frontier_exploration_ns
{
    using costmap_2d::FREE_SPACE;      // 空闲栅格类型
    using costmap_2d::LETHAL_OBSTACLE; // 占据栅格类型
    using costmap_2d::NO_INFORMATION;  // 未知栅格类型

    FrontierSearchClass::FrontierSearchClass(costmap_2d::Costmap2D *costmap, double potential_scale, double gain_scale, double min_frontier_size) : costmap_(costmap), gain_scale_(gain_scale), potential_scale_(potential_scale), min_frontier_size_(min_frontier_size) {} // 构造函数

    /**
     * @brief 从当前位置（世界坐标系）开始搜寻前沿集合  返回找到满足最小阈值的前沿容器
     */
    std::vector<FrontierStruct> FrontierSearchClass::SearchFrom(geometry_msgs::Point position)
    {
        std::vector<FrontierStruct> frontiers_vec; // 边界点集 的容器

        unsigned int mx, my;
        if (!costmap_->worldToMap(position.x, position.y, mx, my)) // 切换世界坐标系到栅格坐标系
        {
            ROS_ERROR("----Robot out of costmap bounds,cannot search for frontiers----");
            return frontiers_vec; // 返回空列表
        }
        // 作用是在当前作用域中对 Costmap2D 对象中的互斥锁进行自动加锁和解锁，保证了对共享资源的安全访问。
        std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
        // 当 lock 对象的生命周期结束时（即超出作用域时），会自动调用析构函数，解锁 mutex_，确保在作用域结束时自动释放互斥锁，避免了手动管理锁的繁琐和容易出错的问题。

        map_ = costmap_->getCharMap(); // 指针 对其取值获取成本地图的字符形式，表示地图上每个单元格的状态（栅格值 0-255）
        size_x_ = costmap_->getSizeInCellsX();
        size_y_ = costmap_->getSizeInCellsY();

        // 初始标志容器  分别跟踪前沿的栅格 和 和已访问的栅格的状态
        std::vector<bool> frontiers_flag(size_x_ * size_y_, false);
        std::vector<bool> visited_flag(size_x_ * size_y_, false);

        std::queue<unsigned int> bfs_queue; // 广度优先搜索的队列

        unsigned int clear_index, pos_index = costmap_->getIndex(mx, my); // 将机器人当前位置的栅格坐标转换为一维索引

        // nearestCell 函数的目的是在机器人当前位置附近找到最近的自由空间单元
        // 使用 nearestCell 函数查找最近的自由空间单元，将其索引存储在 clear 中
        if (nearestCell(clear_index, pos_index, FREE_SPACE, *costmap_))
            bfs_queue.push(clear_index);
        else // 未找到就将机器人当前位置入队
        {
            bfs_queue.push(pos_index);
            ROS_WARN("----Could not find nearby clear cell to start search frontier----");
        }
        visited_flag[bfs_queue.front()] = true; // 首元素栅格已经被访问

        while (!bfs_queue.empty()) // 广度优先搜索 先进后出
        {
            unsigned int index_queue_front = bfs_queue.front();
            bfs_queue.pop(); // 出队

            for (auto iter_nbr_index : nhood4(index_queue_front, *costmap_)) // 遍历这四个相邻的cell
            {
                // 1. 确认是空闲栅格且未访问
                // 2. 如果第一次传入的是pos_index则意思没有寻找到最近free栅格，状态值小于等于起点栅格的状态值时，才将该栅格加入到探索队列中
                if (map_[iter_nbr_index] <= map_[index_queue_front] && !visited_flag[iter_nbr_index])
                {
                    visited_flag[iter_nbr_index] = true;
                    bfs_queue.push(iter_nbr_index); // 将这个邻近栅格入队
                }
                else if (IsNewFrontierCell(iter_nbr_index, frontiers_flag))
                {
                    // 标记当前栅格是有效边界位置未知栅格
                    frontiers_flag[iter_nbr_index] = true;
                    FrontierStruct new_frontier = BuildNewFrontierStruct(iter_nbr_index, pos_index, frontiers_flag);
                    if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) // 最小尺寸阈值判断
                        frontiers_vec.push_back(new_frontier);                               // 放入到候选访问前沿容器中去
                }
            }
            /* code */
        }

        // for (auto &iter_frontier : frontiers_vec)
        //     iter_frontier.cost = CalculateFrontierCost(iter_frontier);
        CalculateFrontierCost(frontiers_vec);

        std::sort(frontiers_vec.begin(), frontiers_vec.end(), [](const FrontierStruct &f1, const FrontierStruct &f2)
                  { return f1.cost < f2.cost; }); // 对候选前沿点集合进行排序 Lambda表达式

        return frontiers_vec;
    }

    /**
     * @brief  得到一个新的边界
     * @return FrontierStruct
     */
    FrontierStruct FrontierSearchClass::BuildNewFrontierStruct(unsigned int initial_cell_index, unsigned int robot_index, std::vector<bool> &frontier_flag)
    {
        FrontierStruct new_frontier;
        unsigned int mx, my;
        costmap_->indexToCells(initial_cell_index, mx, my);                           // 一维索引转换为栅格地图坐标
        costmap_->mapToWorld(mx, my, new_frontier.initial.x, new_frontier.initial.y); // 栅格地图坐标转换为世界坐标 作为初始点

        std::queue<unsigned int> bfs_queue;
        bfs_queue.push(initial_cell_index);
        unsigned int rx, ry;
        double robot_x, robot_y;
        costmap_->indexToCells(robot_index, rx, ry);    // 一维索引转换为栅格地图坐标
        costmap_->mapToWorld(rx, ry, robot_x, robot_y); // 栅格地图转换为世界坐标
        // 广度优先搜索
        while (!bfs_queue.empty())
        {
            unsigned int index_queue_front = bfs_queue.front();
            bfs_queue.pop(); // 出队

            // 遍历8邻域
            for (auto iter_nbr_index : nhood8(index_queue_front, *costmap_))
            {
                if (IsNewFrontierCell(iter_nbr_index, frontier_flag))
                {
                    frontier_flag[iter_nbr_index] = true; // 标记确定是有效边界未知栅格

                    unsigned int cx, cy;
                    double wx, wy;
                    costmap_->indexToCells(iter_nbr_index, cx, cy);
                    costmap_->mapToWorld(cx, cy, wx, wy);
                    geometry_msgs::Point point;
                    point.x = wx;
                    point.y = wy;
                    new_frontier.points_vec.push_back(point); // 放入边界栅格容器中

                    new_frontier.size++;           // 累计新边界的栅格总数
                    new_frontier.centroid.x += wx; // 累计 为计算质心
                    new_frontier.centroid.y += wy;

                    // 计算栅格距离robot的距离
                    double distance = sqrt(pow((double(robot_x) - double(wx)), 2.0) + pow((double(robot_y) - double(wy)), 2.0));
                    if (distance < new_frontier.min_distance)
                    {
                        new_frontier.min_distance = distance; // 更新边界所有有效栅格中离robot最近距离
                        new_frontier.middle.x = wx;
                        new_frontier.middle.y = wy;
                    }
                    bfs_queue.push(iter_nbr_index);
                }
            }
        }

        new_frontier.centroid.x /= new_frontier.size;
        new_frontier.centroid.y /= new_frontier.size;
        return new_frontier;
    }

    /**
     * @brief 对于未知栅格判断是否是新的 有效边界栅格
     */
    bool FrontierSearchClass::IsNewFrontierCell(unsigned index, const std::vector<bool> &frontier_flag)
    {

        if (map_[index] != NO_INFORMATION || frontier_flag[index])
            return false;

        for (auto iter_nbr_index : nhood4(index, *costmap_)) // 4邻域栅格中至少有一个是空白栅格
        {
            if (map_[iter_nbr_index] == FREE_SPACE)
                return true;
        }

        return false;
    }

    /**
     * @brief 计算代价
     */
    double FrontierSearchClass::CalculateFrontierCost(const FrontierStruct &frontier)
    {
        // TODO：应该归一化处理
        return (potential_scale_ * frontier.min_distance * costmap_->getResolution()) - (gain_scale_ * frontier.size * costmap_->getResolution());
    }

    /**
     * @brief 计算代价 归一化形式 考虑转角航向代价
     * @param  frontiers_vec    My Param doc
     */
    void FrontierSearchClass::CalculateFrontierCost(std::vector<FrontierStruct> &frontiers_vec)
    {
        double max_min_distance = 1e-4;
        double max_size = 1e-4;
        double max_delte_angle = 1e-4;

        for (const auto &iter_frontier : frontiers_vec)
        {
            if (iter_frontier.min_distance > max_min_distance)
                max_min_distance = iter_frontier.min_distance;

            if (iter_frontier.size > max_size)
                max_size = iter_frontier.size;
            // if (iter_frontier.delta_angle > max_delte_angle)
            //     max_delte_angle = iter_frontier.delta_angle;
        }

        for (auto &iter_frontier : frontiers_vec)
        {
            iter_frontier.cost = potential_scale_ * (iter_frontier.min_distance / max_min_distance) - gain_scale_ * (iter_frontier.size / max_size);
        }
    }
} // end of namespace