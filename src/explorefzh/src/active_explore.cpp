/**
 * @file active_explore.cpp
 * @brief
 *
 * @author ZhiHang_FU (fu_zhihang@126.com)
 * @version 1.0
 * @date 2024-04-04
 * @copyright Copyright (c) 2024  the author of this program
 */
#include "../include/active_explore.hpp"
#include <thread>

/**
 * @brief 运算符重载
 */
inline static bool operator==(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2)) < 0.01;
}

namespace explore_ns
{
    ExploreClass::ExploreClass() : private_nh_("~"),
                                   tf_listener_(ros::Duration(10.0)), // 设置初始化超时时间 10s内未获取tf变换消息则认为操作超时
                                   costmap_client_(private_nh_, relative_nh_, &tf_listener_),
                                   move_base_client_("move_base"),
                                   prev_distance_(0.0),
                                   last_markers_count_(0),
                                   dbscan_(private_nh_) // DBSCAN算法
    {
        double timeout;
        double min_frontier_size;
        private_nh_.param("planner_frequency", planner_frequency_, 1.0);
        private_nh_.param("progress_timeout", timeout, 30.0);
        progress_timeout_ = ros::Duration(timeout);
        private_nh_.param("visualize", visualize_, false);
        private_nh_.param("potential_scale", potential_scale_, 1e-3);
        private_nh_.param("orientation_scale", orientation_scale_, 0.0);
        private_nh_.param("gain_scale", gain_scale_, 1.0);
        private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
        private_nh_.param("alpha", alpha_, 0.5);
        private_nh_.param("beta", beta_, 0.5);

        search_ = frontier_exploration_ns::FrontierSearchClass(costmap_client_.getCostmap(),
                                                               potential_scale_,
                                                               gain_scale_,
                                                               orientation_scale_,
                                                               min_frontier_size);
        if (visualize_)
            marker_array_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);

        ROS_DEBUG("Waiting to connect to move_base server");
        move_base_client_.waitForServer();
        ROS_DEBUG("Connected to move_base server successfully");

        exploring_timer_ = relative_nh_.createTimer(ros::Duration(1.0 / planner_frequency_),
                                                    [this](const ros::TimerEvent &)
                                                    { MakePlan(); });
    }

    /**
     * @brief Destroy the Explore Class:: Explore Class object
     */
    ExploreClass::~ExploreClass() { stop(); }

    /**
     * @brief 决定计划前往的目标点
     */
    void ExploreClass::MakePlan()
    {
        auto pose = costmap_client_.getRobotPose();
        auto frontiers_vec = search_.SearchFrom(pose); // 从当前robot所在位置开始寻找

        if (frontiers_vec.empty())
        {
            ROS_DEBUG("All frontiers have been explored");
            VisualizeFrontiers(frontiers_vec, visualize_); // 含有清除功能
            stop();
            return;
        }

        auto iter_frontier = std::find_if_not(frontiers_vec.begin(), // 查找不符合条件的元素；返回值是第一个符合查找规则的元素
                                              frontiers_vec.end(),
                                              [this](const frontier_exploration_ns::FrontierStruct &f)
                                              { return IsGoalOnBlackList(f.centroid); });
        auto optimal_frontier = *iter_frontier; // 默认的

        if (frontiers_vec.size() >= 5)
        {
            vector<set<int>> clusters;
            set<int> noises;
            std::vector<frontier_exploration_ns::FrontierStruct> frontier_vec_clear = OutGoalOnBlackList(frontiers_vec);
            optimal_frontier = dbscan_.getOptimalFrontier(frontier_vec_clear, alpha_, beta_, clusters, noises);
            VisualizeFrontiers(frontiers_vec, visualize_, clusters, noises);
        }
        else
            VisualizeFrontiers(frontiers_vec, visualize_);

        if (iter_frontier == frontiers_vec.end()) // 代表没有了边界点或者里面全是黑名单的边界点
        {
            ROS_WARN("Only frontiers on black list have not been explored");
            stop();
            return;
        }

        geometry_msgs::Point target_position = optimal_frontier.centroid;
        bool same_goal = prev_goal_ == target_position; // 判断是否相同
        prev_goal_ = target_position;

        if (!same_goal || prev_distance_ > optimal_frontier.min_distance)
        {
            last_progress_ = ros::Time::now();
            prev_distance_ = optimal_frontier.min_distance;
        }
        if (ros::Time::now() - last_progress_ > progress_timeout_)
        {
            frontier_blacklist_.push_back(target_position);
            ROS_WARN("Adding current goal (%f, %f) to black list", target_position.x, target_position.y);
            MakePlan(); // 回调执行
            return;
        }
        if (same_goal)
        {
            ROS_DEBUG("same_goal");
            return;
        }

        // 给move_base 发送目标点
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position = target_position;
        goal.target_pose.pose.orientation.w = 1;
        goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
        goal.target_pose.header.stamp = ros::Time::now();
        move_base_client_.sendGoal(goal,
                                   [this, target_position](const actionlib::SimpleClientGoalState &status,
                                                           const move_base_msgs::MoveBaseResultConstPtr &result)
                                   {
                                       ReachedGoal(status, result, target_position);
                                   });
    }

    /**
     * @brief 可视化边界
     */
    void ExploreClass::VisualizeFrontiers(const std::vector<frontier_exploration_ns::FrontierStruct> &frontiers_vec, bool viusal_flag)
    {
        if (!viusal_flag)
            return;
        auto color_map = generate4BaseColorMap();

        ROS_DEBUG("visualising %lu frontiers", frontiers_vec.size());
        visualization_msgs::MarkerArray marker_array;
        std::vector<visualization_msgs::Marker> &markers_vec = marker_array.markers;
        visualization_msgs::Marker marker;

        marker.header.frame_id = costmap_client_.getGlobalFrameID();
        marker.header.stamp = ros::Time::now();
        marker.ns = "frontiers";
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color = color_map["blue"];
        marker.lifetime = ros::Duration(0); // 设置可视化标记的生命周期为0秒，即使不持续显示，也不会自动消失
        marker.frame_locked = true;

        double min_cost = frontiers_vec.empty() ? 0.0 : frontiers_vec.front().cost;
        marker.action = visualization_msgs::Marker::ADD;
        size_t id = 0;
        for (auto &frontier : frontiers_vec)
        {
            marker.type = visualization_msgs::Marker::POINTS;
            marker.id = int(id);
            marker.pose.position = {};
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.points = frontier.points_vec;

            bool temp_color_flag = false;
            if (IsGoalOnBlackList(frontier.centroid))
            {
                marker.color = color_map["red"];
                temp_color_flag = true;
            }
            else
                marker.color = color_map["blue"];
            markers_vec.push_back(marker);
            ++id;

            marker.type = visualization_msgs::Marker::CYLINDER; // 圆柱体
            marker.id = int(id);
            marker.pose.position = frontier.centroid;
            double scale = std::max(std::min(std::abs(min_cost * 0.45 / frontier.cost), 0.6), 0.2);
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;
            marker.points = {};
            if (!temp_color_flag)
                marker.color = color_map["yellow"];
            else
                marker.color = color_map["red"];
            markers_vec.push_back(marker);
            ++id;
        }
        size_t current_markers_count = markers_vec.size();
        // 删除之前不再使用的标记
        marker.action = visualization_msgs::Marker::DELETE;
        for (; id < last_markers_count_; ++id)
        {
            marker.id = int(id);
            markers_vec.push_back(marker);
        }
        last_markers_count_ = current_markers_count;
        marker_array_pub_.publish(marker_array);
    }

    void ExploreClass::VisualizeFrontiers(const std::vector<frontier_exploration_ns::FrontierStruct> &frontiers_vec, bool viusal_flag, vector<set<int>> &clusters, set<int> &noise)
    {
        if (!viusal_flag)
            return;
        auto color_map = generate4BaseColorMap();
        ROS_DEBUG("----visualising %lu frontiers----", frontiers_vec.size());
        visualization_msgs::MarkerArray marker_array;
        std::vector<visualization_msgs::Marker> &markers_vec = marker_array.markers;
        visualization_msgs::Marker marker;

        marker.header.frame_id = costmap_client_.getGlobalFrameID();
        marker.header.stamp = ros::Time::now();
        marker.ns = "frontiers";
        marker.lifetime = ros::Duration(0); // 设置可视化标记的生命周期为0秒，即使不持续显示，也不会自动消失
        marker.frame_locked = true;

        double min_cost = frontiers_vec.empty() ? 0.0 : frontiers_vec.front().cost;
        marker.action = visualization_msgs::Marker::ADD;
        size_t id = 0;
        for (auto &index : noise)
        {
            marker.type = visualization_msgs::Marker::POINTS;
            marker.id = int(id);
            marker.pose.position = {};
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.points = frontiers_vec[index].points_vec;
            bool temp_color_flag = false;
            if (IsGoalOnBlackList(frontiers_vec[index].centroid))
            {
                marker.color = color_map["red"];
                temp_color_flag = true;
            }
            else
                marker.color = color_map["blue"];
            markers_vec.push_back(marker);
            ++id;

            marker.type = visualization_msgs::Marker::CYLINDER; // 圆柱体
            marker.id = int(id);
            marker.pose.position = frontiers_vec[index].centroid;
            double scale = std::max(std::min(std::abs(min_cost * 0.45 / frontiers_vec[index].cost), 0.6), 0.2);
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;
            marker.points = {};
            if (!temp_color_flag)
                marker.color = color_map["yellow"]; // 噪声点全是黄色
            else
                marker.color = color_map["red"];
            markers_vec.push_back(marker);
            ++id;

        } // end of noise

        int temp_color_n = 0;
        for (auto &set_iter : clusters)
        {
            temp_color_n += 2;
            for (auto &index : set_iter)
            {
                marker.type = visualization_msgs::Marker::POINTS;
                marker.id = int(id);
                marker.pose.position = {};
                marker.scale.x = 0.07;
                marker.scale.y = 0.07;
                marker.scale.z = 0.07;
                marker.points = frontiers_vec[index].points_vec;
                bool temp_color_flag = false;
                if (IsGoalOnBlackList(frontiers_vec[index].centroid))
                {
                    marker.color = color_map["red"];
                    temp_color_flag = true;
                }
                else
                    marker.color = generate20DiffColors(temp_color_n);
                markers_vec.push_back(marker);
                ++id;

                marker.type = visualization_msgs::Marker::CYLINDER; // 圆柱体
                marker.id = int(id);
                marker.pose.position = frontiers_vec[index].centroid;
                double scale = std::max(std::min(std::abs(min_cost * 0.45 / frontiers_vec[index].cost), 0.6), 0.2);
                marker.scale.x = scale;
                marker.scale.y = scale;
                marker.scale.z = scale;
                marker.points = {};
                if (!temp_color_flag)
                    marker.color = generate20DiffColors(temp_color_n + 1);
                else
                    marker.color = color_map["red"];
                markers_vec.push_back(marker);
                ++id;
            }
        } //  end of cluaster

        size_t current_markers_count = markers_vec.size();
        // 删除之前不再使用的标记
        marker.action = visualization_msgs::Marker::DELETE;
        for (; id < last_markers_count_; ++id)
        {
            marker.id = int(id);
            markers_vec.push_back(marker);
        }
        last_markers_count_ = current_markers_count;
        marker_array_pub_.publish(marker_array);
    }

    // 生成四种基本不同颜色map
    std::map<std::string, std_msgs::ColorRGBA> ExploreClass::generate4BaseColorMap()
    {
        std::map<std::string, std_msgs::ColorRGBA> colors_map = {};
        std_msgs::ColorRGBA blue;
        blue.r = 0;
        blue.g = 0;
        blue.b = 1.0;
        blue.a = 1.0;
        colors_map["blue"] = blue;
        colors_map["b"] = blue;
        std_msgs::ColorRGBA red;
        red.r = 1.0;
        red.g = 0;
        red.b = 0;
        red.a = 1.0;
        colors_map["red"] = red;
        colors_map["r"] = red;
        std_msgs::ColorRGBA green;
        green.r = 0;
        green.g = 1.0;
        green.b = 0.0;
        green.a = 1.0;
        colors_map["green"] = green;
        colors_map["g"] = green;
        std_msgs::ColorRGBA yellow;
        yellow.r = 1.0;
        yellow.g = 1.0;
        yellow.b = 0.0;
        yellow.a = 1.0;
        colors_map["yellow"] = yellow;
        colors_map["y"] = yellow;
        return colors_map;
    }

    std_msgs::ColorRGBA ExploreClass::generate20DiffColors(int n)
    {
        n = abs(n % 20);
        std_msgs::ColorRGBA color = (generate4BaseColorMap())["red"];
        int baseRed = color.r * 255, baseGreen = color.g * 255, baseBlue = color.b * 255, step = 28;

        // 根据n的值调整RGB分量
        if ((n % 2) == 0)
        {
            color.r = static_cast<float>((baseRed + n * step) % 255) / 255.0;
            color.g = static_cast<float>((baseGreen + n * step) % 255) / 255.0;
            color.b = static_cast<float>((baseBlue + n * step / 2) % 255) / 255.0;
        }
        else
        {
            color.r = static_cast<float>((baseRed + n * step) % 255) / 255.0;
            color.g = static_cast<float>((baseGreen + n * step / 2) % 255) / 255.0;
            color.b = static_cast<float>((baseBlue + n * step) % 255) / 255.0;
        }
        return color;
    }

    /**
     * @brief 判断goal是否在黑名单中
     */
    bool ExploreClass::IsGoalOnBlackList(const geometry_msgs::Point &goal)
    {
        constexpr static size_t tolerace = 5; // 容忍度
        costmap_2d::Costmap2D *costmap_2d = costmap_client_.getCostmap();

        for (auto &iter_frontier : frontier_blacklist_) // 所有都遍历一遍
        {
            double x_delta = fabs(goal.x - iter_frontier.x);
            double y_delta = fabs(goal.y - iter_frontier.y);

            if (x_delta < tolerace * costmap_2d->getResolution() &&
                y_delta < tolerace * costmap_2d->getResolution())
                return true;
        }
        return false;
    }

    /**
     * @brief 得到不含黑名单边界的所有边界
     */
    std::vector<frontier_exploration_ns::FrontierStruct> ExploreClass::OutGoalOnBlackList(const std::vector<frontier_exploration_ns::FrontierStruct> &frontier_vec)
    {
        std::vector<frontier_exploration_ns::FrontierStruct> result;
        for (const auto &frontier_iter : frontier_vec)
        {
            if (!IsGoalOnBlackList(frontier_iter.centroid))
                result.push_back(frontier_iter);
        }
        return result;
    }

    /**
     * @brief 到达目标点的回调函数 用于处理机器人是否已经达到目标的情况
     * 如果机器人未能成功到达目标，则将当前计划的前沿目标添加到黑名单中，并触发重新规划。
     * @param  status
     * @param  frontier_goal
     */
    void ExploreClass::ReachedGoal(const actionlib::SimpleClientGoalState &status,
                                   const move_base_msgs::MoveBaseResultConstPtr &,
                                   const geometry_msgs::Point &frontier_goal)
    {
        ROS_WARN("---reached goal with status: %s ----", status.toString().c_str());
        // 检查机器人达到目标的状态是否为 ABORTED，如果是，表示机器人未能成功到达目标。
        if (status == actionlib::SimpleClientGoalState::ABORTED)
        {
            frontier_blacklist_.push_back(frontier_goal);
            ROS_WARN("---adding current goal to black list----");
        }
        // find new goal immediatelly regardless of planning frequency.
        // 不论计划的频率如何，立即找到新的目标
        // execute via timer to prevent dead lock in move_base_client (this is
        // callback for sendGoal, which is called in makePlan). the timer must live
        // until callback is executed.
        // 创建一个一次性定时器 (oneshot_)，在定时器触发时调用 makePlan() 函数。
        // 这是为了确保在 sendGoal 的回调中调用 makePlan() 不会导致死锁。
        // 定时器被设置为一次性的，因此它会在执行一次 makePlan() 后自动销毁。
        oneshot_ = relative_nh_.createTimer(ros::Duration(0, 0), [this](const ros::TimerEvent &)
                                            { MakePlan(); }, true);
    }

    void ExploreClass::start()
    {
        exploring_timer_.start();
    }
    void ExploreClass::stop()
    {

        ROS_WARN("### Exploration Stopped ### Cancel All Goals ### ");

        move_base_client_.cancelAllGoals();
        exploring_timer_.stop();
    }
}

int main(int argc, char *argv[])
{

    /* code */
    ros::init(argc, argv, "explorefzh_node");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    explore_ns::ExploreClass explore_fzh;

    ros::spin();
    return 0;
}
