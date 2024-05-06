#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <string>
#include <map>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_client.h"
#include "search_frontier.hpp"
#include "dbscan.h"

namespace explore_ns
{
    class ExploreClass
    {
    public:
        ExploreClass();
        ~ExploreClass();
        void start();
        void stop();

    private:
        void MakePlan();
        void VisualizeFrontiers(const std::vector<frontier_exploration_ns::FrontierStruct> &frontiers, bool viusal_flag);
        void VisualizeFrontiers(const std::vector<frontier_exploration_ns::FrontierStruct> &frontiers, bool viusal_flag, vector<set<int>> &clusters, set<int> &noise);
        std::map<std::string, std_msgs::ColorRGBA> generate4BaseColorMap();
        std_msgs::ColorRGBA generate20DiffColors(int n);

        void ReachedGoal(const actionlib::SimpleClientGoalState &status,
                         const move_base_msgs::MoveBaseResultConstPtr &result,
                         const geometry_msgs::Point &frontier_goal);
        bool IsGoalOnBlackList(const geometry_msgs::Point &goal);
        std::vector<frontier_exploration_ns::FrontierStruct> OutGoalOnBlackList(const std::vector<frontier_exploration_ns::FrontierStruct> &frontier_vec);

        ros::NodeHandle private_nh_;
        ros::NodeHandle relative_nh_;
        ros::Publisher marker_array_pub_;
        tf::TransformListener tf_listener_;

        Costmap2DClient costmap_client_;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

        frontier_exploration_ns::FrontierSearchClass search_;
        ros::Timer exploring_timer_;
        ros::Timer oneshot_;

        std::vector<geometry_msgs::Point> frontier_blacklist_;
        geometry_msgs::Point prev_goal_;
        double prev_distance_; // 上一次机器人和目标点的实际距离
        ros::Time last_progress_;
        size_t last_markers_count_;

        double planner_frequency_;
        double potential_scale_, orientation_scale_, gain_scale_;
        ros::Duration progress_timeout_; // 程序超时时间
        bool visualize_;

        dbscan_ns::DbscanClass dbscan_;
        double alpha_;
        double beta_;
    };

} // end of explore_ns