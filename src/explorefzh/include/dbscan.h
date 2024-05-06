#pragma once

#include <vector>

#include <cmath>
#include <set>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "../include/search_frontier.hpp"

using namespace std;

namespace dbscan_ns
{
    struct EvaluationStruct
    {
        double value;
        double cost_bar; // cost 平均值
        int number;      // cluster内点的个数  如果是噪音点 则n=1；
        int index;       // 簇内核心边界下标
    };
    class DbscanClass
    {
    private:
        int minPts_;
        double epsilon_;
        double CalculateDistance(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
        {
            return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
        }
        vector<set<int>> getPointSet(const vector<geometry_msgs::Point> points_vec);
        vector<set<int>> getCluster(vector<set<int>> points_set_vec);

    public:
        DbscanClass() = delete;
        DbscanClass(ros::NodeHandle &param_nh);
        frontier_exploration_ns::FrontierStruct getOptimalFrontier(vector<frontier_exploration_ns::FrontierStruct> fontier_vec, double alpha, double beta, vector<set<int>> &result_clusters, set<int> &result_noise);
        double get_epsilon() const { return epsilon_; }
        int get_minPts() const { return minPts_; }
        void DispCluster(vector<set<int>> cluster);
        ~DbscanClass(){};
    };
}