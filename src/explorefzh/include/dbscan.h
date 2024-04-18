#pragma once

#include <vector>

#include <cmath>
#include <set>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

using namespace std;

namespace dbscan_ns
{
    class DbscanClass
    {
    private:
        unsigned int minPts;
        double epsilon;
        double distance(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
        {
            return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
        }

    public:
        DbscanClass()=delete;
        DbscanClass(ros::NodeHandle &param_nh);
        vector<set<int>> getPointSet(const vector<geometry_msgs::Point> points_vec);
        vector<set<int>> getCluster(vector<set<int>> points_set_vec);
    };
}