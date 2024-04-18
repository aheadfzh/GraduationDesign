/**
 * @file dbscan.cpp
 * @brief dbscan算法 注意 set 容器中的元素是唯一的
 *
 * @author ZhiHang_FU (fu_zhihang@126.com)
 * @version 1.0
 * @date 2024-04-18
 * @copyright Copyright (c) 2024  the author of this program
 */
#include "../include/dbscan.h"

namespace dbscan_ns
{

    DbscanClass::DbscanClass(ros::NodeHandle &param_nh)
    {
        double minPts_;
        param_nh.param("dbscan_minPts", minPts_, 2.0);
        param_nh.param("dbscan_epsilon", epsilon, 5.0);
        minPts = int(minPts_);


    }

    // 获得初始集合  这里就可以得到噪音点
    vector<set<int>> DbscanClass::getPointSet(const vector<geometry_msgs::Point> points_vec)
    {
        vector<set<int>> PointSet(points_vec.size());
        for (size_t i = 0; i < points_vec.size(); i++)
        {
            for (size_t j = 0; j < points_vec.size(); j++)
            {
                if (distance(points_vec[i], points_vec[j]) <= epsilon)
                    PointSet[i].insert(j);
            }
        }
        return PointSet;
    }

    /**
     * @brief 只能由核心点拓展，非核心点只能被囊括在内，而不由它开始拓展
     * @param  points_set_vec   My Param doc
     * @param  minPts           My Param doc
     * @return vector<set<int>>  是Point容器的元素下标 组成的集合
     */
    vector<set<int>> DbscanClass::getCluster(vector<set<int>> points_set_vec)
    {
        vector<bool> visted_flag(points_set_vec.size(), false);
        vector<set<int>> cluster;

        for (size_t i = 0; i < points_set_vec.size(); i++) // 遍历每个集合
        {
            if (!visted_flag[i] && points_set_vec[i].size() >= minPts) // 如果这个集合没有遍历过 集合的元素大于minPts （核心点集合）
            {
                // 遍历第i个集合中的每个元素
                for (set<int>::const_iterator j = points_set_vec[i].begin(); j != points_set_vec[i].end(); j++)
                {
                    visted_flag[*j] = true;
                    if (points_set_vec[*j].size() >= minPts) // 以第*j元素为圆心的邻域集合内有大于minPts个点 （核心点集合）
                        for (auto k = points_set_vec[*j].begin(); k != points_set_vec[*j].end(); k++)
                        {
                            points_set_vec[i].insert(*k); // 间接连接的点也加入该集合
                            visted_flag[*k] = true;       // 并且标记这个点 ，以这个点为圆心的集合将不再被访问
                        }
                }
                cluster.push_back(points_set_vec[i]);
            }
        }

        return cluster;
    }

}