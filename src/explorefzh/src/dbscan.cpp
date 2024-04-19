/**
 * @file dbscan.cpp
 * @brief dbscan算法 注意 set 容器中的元素是唯一的
 *          核心点，边界点和噪声点。邻域半径R内样本点的数量大于等于minpoints的点叫做核心点。
 *          不属于核心点但在某个核心点的邻域内的点叫做边界点。既不是核心点也不是边界点的是噪声点。
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
        param_nh.param("dbscan_minPts", minPts_, 3.0);
        param_nh.param("dbscan_epsilon", epsilon, 2.99);
        minPts = int(minPts_);
    }

    void DbscanClass::dispCluster(vector<set<int>> cluster)
    {
        string temp_cout = "Cluster:";
        for (auto s_iter : cluster)
        {
            temp_cout += "{";
            int count = 0;
            for (auto iter : s_iter)
            {
                temp_cout += to_string(iter);
                if (++count < s_iter.size())
                {
                    temp_cout += ",";
                }
            }
            temp_cout += "}  ";
        }
        ROS_WARN_STREAM(temp_cout);
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
     * @brief 只能由核心点拓展，非核心点只能被囊括在内，而不由它开始拓展 核心算法
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

    frontier_exploration_ns::FrontierStruct DbscanClass::getOptimalFrontier(vector<frontier_exploration_ns::FrontierStruct> frontier_vec, double alpha, double beta)
    {

        // string temp_cout = "Points:";
        ROS_WARN("### DBSCAN start!!! ###");
        
        vector<EvaluationStruct> eval_vec;
        vector<geometry_msgs::Point> centroid_vec;

        vector<set<int>> point_set_vec; // 实际上是下标 初始集合
        vector<set<int>> cluster_vec;   // 点簇集合

        set<int> all_set;       // 所有点下标
        set<int> noise_set;     // 噪声点下标集合
        set<int> not_noise_set; // 非噪音下标点集

        for (size_t i = 0; i < frontier_vec.size(); i++)
        {
            centroid_vec.push_back(frontier_vec[i].centroid); // 质心点集
            all_set.insert(i);
        }
        point_set_vec = getPointSet(centroid_vec);
        cluster_vec = getCluster(point_set_vec);
        // dispCluster(cluster_vec);  // 显示分类集合

        int size_max = 1;                        // 需要先遍历所有簇后才能得到最大规模数 然后才能计算评价值
        for (const auto &set_iter : cluster_vec) // 遍历所有簇
        {
            if (set_iter.size() > size_max) // 寻找所有簇的最大规模
                size_max = set_iter.size();

            EvaluationStruct eval;
            double temp_min_cost = frontier_vec[*(set_iter.begin())].cost; // 暂时以首元素作为最小代价cost
            eval.index = *(set_iter.begin());

            for (auto int_iter : set_iter) // 遍历单个簇内元素
            {
                not_noise_set.insert(int_iter); // 生成非噪声点下标集合

                if (frontier_vec[int_iter].cost < temp_min_cost) // 在簇中寻找最小代价边界点
                {
                    temp_min_cost = frontier_vec[int_iter].cost;
                    eval.index = int_iter;
                }

                eval.cost_bar += frontier_vec[int_iter].cost; // 累加
            }

            eval.number = set_iter.size(); // 簇的规模
            eval.cost_bar /= eval.number;  // 得到簇的代价平均值

            eval_vec.push_back(eval);
        }

        std::set_difference(all_set.begin(), all_set.end(), not_noise_set.begin(), not_noise_set.end(), std::inserter(noise_set, noise_set.begin())); // 取差集即可得到所有噪音点
        // 得到集合 簇容器cluster和 noise_set后
        for (auto int_iter : noise_set)
        {
            EvaluationStruct eval;
            eval.number = 1;
            eval.cost_bar = frontier_vec[int_iter].cost;
            eval.index = int_iter;

            eval_vec.push_back(eval);
        }

        int optimal_index = 0;
        double min_value = std::numeric_limits<double>::max();
        for (auto &eval : eval_vec)
        {
            eval.value = alpha * eval.cost_bar - beta * eval.number / size_max;
            if (eval.value < min_value)
            {
                min_value = eval.value;
                optimal_index = eval.index;
            }
        }

        return frontier_vec[optimal_index];
    }

}

/*
 temp_cout += "(";
            temp_cout += to_string(frontier_vec[i].centroid.x);
            temp_cout += ",";
            temp_cout += to_string(frontier_vec[i].centroid.y);
            temp_cout += ")  ";
        }
        ROS_WARN_STREAM(temp_cout);
*/