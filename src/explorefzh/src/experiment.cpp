/**
 * @file explore_data.cpp
 * @brief
 *      display trajectory and publish experiment data
 * @author ZhiHang_FU (fu_zhihang@126.com)
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <thread>
#include <mutex>

#include "explorefzh/explore_data.h"
class ExperimentClass
{
private:
    ros::NodeHandle nh_;
    ros::Publisher data_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber map_sub_;

    explorefzh::explore_data data_msg_;
    std::mutex data_mutex_; // 互斥量  “上锁”
    nav_msgs::Path path_msg_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool running_;
    double data_record_frequency_;
    std::thread path_pub_thread_;
    std::thread data_pub_thread_;

public:
    /**
     * @brief Construct a new Experiment Class object
     */
    ExperimentClass() : nh_("~"), tf_listener_(tf_buffer_), running_(true)
    {
        // 话题命名空间 加上“/”才代表全局名称，否则是私有名称
        path_pub_ = nh_.advertise<nav_msgs::Path>("/robot_path", 10);
        data_pub_ = nh_.advertise<explorefzh::explore_data>("/explore_data", 10);
        map_sub_ = nh_.subscribe("/map", 20, &ExperimentClass::mapCallback, this);

        nh_.param("data_pub_frequency", data_record_frequency_, 2.0); // 发布data频率

        // 多线程 单独开两个线程
        path_pub_thread_ = std::thread(&ExperimentClass::PathPub, this);
        data_pub_thread_ = std::thread(&ExperimentClass::DataPub, this);

        ros::spin();
    }
    /**
     * @brief Destroy the Experiment Class object
     */
    ~ExperimentClass()
    {
        running_ = false;
        if (path_pub_thread_.joinable())
            path_pub_thread_.join();
        if (data_pub_thread_.joinable())
            data_pub_thread_.join();
        ros::shutdown();
    }
    /**
     * @brief 发布robot所经过的路径 rviz  显示轨迹
     */
    void PathPub()
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        path_msg_.header.frame_id = "map";

        double last_position_x = 0, last_position_y = 0, last_position_z = 0;
        ros::Rate loop_rate(5.0);
        while (ros::ok() && running_)
        {
            geometry_msgs::TransformStamped transfrom_stamped;
            try
            {
                transfrom_stamped = tf_buffer_.lookupTransform("map", "base_footprint", ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_ERROR("TF Exception: %s", ex.what());
                continue;
            }
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose.position.x = transfrom_stamped.transform.translation.x;
            pose_stamped.pose.position.y = transfrom_stamped.transform.translation.y;
            pose_stamped.pose.position.z = transfrom_stamped.transform.translation.z;
            pose_stamped.pose.orientation = transfrom_stamped.transform.rotation;

            path_msg_.poses.push_back(pose_stamped);
            path_pub_.publish(path_msg_);

            double dx = pose_stamped.pose.position.x - last_position_x;
            double dy = pose_stamped.pose.position.y - last_position_y;
            double dz = pose_stamped.pose.position.z - last_position_z;
            double delta_distance = sqrt(dx * dx + dy * dy + dz * dz);

            last_position_x = pose_stamped.pose.position.x;
            last_position_y = pose_stamped.pose.position.y;
            last_position_z = pose_stamped.pose.position.z;

            data_mutex_.lock();
            data_msg_.distance += delta_distance;
            data_mutex_.unlock();

            loop_rate.sleep();
        }
    }
    /**
     * @brief 发布数据 data_msg_={时间time 路程distance 已知栅格数量area}
     */
    void DataPub()
    {
        ros::Rate loop_rate(data_record_frequency_);

        while (ros::ok() && running_)
        {
            data_mutex_.lock();
            data_msg_.time = ros::Time::now().toSec();
            data_pub_.publish(data_msg_);
            data_mutex_.unlock();

            loop_rate.sleep();
        }
    }
    /**
     * @brief 回调函数 统计已知栅格地图数据
     * @param  msg              My Param doc
     */
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        int occupied_cells_count = 0;
        int free_cells_count = 0;
        for (size_t i = 0; i < msg->data.size(); i++)
        {
            if (msg->data[i] == 100)
                occupied_cells_count++;
            else if (msg->data[i] == 0)
                free_cells_count++;
        }
        data_mutex_.lock();
        data_msg_.area = free_cells_count + occupied_cells_count;
        data_mutex_.unlock();
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "experiment_node");
    ExperimentClass experiment;

    return 0;
}
