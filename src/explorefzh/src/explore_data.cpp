#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>

// #include <costmap_2d/costmap_2d.h>
// #include <costmap_2d/cost_values.h>
// #include "../include/costmap_client.h"

#include "explorefzh/explore_data.h"

class ExploreDataClass
{
private:
    ros::NodeHandle nh;
    ros::Publisher pathPub;
    ros::Publisher dataPub;
    ros::Subscriber mapSub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    double totalTrajectoryLength;

    explorefzh::explore_data data_msg;

    nav_msgs::Path pathMsg;

public:
    ExploreDataClass() : nh("~"), totalTrajectoryLength(0)
    {
        pathPub = nh.advertise<nav_msgs::Path>("robot_path", 10);
        dataPub = nh.advertise<explorefzh::explore_data>("explore_data", 10);
        // mapSub = nh.subscribe("map", 10, &ExploreDataClass::mapCallback, this);

        tfListener = new tf2_ros::TransformListener(tfBuffer);

        data_msg.area = 0;
        data_msg.distance = 0;
        data_msg.time = 0;
    }
    void pubExploreData()
    {

        unsigned int temp = 0;

        ros::Rate rate(4.0);

        pathMsg.header.frame_id = "map";
        double last_x = 0, last_y = 0, last_z = 0;

        while (ros::ok())
        {
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("TF Exception: %s", ex.what());
                continue;
            }

            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.frame_id = "map";
            poseStamped.header.stamp = ros::Time::now();
            poseStamped.pose.position.x = transformStamped.transform.translation.x;
            poseStamped.pose.position.y = transformStamped.transform.translation.y;
            poseStamped.pose.position.z = transformStamped.transform.translation.z;
            poseStamped.pose.orientation = transformStamped.transform.rotation;

            pathMsg.poses.push_back(poseStamped);
            pathPub.publish(pathMsg);

            // Calculate segment length and update total length
            if (pathMsg.poses.size() > 1)
            {
                double dx = poseStamped.pose.position.x - last_x;
                double dy = poseStamped.pose.position.y - last_y;
                double dz = poseStamped.pose.position.z - last_z;

                double segmentLength = std::sqrt(dx * dx + dy * dy + dz * dz);
                totalTrajectoryLength += segmentLength;
                data_msg.distance = totalTrajectoryLength;
            }
            last_x = poseStamped.pose.position.x;
            last_y = poseStamped.pose.position.y;
            last_z = poseStamped.pose.position.z;

            // dataPub.publish(data_msg);

            ++temp %= 8;
            if (temp == 1)
            {
                ROS_ERROR("$$##  Distance: %f Meter ##$$", totalTrajectoryLength);
            }

            rate.sleep();
        }
    }
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        // int occupiedCount = 0;
        // int freeCount = 0;

        // // 遍历地图中的每个栅格
        // for (size_t i = 0; i < msg->data.size(); ++i)
        // {
        //     if (msg->data[i] == 100) // 100 表示占据栅格
        //         occupiedCount++;
        //     else if (msg->data[i] == 0) // 0 表示空闲栅格
        //         freeCount++;
        // }

        // ROS_ERROR("Occupied grids: %d, Free grids: %d", occupiedCount, freeCount);
        // data_msg.area = freeCount + occupiedCount;
        // dataPub.publish(data_msg);

        // ros::Rate rate(2); // 设置每秒执行次回调函数
        // rate.sleep();      // 等待一段时间，控制回调函数的执行频率
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_publisher");
    ExploreDataClass exploreData;
    exploreData.pubExploreData();

    return 0;
}
