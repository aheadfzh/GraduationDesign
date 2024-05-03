// #include <ros/ros.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// int main(int argc, char *argv[])
// {
//     /* code */
//     ros::init(argc, argv, "set_navigation_goal_node");

//     MoveBaseClient mb_client("move_base", true);

//     while (!mb_client.waitForServer(ros::Duration(5.0)))
//     {
//         ROS_INFO("Waiting for the move_base action server to come up");

//         /* code */
//     }

//     move_base_msgs::MoveBaseGoal goal;

//     goal.target_pose.header.frame_id = "map";
//     goal.target_pose.header.stamp = ros::Time::now();

//     goal.target_pose.pose.position.x = 2.0;
//     goal.target_pose.pose.orientation.w = 2.0;

//     ROS_INFO("Sending goal");

//     mb_client.sendGoal(goal);

//     mb_client.waitForResult();

//     if (mb_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//         ROS_INFO("Hooray,the base moved %f meter forward",goal.target_pose.pose.position.x);
//     else
//         ROS_INFO("The base failed to move forward 1 meter for some reason");

//     return 0;
// }
/*****************************************//**/
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    ROS_WARN("successfully subscriber map!!!");
    int occupiedCount = 0;
    int freeCount = 0;

    // 遍历地图中的每个栅格
    for (size_t i = 0; i < msg->data.size(); ++i)
    {
        if (msg->data[i] == 100) // 100 表示占据栅格
            occupiedCount++;
        else if (msg->data[i] == 0) // 0 表示空闲栅格
            freeCount++;
    }
    // 在这里处理接收到的map消息
    ROS_ERROR("map size %d ", freeCount + occupiedCount);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_subscriber");
    ros::NodeHandle nh("~map");

    ros::Subscriber sub = nh.subscribe("map", 10, mapCallback);

    ros::spin();

    return 0;
}
/*************************/