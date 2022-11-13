#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <f1tenth_simulator/scan_range.h>
#include <std_msgs/Float64.h>
#include <stdio.h>

// void point1Callback(const std_msgs::Float64 point)
// {
//     float min = point.data;

//     ROS_INFO("min: [%lf]", min);
// }

// void point2Callback(const std_msgs::Float64 point)
// {
//     float max = point.data;

//     ROS_INFO("max: [%lf]", max);
// }

void point1Callback(const f1tenth_simulator::scan_range point)
{
    float min = point.scan_range;

    // ROS_INFO("min: [%lf]", min);
}

void point2Callback(const f1tenth_simulator::scan_range point)
{
    float max = point.scan_range;

    // ROS_INFO("max: [%lf]", max);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point");

    ros::NodeHandle n;
    n = ros::NodeHandle("~");

    // std::string point1_topic, point2_topic;
    // n.getParam("point1_topic", point1_topic);
    // n.getParam("point2_topic", point2_topic);

    ros::Subscriber point1_sub = n.subscribe("/farthest_point", 1, point1Callback);
    ros::Subscriber point2_sub = n.subscribe("/closet_point", 1, point2Callback);

    ros::spin();

    return 0;
}