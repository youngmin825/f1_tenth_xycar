#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <f1tenth_simulator/scan_range.h>
#include <stdio.h>

ros::Publisher closet;
ros::Publisher farthest;

void lidarCallback(const sensor_msgs::LaserScan laser)
{
    float min = 100;
    float max = 1;

    // ROS_INFO("%d, %d", laser.angle_max, laser.angle_min);

    for (int i = 0; i < laser.ranges.size(); i++)
    {
        if(laser.ranges[i] < min )
        {
            min = laser.ranges[i];
        }

        if(laser.ranges[i] > max)
        {
            max = laser.ranges[i];
        }
    }
    // printf("%c\n", '2');
    // std_msgs::Float64 msg_closet;
    // std_msgs::Float64 msg_farthest;
    f1tenth_simulator::scan_range msg_closet;
    f1tenth_simulator::scan_range msg_farthest;

    // msg_closet.data = min;
    // msg_farthest.data = max;
    msg_closet.scan_range = min;
    msg_farthest.scan_range = max;

    closet.publish(msg_closet);
    farthest.publish(msg_farthest);
    // printf("%f",msg_closet.data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar");

    ros::NodeHandle n;


    n = ros::NodeHandle("~");

    // std::string scan_topic;
    // n.getParam("scan_topic", scan_topic);
    // printf("%c\n", '1');
    
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, lidarCallback);
    // closet = n.advertise<std_msgs::Float64>("/closet_point", 1000);
    // farthest = n.advertise<std_msgs::Float64>("/farthest_point", 1000);
    closet = n.advertise<f1tenth_simulator::scan_range>("/closet_point", 1000);
    farthest = n.advertise<f1tenth_simulator::scan_range>("/farthest_point", 1000);
    // ros::Publisher closest_point_pub = n.advertise<sensor_msgs::LaserScan>("chatter", 1);
    // ros::Publisher farthest_point_pub = n.advertise<sensor_msgs::LaserScan>("chatter", 1);

    // while (ros::ok())
    // {

    // }
    ros::spin();

    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////
// class lidar {

// private:
//     ros::NodeHandle n;

//     ros::Subscriber laser_sub;

//     ros::Publisher closest_pub;
//     ros::Publisher farthest_pub;

// public:
//     lidar(){

//         n = ros::NodeHandle("~");

//         std::string scan_topic;
//         n.getParam("scan_topic", scan_topic);
//         // std::string scan_topic, point1_topic, point2_topic;
//         // n.getParam("scan_topic", scan_topic);
//         // n.getParam("point1_topic", point1_topic);
//         // n.getParam("point2_topic", point2_topic);
//         ros::Publisher closest_pub = n.advertise<std_msgs::Float64>("/point1_topic", 1);
//         ros::Publisher farthest_pub = n.advertise<std_msgs::Float64>("/point2_topic", 1);

//         ros::Subscriber laser_sub = n.subscribe(scan_topic, 1, &lidar::lidarCallback, this);  
//     }

//     void lidarCallback(const sensor_msgs::LaserScan & laser)
//     {
//         std_msgs::Float64 lidar_msg1;
//         std_msgs::Float64 lidar_msg2;

//         float min = 100;
//         float max = 1;

//         for (int i = 0; i < laser.ranges.size(); i++)
//         {
//             if(laser.ranges[i] < min )
//             {
//                 min = laser.ranges[i];
//             }

//             if(laser.ranges[i] > max)
//             {
//                 max = laser.ranges[i];
//             }
//         }

//         ROS_INFO("max: [%lf], min: [%lf]", max,  min);

//         lidar_msg1.data = max;
//         lidar_msg2.data = min;

//         closest_pub.publish(lidar_msg2);
//         farthest_pub.publish(lidar_msg1);
//     }
// };


// int main(int argc, char **argv){
//     ros::init(argc, argv, "lidar");
//     lidar ld;
//     ros::spin();
//     return 0;
// }
///////////////////////////////////////////////////////////////////////////////////////////////


// void lidarCallback(const sensor_msgs::LaserScan laser)
// {
//     float min = 100;
//     float max = 1;

//     for (int i = 0; i < laser.ranges.size(); i++)
//     {
//         if(laser.ranges[i] < min )
//         {
//             min = laser.ranges[i];
//         }

//         if(laser.ranges[i] > max)
//         {
//             max = laser.ranges[i];
//         }
//     }


//     ROS_INFO("max: [%lf], min: [%lf]", max,  min);
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "lidar");

//     ros::NodeHandle n;
//     n = ros::NodeHandle("~");

//     std::string scan_topic;
//     n.getParam("scan_topic", scan_topic);

//     ros::Subscriber laser_sub = n.subscribe(scan_topic, 10, lidarCallback);

//     // ros::Publisher closest_point_pub = n.advertise<sensor_msgs::LaserScan>("chatter", 1);
//     // ros::Publisher farthest_point_pub = n.advertise<sensor_msgs::LaserScan>("chatter", 1);

//     // while (ros::ok())
//     // {

//     // }
//     ros::spin();

//     return 0;
// }