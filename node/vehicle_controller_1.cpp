#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>


#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <tf/transform_listener.h>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>

#define WHEELBASE_LEN 0.0381


class vehicle_controller {
private:
    // A ROS node
    ros::NodeHandle n;

    // Listen for messages from joystick, keyboard, laser scan, odometry, and IMU
    ros::Subscriber odom_sub;
    ros::Publisher odom_pub_1;
    ros::Publisher odom_pub_2;


    tf::TransformListener heading;
    nav_msgs::Odometry odom_test_1;
    nav_msgs::Odometry odom_test_2;

    int use_ackermann_model;
    std_msgs::Int64 min_index;
    int i;
    float curr_x;
    float curr_y;
    

public:
    vehicle_controller() {
        // Initialize the node handle
        n = ros::NodeHandle("~");



        odom_sub = n.subscribe("/odom", 1, &vehicle_controller::odom_callback, this);
        odom_pub_1 = n.advertise<nav_msgs::Odometry>("/odom_test_1", 10);
        odom_pub_2 = n.advertise<nav_msgs::Odometry>("/odom_test_2", 50);
        
        i = 0;
        curr_x = 0.0;
        curr_y = 0.0;

    }

    // void construct_path()
    // {
    //     fstream file;
    //     vector<double> array;
    //     file.open("~/xycar_ws/src/f1tenth_simulator/logs/wp-2022-11-22-15-46-54.csv",ios::in);
    //     while (!file.eof())
    //     {
    //         file >> array[i];
    //         i += 1;
    //     }
    //     i = 0;
    // }
    void odom_callback(const nav_msgs::Odometry & data) {
        // Keep track of state to be used elsewhere
        
     

        odom_test_1 = data;
        odom_test_1.header.frame_id = "odom_1";

        odom_test_2 = data;
        odom_test_2.header.frame_id = "odom_2";
        

        odom_pub_2.publish(odom_test_2);
        ros::Rate rate(0.1);
        odom_pub_1.publish(odom_test_1);
        

    }



};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "odom_test");
    vehicle_controller bc;
    ros::spin();
    return 0;
}
