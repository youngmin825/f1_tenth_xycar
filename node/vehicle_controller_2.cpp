#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <fstream>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <tf/transform_listener.h>
#include <cmath>

#define WHEELBASE_LEN 0.0381


class vehicle_controller_2 {
private:
    // A ROS node
    ros::NodeHandle n;

    // Listen for messages from joystick, keyboard, laser scan, odometry, and IMU
    ros::Subscriber odom_sub;
    ros::Publisher odom_pub_2;


    float ang_goal_x;
    float ang_goal_y;

    float vel_goal_x;
    float vel_goal_y;

    int lookahead_state;

    float integral;
    float prev_error;
    float kp;
    float ki;
    float kd;
    float error;
    float servo_offset;

    float curr_x;
    float curr_y;
    tf::TransformListener heading;
    nav_msgs::Odometry odom_test_2;

    int use_ackermann_model;



public:
    vehicle_controller_2() {
        // Initialize the node handle
        n = ros::NodeHandle("a");

        

        odom_sub = n.subscribe("/odom_test_1", 1, &vehicle_controller_2::odom_callback_2, this);
        odom_pub_2 = n.advertise<nav_msgs::Odometry>("/odom_test_2", 10);
        ang_goal_x = 0.0;
        ang_goal_y = 0.0;

        vel_goal_x = 0.0;
        vel_goal_y = 0.0;

        kp = 0.65;
        kd = 0.25;
        ki = 0.00000001;
        servo_offset = 0.0;
        prev_error = 0.0;
        error = 0.0;
        integral = 0.0;


        use_ackermann_model = 1;
        


    }

    void odom_callback_2(const nav_msgs::Odometry & data) {
        // Keep track of state to be used elsewhere
        // float front_axle_x;
        // float front_axle_y;

        // float proj_x;
        // float proj_y;

        // float eucl_d;



        // curr_x = data.pose.pose.position.x;
        // curr_y = data.pose.pose.position.y;
        
        // heading = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x,
        //                                                 data.pose.pose.orientation.y,
        //                                                 data.pose.pose.orientation.z,
        //                                                 data.pose.pose.orientation.w))[2];

        // if (use_ackermann_model == 1)
        // {
        //     front_axle_x = (WHEELBASE_LEN * math.cos(heading)) + curr_x;
        //     front_axle_y = (WHEELBASE_LEN * math.sin(heading)) + curr_y;

        //     curr_x = front_axle_x;
        //     curr_y = front_axle_y;

        // }
        // std::cout << "curr_x : " << curr_x << std::endl;

        // eucl_d = sqrt(pow(ang_goal_x - curr_x, 2) + pow(ang_goal_y - curr_y, 2));

        // proj_x = eucl_d * cos(heading) + curr_x;
        // proj_y = eucl_d * sin(heading) + curr_y;

        odom_test_2 = data;

        odom_pub_2.publish(odom_test_2);


    }



};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "odom_test_2");
    vehicle_controller_2 bc;
    ros::spin();
    return 0;
}
