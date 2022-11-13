#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"
// TODO: include ROS msg type headers and libraries

class Safety {
// The class that handles emergency braking

private: // private에 NodeHandle과 publisher, subscriber을 선언한다.
    ros::NodeHandle n;
    double speed;
    bool in_collision=false;
    double ttc_threshold = 0.1;
    std::vector<bool> mux_controller;
    int mux_size;

    bool safety_on;

    racecar_simulator::CarState state;

    
    std::vector<double> cosines;

    std::vector<double> car_distances;
    // TODO: create ROS subscribers and publishers
    ros::Subscriber scan;
    ros::Subscriber odom;
    ros::Publisher mux_pub;
    // ros::Publisher TTC;

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        std::string odom_topic;
        n.getParam("odom_topic", odom_topic);


        n.getParam("mux_size", mux_size);
        printf("%d", mux_size);
        mux_controller.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) 
        {
            mux_controller[i] = false;
        }

        safety_on = false;

        int scan_beams;
        double scan_fov, scan_ang_incr, wheelbase, width, scan_distance_to_base_link;
        // n.getParam("ttc_threshold", ttc_threshold);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n.getParam("width", width);
        n.getParam("wheelbase", wheelbase);
        n.getParam("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;

        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = racecar_simulator::Precompute::get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = racecar_simulator::Precompute::get_car_distances(scan_beams, wheelbase, width, 
                scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods
        
        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        odom = n.subscribe("/odom", 1, &Safety::odom_callback, this);
        scan = n.subscribe("/scan", 1, &Safety::scan_callback, this);
        mux_pub = n.advertise<std_msgs::Int32MultiArray>("/mux", 10);
        // TTC = n.advertise<sensor_msgs::LaserScan>("/TTC", 10);
    }
    
    

    void publish_mux() {
        // make mux message
        std_msgs::Int32MultiArray mux_msg;
        mux_msg.data.clear();
        // push data onto message
        for (int i = 0; i < mux_size; i++) {
            mux_msg.data.push_back(int(mux_controller[i]));
        }

        // publish mux message
        mux_pub.publish(mux_msg);
    }

    void collision_helper() {
        // This function will turn off ebrake, clear the mux and publish it

        safety_on = false;

        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }

        publish_mux();
    }

    

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x;
        
        
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        // This function calculates TTC to see if there's a collision
        if (speed != 0) {
            for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;

                // calculate projected velocity
                double proj_velocity = speed * cosines[i];
                double ttc = (scan_msg->ranges[i] - car_distances[i]) / proj_velocity;

                // if it's small, there's a collision
                if ((ttc < ttc_threshold) && (ttc >= 0.0)) { 
                    // Send a blank mux and write to file
                    collision_helper();

                    in_collision = true;

                    return;
                }
            }
            // if it's gone through all beams without detecting a collision, reset in_collision
            in_collision = false;
    //     // TODO: publish drive/brake message
    }
        // TTC.publish(scan_msg);

    }


};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}