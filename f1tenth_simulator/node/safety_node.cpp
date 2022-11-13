#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include "f1tenth_simulator/precompute.hpp"

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    // TODO: create ROS subscribers and publishers

    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;

    // ros::Publisher drive_pub;
    ros::Publisher controll_pub;

    std::vector<double> cosines;

    std::vector<double> car_distances;

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
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

        // get topic name
        std::string scan_topic, odom_topic, brake_bool_topic, drive_topic;
        n.getParam("scan_topic", scan_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("brake_bool_topic", brake_bool_topic);
        // n.getParam("drive_topic", drive_topic);

        //subscribe
        scan_sub = n.subscribe("/scan", 1, &Safety::scan_callback, this);
        odom_sub = n.subscribe("/odom", 1, &Safety::odom_callback, this);

        //publish
        controll_pub = n.advertise<std_msgs::Bool>("/brake_bool", 1);
        // drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);

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

        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        std_msgs::Bool brake_on;
        ackermann_msgs::AckermannDriveStamped velocity;

        // TODO: calculate TTC
        if(speed != 0)
        {
            for(size_t i = 0; i < scan_msg->ranges.size(); i++)
            {
                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                
                // calculate projected velocity
                double proj_velocity = speed * cosines[i];
                // printf("ttc = %lf\n", proj_velocity);
                double ttc = (scan_msg->ranges[i] - car_distances[i]) / proj_velocity;
                // printf("ttc = %lf\n", ttc);
                if((ttc < 0.1) && (ttc >=0))
                {
                    brake_on.data = true;
                    velocity.drive.speed = 0.0;
                    // printf("ttc = %lf\n", ttc);
                }
                else
                {
                    brake_on.data = false;
                    velocity.drive.speed = speed;
                }
            }
        }
        // TODO: publish drive/brake message
        controll_pub.publish(brake_on);
        // drive_pub.publish(velocity);
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}