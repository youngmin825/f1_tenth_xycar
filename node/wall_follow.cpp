#include <ros/ros.h>
#include <ros/package.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class WallFollow
{
private:
    ros::NodeHandle n;

    ros::Subscriber lidar_sub;
    ros::Publisher drive_pub;

    // float kp = 0.5;
    // float kd = 0.25;
    // float ki = 0.0005;
    float kp = 2.0;
    float kd = 0;
    float ki = 0;
    float servo_offset = 0.0;
    float prev_error = 0.0 ;
    float error = 0.0;
    float integral = 0.0;

    //WALL FOLLOW PARAMS
    int ANGLE_RANGE = 270; // Hokuyo 10LX has 270 degrees scan
    float DESIRED_DISTANCE_RIGHT = 0.9; // meters
    float DESIRED_DISTANCE_LEFT = 0.55;
    float VELOCITY = 2.00; // meters per second
    float CAR_LENGTH = 0.50; // Traxxas Rally is 20 inches or 0.5 meters

public:
    WallFollow()
    {
        n = ros::NodeHandle();

        //subscribe
        lidar_sub = n.subscribe("/scan", 1, &WallFollow::lidar_callback, this);

        //publish
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
    }

    float* getRange(sensor_msgs::LaserScan data, float* angle)
    {
        float Dist[sizeof(angle)];

        for(int i = 0; i < sizeof(angle); i++)
        {
            if(data.ranges[i] != 0)
            {
                Dist[i] = data.ranges[i];
            }
            else{
                Dist[i] = 0;
            }
        }

        return Dist;     
    }

    void pid_control(float error, float velocity)
    {
        float angle = 0.0;
        float speed = 0.0;

        float p_error = error;
        float d_error = error - prev_error;
        float i_error = prev_error + error;

        prev_error = error;

        angle = (kp * p_error) + (ki * i_error) + (kd * d_error);

        if(angle >= -10 / 180 * M_PI || angle <= 10 / 180 * M_PI )
            speed = 3.0;
        else if(((angle >= -20 / 180 * M_PI) && (angle <= -10 / 180 * M_PI)) || ((angle <= 20 / 180 * M_PI) && (angle >= 10 / 180 * M_PI)))    
            speed = 2.0;
        else
            speed = 1.0;


        ackermann_msgs::AckermannDriveStamped drive_msg;
        // drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = speed;
        drive_pub.publish(drive_msg);
    }

    float followLeft(float* data, float leftDist)
    {
        sensor_msgs::LaserScan laser;

        float b = data[50];
        float a = data[70];

        float theta = 70*laser.angle_increment;

        float alpha = atan( (a*cos(theta) - b) / (a*sin(theta)));
        float Dt = b * cos(alpha);

        float Dt_1 = Dt + 2*sin(alpha);

        float error = leftDist - Dt_1;

        return error;
    }

    void lidar_callback(sensor_msgs::LaserScan data)
    {
        float angle[sizeof(int((2*M_PI)/data.angle_increment))];
        float dist = 0.9;
        float velocity = 2;
        for(int i = 0; i < sizeof(angle); i++)
        {
            angle[i] = i * data.angle_increment;
        }

        float* distance = getRange(data, angle);
        error = followLeft(distance, dist);
        pid_control(error, velocity);
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "WallFollow_node");
    WallFollow sn;
    ros::spin();  
    return 0;
}