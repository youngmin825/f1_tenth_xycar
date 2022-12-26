#!/usr/bin/python
from __future__ import print_function
from codecs import ignore_errors
import sys
import math
# from termios import VEOL
# import numpy as np
# from yaml import scan

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
# kp = 0.0
# kd = 0.0
# ki = 0.

# top score
kp = 0.65
kd = 0.25
ki = 0.00000001
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
Dt = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 0.50 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = "/scan"
        drive_topic = "/nav"

        # rospy.init_node("wall_follow", anonymous=False)

        # Subscriber
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)

        # Publisher
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)



    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement

        Dist = []
        for i in range(len(angle)):
            # number control
            if data.ranges[i] != 0:
                Dist.append(data.ranges[i])
            else:
                Dist.append = 0

        return Dist

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd

        # rospy.loginfo("error = %f", error)
        angle = 0.0
        speed = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller for
        p_error = error
        d_error = error - prev_error
        i_error = integral + error

        # print(p_error, d_error, i_error)

        integral = integral + error
        prev_error = error

        angle = (kp * p_error) + (ki * i_error) + (kd * d_error)

        # rospy.loginfo("angle = %f", angle)

        if(angle >= -10 / 180 * math.pi or angle <= 10 / 180 * math.pi ):
            speed = 2.0
        elif(((angle >= -20 / 180 * math.pi) and (angle <= -10 / 180 * math.pi)) or ((angle <= 20 / 180 * math.pi) and (angle >= 10 / 180 * math.pi))):
            speed = 1.5
        else:
            speed = 1.0
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        drive_msg.drive.jerk = Dt
        self.drive_pub.publish(drive_msg)

    def followLeft(self, distance, leftDist, data, angle):
        #Follow left wall as per the algorithm 
        #TODO:implement
        global Dt
        
        b = distance[270]
        a = distance[400]


        # rospy.loginfo("0 = %d , 270 = % d, 540 = %d, 810 = %d", distance[0], distance[270], distance[540], distance[810])
        rospy.loginfo("a = %f, b = %f", a, b)
        theta= 130 * data.angle_increment

        alpha = math.atan( (a*math.cos(theta) - b) / (a*math.sin(theta)) )
        Dt = b * math.cos(alpha)

        Dt_1 = Dt + (VELOCITY)*math.sin(alpha)

        error = leftDist - Dt_1 


        return error 

    def lidar_callback(self, data):
        angle = []
        scan_beam = 1080
       
        for i in range(scan_beam):
            angle.append(data.angle_min + i * data.angle_increment)

        distance = self.getRange(data, angle)
        error = self.followLeft(distance, DESIRED_DISTANCE_RIGHT, data, angle) 
        #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)