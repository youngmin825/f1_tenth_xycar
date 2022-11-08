#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

pre_heading = 540
valid_range = 540

# kp = 0.7
# kd = 0.1
# ki = 0.00001
# prev_error = 0.0 
# error = 0.0
# integral = 0.0

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) #TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1) #TODO
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        global valid_range

        average_num = 30
        
        proc_ranges = [0 for i in range(valid_range)]

        for i in range(valid_range):
            for j in range(average_num):
                proc_ranges[i] += ranges[valid_range/2 + i - j]   # if i = 0 //  proc_ranges[0] = ranges[270] + ranges[269] + ...
            proc_ranges[i] = proc_ranges[i] / (average_num * 2)
        return proc_ranges

        # n = 20
        # proc_ranges = list(ranges[270:810])
        # for i in range(len(proc_ranges)):
        #     for j in range(n):
        #         if i - j + n/2 < 0:
        #             proc_ranges[i] += proc_ranges[i - j + n/2 + 660] / n
        #         elif i - j + n/2 >= 660:
        #             proc_ranges[i] += proc_ranges[i - j + n/2 - 660] / n
        #         else:
        #             proc_ranges[i] += proc_ranges[i - j + n/2] / n

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        gap_length = 0
        count = 0
        start_i = 0
        end_i = 0

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > 0.5:
                count += 1

            else:
                if gap_length < count:
                    gap_length = count
                    end_i = i
                    start_i = i - count
                count = 0
                
        return start_i, end_i
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        global valid_range
        best_index = 0
        far_dist = 0

        for i in range(start_i, end_i + 1):
            if far_dist < ranges[i]:
                far_dist = ranges[i]
                best_index = i

        return best_index + valid_range/2

        # print("%d, %d", start_i, end_i)
        return (end_i + start_i)/2 + valid_range/2

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        global pre_heading
        global valid_range

        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        min = 100
        min_idx = 0
        for i in range(len(proc_ranges)):
            if(proc_ranges[i] < min):
                min = proc_ranges[i]
                min_idx = i

        #Eliminate all points inside 'bubble' (set them to zero) 
        bubble_radius = 0.5

        i = 1
        while(1):
            if (min_idx + i) <= valid_range/2 - 1 and proc_ranges[min_idx + i] < (min + bubble_radius):
                proc_ranges[min_idx + i] = 0
                i = i + 1
            else:
                break

        i = 1
        while(1):
            if (min_idx - i) >= 0 and proc_ranges[min_idx - i] < (min + bubble_radius):
                proc_ranges[min_idx - i] = 0
                i = i + 1
            else:
                break

        #Find max length gap 
        start_i, end_i = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        best_point_index = self.find_best_point(start_i, end_i, proc_ranges)

        # # PID
        # global integral
        # global prev_error
        # global kp
        # global ki
        # global kd

        # error = -(pre_heading - best_point_index)

        # p_error = error
        # d_error = error - prev_error
        # i_error = integral + error

        # integral = integral + error
        # prev_error = error

        # angle = (kp * p_error) + (ki * i_error) + (kd * d_error)

        angle = -(pre_heading - best_point_index)
            
        if min < 1:
            angle = angle*2
        elif 1 < min < 2:
            angle = angle*1.5
        else:
            angle = angle
        print(start_i, end_i)
        print(best_point_index)
        print(ranges[best_point_index])
        print(angle)
        

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle * data.angle_increment
        drive_msg.drive.speed = 0.5
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
