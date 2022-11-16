#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

pre_heading = 540
valid_range = 540
start_idx = 270


class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) #TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1) #TODO
        
        self.ranges = np.array(0)

    # 전처리를 한다.
    def preprocess_lidar(self):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        
        global valid_range
        global start_idx

        # 앞뒤 lidar index를 앞뒤 5개씩의 값을 가지고 평균을 구한다. 
        average_num = 5

        
        # proc_range는 앞의 180도만 보게 제안한다.
        proc_ranges = [0 for i in range(valid_range)]

        # lidar range의 평균을 구한다.
        for i in range(valid_range):
            for j in range(average_num):
                

                proc_ranges[i] += self.ranges[start_idx + i - j]  

            for j in range(average_num):

                  

                proc_ranges[i] += self.ranges[start_idx + i + j]

            proc_ranges[i] = proc_ranges[i] / (average_num * 2)
    
        
        
        return proc_ranges

        
    # 최대 gap을 찾는다.
    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        
        gap_length = 0
        count = 0
        start_i = 0
        end_i = 0

        # 거리가 2.0이상인 것만 추려낸다.
        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > 2.0:
                count += 1

            else:
                if gap_length < count:
                    gap_length = count
                    end_i = i
                    start_i = i - count
                count = 0
                
        # gap의 시작 인덱스와 끝 인덱스이다.
        return start_i, end_i
    
    # gap에서 가장 먼 점을 찾는다.
    def find_best_point(self, start_i, end_i, proc_ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        
        global valid_range
        best_index = 0
        far_dist = 0
        count = 0
        for i in range(start_i, end_i + 1):
            if far_dist < proc_ranges[i]:
                far_dist = proc_ranges[i]
                best_index = i

        # best index(180도만 본다.)를 원래 360도 인덱스로 바꿔 준다.
        return best_index + valid_range* 0.5

        
        return (end_i + start_i)/2 + valid_range/2

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        global pre_heading
        global valid_range

        self.ranges = np.array(data.ranges)
        proc_ranges = self.preprocess_lidar()

        #Find closest point to LiDAR
        ###############################################
        min = 100
        min_idx = 0
        for i in range(len(proc_ranges)):
            if(proc_ranges[i] < min):
                min = proc_ranges[i]
                min_idx = i
        ###############################################

        #Eliminate all points inside 'bubble' (set them to zero) 
        ###############################################
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
        ###############################################

        #Find max length gap 
        start_i, end_i = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        best_point_index = self.find_best_point(start_i, end_i, proc_ranges)

        
        # 현재 앞의 인덱스와 best_index 사이의 차이를 각도로 설정한다. 
        angle = -(pre_heading - best_point_index)
        speed = 0.5

        # print('angle',angle * data.angle_increment )
        angle = angle * data.angle_increment 

        # 각도에 따라 속도를 달리한다.
        if abs(angle) > 0.3:
            speed = speed * 0.7
           
        elif abs(angle) > 0.17:
            speed = speed * 1.0
          
        else:
        
            speed = speed * 1.1

        
        # print(start_i, end_i)
        # print('min',min)
        # print(best_point_index)
        # print(ranges[best_point_index])
        # print(ranges[540])

        # print('best_point', best_point_index, 'distance',self.ranges[int(best_point_index)])

        
        # print('angle',angle)
        
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle 
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)