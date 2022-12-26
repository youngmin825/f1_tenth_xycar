#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import math
import csv

from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped


plan = []

# odom과 waypoint 사이의 거리값 중에서 최소값을 가진 index를 보낸다.
min_index_pub = rospy.Publisher('/purepursuit_control/index_nearest_point', Int64, queue_size = 1)

# 최소의 거리값을 가지는 x,y 값을 보낸다.
min_pose_pub  = rospy.Publisher('/purepursuit_control/visualize_nearest_point', PoseStamped, queue_size = 1)

def construct_path():
    file_path = os.path.expanduser('~/xycar_ws/src/f1tenth_simulator/logs/wp-2022-11-22-15-46-54.csv')

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)
            # print(waypoint)

    # 각 waypoint의 점들을 모두 실수로 바꾼다.
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])
            # print(plan[index][point])
def odom_callback(data):
    min_index      = Int64()
    curr_x         = data.pose.pose.position.x
    curr_y         = data.pose.pose.position.y
    min_index.data = find_nearest_point(curr_x, curr_y)
    min_index_pub.publish(min_index)

    pose                 = PoseStamped()
    pose.pose.position.x = plan[min_index.data][0]
    pose.pose.position.y = plan[min_index.data][1]
    min_pose_pub.publish(pose)

def find_nearest_point(curr_x, curr_y):
    ranges = []
    for index in range(0, len(plan)):
        eucl_x = math.pow(curr_x - plan[index][0], 2)
        eucl_y = math.pow(curr_y - plan[index][1], 2)
        eucl_d = math.sqrt(eucl_x + eucl_y)
        ranges.append(eucl_d)
    
    # 거리값 중에서 최소값을 가진 index를 보낸다.
    return(ranges.index(min(ranges)))

if __name__ == '__main__':
    try:
        rospy.init_node('nearest_pose_isolator', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        rospy.Subscriber('/odom', Odometry, odom_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
