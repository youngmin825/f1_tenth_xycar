#!/usr/bin/python
# -*- coding: utf-8 -*-


# https://github.com/aamishhussain/path_following/tree/1aedd93e5f6d6641f43caf863ae4aea7a151afd6
import rospy
import sys
import os
import math
import csv

from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped


plan = []


min_index_pub = rospy.Publisher('/purepursuit_control/index_nearest_point', Int64, queue_size = 1)


min_pose_pub  = rospy.Publisher('/purepursuit_control/visualize_nearest_point', PoseStamped, queue_size = 1)

# waypoint를 불러온다.
def construct_path():
    file_path = os.path.expanduser('~/xycar_ws/src/f1tenth_simulator/logs/wp-2022-11-22-15-46-54.csv')

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)
            # print(waypoint)

    # plan[x좌표][y좌표]
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])
            # print(plan[index][point])

# 현재 odometry값을 받아서 현재에서 가장 가까운 waypoint를 찾는다.
def odom_callback(data):
    
    curr_x = float(data.pose.pose.position.x)
    curr_y = float(data.pose.pose.position.y)
    min_index      = Int64()
    min_index.data = find_nearest_point(curr_x, curr_y)
    # rospy.loginfo(data)
    rospy.loginfo("curr_x= %f",curr_x)
    # rospy.loginfo("min_index.data= %d",min_index.data)
    min_index_pub.publish(min_index)

    pose                 = PoseStamped()
    pose.pose.position.x = plan[min_index.data][0]
    pose.pose.position.y = plan[min_index.data][1]
    min_pose_pub.publish(pose)
    # rospy.sleep(0.1)

# 가장 가까운 waypoint를 찾는 함수
def find_nearest_point(curr_x, curr_y):
    ranges = []
    for index in range(0, len(plan)):
        eucl_x = math.pow(curr_x - plan[index][0], 2)
        eucl_y = math.pow(curr_y - plan[index][1], 2)
        eucl_d = math.sqrt(eucl_x + eucl_y)
        ranges.append(eucl_d)

    # rospy.loginfo(ranges)
    

    return(ranges.index(min(ranges)))

if __name__ == '__main__':
    
    try:
        rospy.init_node('nearest_pose_finder_node', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

        # queue_size를 설정하면 전송중에 손실된 데이터를 담아둘 수 있다. 그러면 odom이 0이 되는 것을 막을 수 있다.
        rospy.Subscriber('/odom_test_2', Odometry, odom_callback, queue_size=5)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
