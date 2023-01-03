#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import math
import csv

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64
from std_msgs.msg import String

global ang_lookahead_dist
global vel_lookahead_dist

adaptive_lookahead  = str(sys.argv[1])
ang_lookahead_dist  = float(sys.argv[2])

plan = []

# if ang_lookahead_dist > 5.0:
#     ang_lookahead_dist = 500
#     vel_lookahead_dist = 1000
# else:
#     ang_lookahead_dist = int(ang_lookahead_dist * 100)
#     vel_lookahead_dist = ang_lookahead_dist * 2

global plan_size
global seq

plan         = []
frame_id     = 'map'
seq          = 0
ang_goal_pub = rospy.Publisher('/purepursuit_control/ang_goal', PoseStamped, queue_size = 1)
vel_goal_pub = rospy.Publisher('/purepursuit_control/vel_goal', PoseStamped, queue_size = 1)
plan_size    = 0

# waypoint를 불러온다.
def construct_path():
    global plan_size
    file_path = os.path.expanduser('~/xycar_ws/src/f1tenth_simulator/logs/wp-2022-11-22-15-46-54.csv')

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # plan[x좌표][y좌표]
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])
        # print(plan[index])
    plan_size = len(plan)

# use adaptive lookahead as reported

brake_lookahead        = 2.00
caution_lookahead      = 2.50
unrestricted_lookahead = 3.00



if adaptive_lookahead != 'true':
    ang_lookahead_dist = int(float(ang_lookahead_dist) * 100)
    vel_lookahead_dist = ang_lookahead_dist * 2
else:
    ang_lookahead_dist = 100
    vel_lookahead_dist = 200

# 상태에 따라서 현재 가장 가까운 waypoint를 기준으로 얼마나 앞을 볼 것인지 결정한다.
def dist_callback(data):
    global ang_lookahead_dist
    global vel_lookahead_dist

    if data.data == 'brake':
        ang_lookahead_dist = int(brake_lookahead * 100)
    elif data.data == 'caution':
        ang_lookahead_dist = int(caution_lookahead * 100)
    else:
        ang_lookahead_dist = int(unrestricted_lookahead * 100)

    vel_lookahead_dist = ang_lookahead_dist * 2

# 목표점인 waypoint를 topic으로 보낸다. 
def pose_callback(data):
    global seq
    global plan_size
    global ang_lookahead_dist
    global vel_lookahead_dist

    # rospy.loginfo("1")

    # ang_lookahead_dist = int(rospy.get_param('lookahead_dist'))

    # pose_index = data.data + ang_lookahead_dist

    # if pose_index >= plan_size - 1:
    #     pose_index = pose_index - plan_size - 2

    pose_index = (data.data + ang_lookahead_dist) % plan_size
    # rospy.loginfo("start = %f", plan[data.data % plan_size][1])
    # rospy.loginfo("start = %f", plan[pose_index][1])
    goal                    = PoseStamped()
    goal.header.seq         = seq
    goal.header.stamp       = rospy.Time.now()
    goal.header.frame_id    = frame_id
    goal.pose.position.x    = plan[pose_index][0]
    goal.pose.position.y    = plan[pose_index][1]
    goal.pose.orientation.z = plan[pose_index][2]
    goal.pose.orientation.w = plan[pose_index][3]
    # rospy.loginfo("goal= %f, index = %d, look = %d", goal.pose.position.x, pose_index, ang_lookahead_dist)
    
    ang_goal_pub.publish(goal)

    # pose_index = data.data + vel_lookahead_dist

    # if pose_index > plan_size:
    #     pose_index = pose_index - plan_size


    pose_index_2 = (data.data + vel_lookahead_dist) % plan_size
    # rospy.loginfo('current index: {}/{}'.format(pose_index, plan_size))

    goal_2                    = PoseStamped()
    goal_2.header.seq         = seq
    goal_2.header.stamp       = rospy.Time.now()
    goal_2.header.frame_id    = frame_id
    goal_2.pose.position.x    = plan[pose_index_2][0]
    goal_2.pose.position.y    = plan[pose_index_2][1]
    goal_2.pose.orientation.z = plan[pose_index_2][2]
    goal_2.pose.orientation.w = plan[pose_index_2][3]

    seq = seq + 1

    vel_goal_pub.publish(goal_2)

if __name__ == '__main__':
    try:
        rospy.init_node('nearest_goal_isolator', anonymous = True)
        rospy.set_param('lookahead_dist', ang_lookahead_dist)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        if adaptive_lookahead == 'true':
            rospy.Subscriber('/purepursuit_control/adaptive_lookahead', String, dist_callback)
        rospy.Subscriber('/purepursuit_control/index_nearest_point', Int64, pose_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
