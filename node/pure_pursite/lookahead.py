#!/usr/bin/python
# -*- coding: utf-8 -*-


# https://github.com/aamishhussain/path_following/tree/1aedd93e5f6d6641f43caf863ae4aea7a151afd6
import rospy
import sys
import os
import math
import csv

from std_msgs.msg import Int64
from std_msgs.msg import String


sectors = []

decision_pub  = rospy.Publisher('/purepursuit_control/adaptive_lookahead', String, queue_size = 1)

# waypoint를 불러온다.
def construct_path():
    file_path = os.path.expanduser('~/xycar_ws/src/f1tenth_simulator/logs/wp-2022-11-22-15-46-54.csv')

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for sector in csv_reader:
            sectors.append(sector)

    # sectors[sector][0]: x좌표, sectors[sector][1]: y좌표
    for sector in range(0, len(sectors)):
            sectors[sector][0] = float(sectors[sector][0])
            sectors[sector][1] = float(sectors[sector][1])

# 현재 위치 상태를 파악한다.   
def pose_callback(data):

    curr_pose_index = data.data
    adaptive_state  = String()
    curr_pose_state = 'caution'

    for index in range(0, len(sectors)):

 
        if sectors[index][1] < sectors[index][0]:
            if curr_pose_index > sectors[index][0] or curr_pose_index < sectors[index][1]:
                curr_pose_state = str(sectors[index][2])
                # rospy.loginfo('current lookahead state: {}'.format(sectors[index][2]))

        
        else:
            if curr_pose_index > sectors[index][0] and curr_pose_index < sectors[index][1]:
                curr_pose_state = str(sectors[index][2])
                # rospy.loginfo('current lookahead state: {}'.format(sectors[index][2]))

    adaptive_state.data = curr_pose_state
    decision_pub.publish(adaptive_state)

if __name__ == '__main__':
    try:
        rospy.init_node('adaptive_lookahead_node', anonymous = True)
        construct_path()
        rospy.Subscriber('/purepursuit_control/index_nearest_point', Int64, pose_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
