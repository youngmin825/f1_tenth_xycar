#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import math
import csv

from std_msgs.msg import Int64
from std_msgs.msg import String


sectors = []

decision_pub  = rospy.Publisher('/purepursuit_control/adaptive_lookahead', String, queue_size = 1)

def construct_path():
    file_path = os.path.expanduser('~/xycar_ws/src/f1tenth_simulator/logs/wp-2022-11-22-15-46-54.csv')

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for sector in csv_reader:
            sectors.append(sector)

    # 앞의 x,y값만 float처리를 한다
    for sector in range(0, len(sectors)):
            sectors[sector][0] = float(sectors[sector][0])
            sectors[sector][1] = float(sectors[sector][1])
    
def pose_callback(data):

    curr_pose_index = data.data
    adaptive_state  = String()
    curr_pose_state = 'caution'

    for index in range(0, len(sectors)):

        # x값 보다 y값이 크다면
        if sectors[index][1] < sectors[index][0]:
            if curr_pose_index > sectors[index][0] or curr_pose_index < sectors[index][1]:
                curr_pose_state = str(sectors[index][2])
                # rospy.loginfo('current lookahead state: {}'.format(sectors[index][2]))

        # y값이 x값보다 크다면
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
