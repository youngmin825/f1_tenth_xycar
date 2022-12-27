#!/usr/bin/python
# https://answers.ros.org/question/359624/convert-imu-to-odom/
import rospy
from nav_msgs.msg import Odometry
# import smbus
import math
from math import sin, cos, pi
import tf 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu



pub = rospy.Publisher('/odom1', Odometry, queue_size=10)
# Register



def imu_to_odom(data):

    convert = Imu()
    convert = data
    odom_broadcaster = tf.TransformBroadcaster()
    
    start_time = rospy.Time.now()
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    x = 0.0
    y = 0.0
    th = 0.0
    rate = rospy.Rate(10) # 10hz while not rospy.is_shutdown(): odom = Odometry()
    


    vx = convert.linear_acceleration.x
    vy = convert.linear_acceleration.y
    # th = yaw
    # https://www.programcreek.com/python/example/103985/tf.transformations.euler_from_quaternion
    roll, pitch, th = tf.transformations.euler_from_quaternion([convert.orientation.x,convert.orientation.y,convert.orientation.z,convert.orientation.w])

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    # delta_th = vth * dt

    x += delta_x
    y += delta_y
    # th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(roll, pitch, th)
    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(roll, pitch, th))
    pub.publish(odom)
    rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('imu_to_odom', anonymous=True)
        
        sub = rospy.Subscriber('/imu', Imu, imu_to_odom ,queue_size=10)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass