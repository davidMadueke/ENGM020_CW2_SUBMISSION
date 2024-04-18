#!/usr/bin/env python3
import rospy
import numpy as np
from david_dubins_car.msg import State
from visualization_msgs.msg import Marker

def callback(data):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.lifetime.secs = 0
    marker.pose.position.x = data.x
    marker.pose.position.y = data.y
    marker.pose.position.z = 0

    marker.pose.orientation.x = data.x
    marker.pose.orientation.y = data.y
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = data.theta
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0  # Don't forget to set the alpha!
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    pub.publish(marker)

def data_processor():
    global pub
    rospy.init_node('sensor_data_processor', anonymous=True)
    rospy.Subscriber("/sensor_data", State, callback)
    pub = rospy.Publisher('/sensor_data_visualization', Marker, queue_size=10)
    #pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    data_processor()

