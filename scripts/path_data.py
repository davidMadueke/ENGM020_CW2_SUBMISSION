#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Int64, Bool

class PathGenerator:
    def __init__(self):
        rospy.init_node('path_generator')
        
        self.marker_pub = rospy.Publisher('/path_marker', Marker, queue_size=10)
        
        self.num_steps = 0
        self.dt = 0.0  # Time resolution
        self.simulation_ended = False
        rospy.Subscriber('/simulation_num_steps', Int64, self.num_steps_callback)
        rospy.Subscriber('/simulation_ended', Bool, self.simulation_ended_callback)
        rospy.Subscriber('/simulation_dt', Float64, self.simul_dt_callback)

        self.generate_path()
    
    def simul_dt_callback(self, data):
        self.dt = data.data

    def simulation_ended_callback(self,data):
        self.simulation_ended = data.data

    def num_steps_callback(self, data):
        self.num_steps = data.data

    def generate_path(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0  # Red color
        

        for i in range(self.num_steps + 50):
            x = i * self.dt
            y = x + 5  # Define the function y = x^3

            point = Point()
            point.x = x
            point.y = y
            point.z = 0  # Assuming 2D motion

            marker.points.append(point)

        self.marker_pub.publish(marker)
        rospy.loginfo(f"Publishing Line Path: {marker.points}")


if __name__ == '__main__':
    try:
        PathGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

