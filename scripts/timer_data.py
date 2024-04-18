#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float64, Int64, Bool

# Initialize ROS node
rospy.init_node('timer_data', anonymous=True)

# Create a publisher to publish Simulation Time Steps
pub_current_time_step = rospy.Publisher('/simulation_current_time_step', Float64, queue_size=10)
pub_num_steps = rospy.Publisher('/simulation_num_steps', Int64, queue_size=10)
pub_dt = rospy.Publisher('/simulation_dt', Float64, queue_size=10)
pub_simulation_ended = rospy.Publisher('/simulation_ended', Bool, queue_size=10)

def data_generator(t_start: int = 0, t_end: int = 120, 
		dt: float = 0.1, simulation_ended: bool = False):
		
	num_steps = int((t_end - t_start) / dt)
	if not simulation_ended:
		for i in range(num_steps):
			current_time_step = 0 + i*dt
		
			pub_dt.publish(dt)
			pub_num_steps.publish(num_steps)
			pub_current_time_step.publish(current_time_step)
			rospy.loginfo(f"Publishing Current Time Step: {current_time_step}")
			rospy.sleep(dt)  # Sleep to control publishing rate
			
		simulation_ended = True
	pub_simulation_ended.publish(simulation_ended)
	rospy.loginfo("Simulation has ended")
	rospy.spin()
		


if __name__ == '__main__':
    # Time parameters
    t_start = 0
    t_end = 120
    dt = 0.1
    num_steps = int((t_end - t_start) / dt)
    try:
        data_generator()
    except rospy.ROSInterruptException:
        pass

