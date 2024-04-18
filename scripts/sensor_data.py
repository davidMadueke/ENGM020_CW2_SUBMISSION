#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float64, Int64, Bool
from david_dubins_car.msg import State # Defined a custom State message returning [X,Y,\THETA]^T

# Initialize ROS node
rospy.init_node('dubins_car_model', anonymous=True)

# Create a publisher to publish X,Y coordinates
pub = rospy.Publisher('/sensor_data', State, queue_size=10)


# Function defining the Dubins car dynamics
def dubins_car_dynamics(state, v, R, delta):
    x_dot = v * np.cos(state[2])
    y_dot = v * np.sin(state[2])
    theta_dot = (v / R) * np.tan(delta)
    return np.array([x_dot, y_dot, theta_dot])

# Function to integrate the dynamics using Euler's method
def integrate_dynamics_euler(dynamics, initial_state, v, R, dt, num_steps):
    states = [initial_state]
    for _ in range(num_steps):
        while not rospy.is_shutdown():
        	current_state = states[-1]
        	delta = pure_pursuit_controller(current_state)
        	next_state = current_state + dynamics(current_state, v, R, delta) * dt
        	states.append(next_state)
        	# Publish State
        	publish_state = State()
        	publish_state.x = next_state[0]
        	publish_state.y = next_state[1]
        	publish_state.theta = next_state[2]
        	pub.publish(publish_state)
        	rospy.loginfo(f"Publishing States: {publish_state}")
        	rospy.sleep(dt)  # Sleep to control publishing rate
    return np.array(states)

# Steering angle function (for example)
def pure_pursuit_controller(state, L_d = 1.0):
    return 0.1  # Constant steering angle for demonstration

# Define callback function that monitors if the simulation has ended
def simulation_callback(data):
	pass

if __name__ == '__main__':
    # Define parameters
    v = 1.0  # Speed of the car
    R = 1.0  # Turn radius of the car

    # Initial state
    x0 = 0.0
    y0 = 0.0
    theta0 = 0.0

    # Initial conditions
    initial_state = np.array([x0, y0, theta0])

    # Time parameters
    t_start = 0
    t_end = 120
    dt = 0.1
    num_steps = int((t_end - t_start) / dt)
    try:
        states = integrate_dynamics_euler(dubins_car_dynamics, initial_state, v, R , dt, num_steps)
    except rospy.ROSInterruptException:
        pass

