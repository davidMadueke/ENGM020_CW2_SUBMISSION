#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float64, Int64, Bool
from david_dubins_car.msg import State # Defined a custom State message returning [X,Y,\THETA]^T

# Function defining the Dubins car dynamics
def dubins_car_dynamics(state, v, R, delta):
	x_dot = v * np.cos(state[2])
	y_dot = v * np.sin(state[2])
	theta_dot = (v / R) * np.tan(delta)
	return np.array([x_dot, y_dot, theta_dot])

def TargetPositions(num_steps):
    ## Declare Constants   
    t = np.linspace(0,2*np.pi,num_steps)
    ## Equation for X and Y coordinate
    X_target = 20 * np.sin(t)
    Y_target = 10 * np.sin(t) * np.cos(t)
    ## concatenate X and Y coordinates
    target_positions = np.vstack([X_target,Y_target])
    target_positions = np.transpose(target_positions)

    return target_positions  

def ClosestPoint(CurrentPosition, TargetPosition, loop):
    dist = np.sqrt( np.square( TargetPosition[:,0] - CurrentPosition[0] ) + np.square( TargetPosition[:,1] - CurrentPosition[1] ) )
    ClosestIndex = np.argmin(dist)                   ##Find index at which minimum distance occurs
    ## Error Removal
    if (ClosestIndex == 149) & (loop < 1/3):
        return 1
    return ClosestIndex

def LookAheadPoint(CurrentPosition,ClosestIndex,L_d):
    for i in range(ClosestIndex+1,len(TPos)):
        dist = np.sqrt( np.square( TPos[i,0] - CurrentPosition[0] ) + np.square( TPos[i,1] - CurrentPosition[1] ) )
        if dist >= L_d:
            return TPos[i]
    return TPos[-1]
    
class SensorNode:
	def __init__(self):
		# Initialize ROS node
		rospy.init_node('dubins_car_model', anonymous=True)
		# Create a publisher to publish X,Y coordinates
		self.pub = rospy.Publisher('/sensor_data', State, queue_size=10)
		# Initialise and subscribe to topics
		self.dt = 0
		self.num_steps= 0
		self.simulation_ended = False
		rospy.Subscriber('/simulation_dt', Float64, self.simul_dt_callback)
		rospy.Subscriber('/simulation_num_steps', Int64, self.num_steps_callback)
		rospy.Subscriber('/simulation_ended', Bool, self.simulation_ended_callback)
		
	    # Define parameters
		self.v = 1.0  # Speed of the car
		self.R = 1.0  # Turn radius of the car
		self.L_d = 1.0 # Lookahead distance of the car
		self.L = 1.0 # Wheelbase for the car
    		
		# Initial state
		x0 = 0.0
		y0 = 0.0
		theta0 = 0.0

		# Initial conditions
		self.initial_state = np.array([x0, y0, theta0])
		self.states = []
    	
    # Subscriber Callbacks
	def simul_dt_callback(self, data):
		self.dt = data.data

	def num_steps_callback(self, data):
		self.num_steps = data.data

	def simulation_ended_callback(self, data):
		self.simulation_ended = data.data
		
	# Steering angle function
	def pure_pursuit_controller(self, current_state):
		target_position = TargetPositions(self.num_steps)
		closest_point_index = ClosestPoint(current_state, target_position, self.dt)
		lookahead_point = LookAheadPoint(current_state, closest_point_index, self.L_d)

		# Calculate alpha and kappa
		xl = lookahead_point[0]
		yl = lookahead_point[1]
		x = current_state[0]
		y = current_state[1]
		alpha = arctan2(yl - y, xl - x)
		kappa = (2 * np.sin(alpha))/ self.L_d
		
		# Use that to find and return delta
		delta = np.arctan((self.L*kappa)/1)
		return delta
	
    # Function to integrate the dynamics using Euler's method
	def integrate_dynamics_euler(self, model_dynamics):
		states = [self.initial_state]
		while not self.simulation_ended:
			current_state = states[-1]
			delta = self.pure_pursuit_controller(current_state)
			next_state = current_state + model_dynamics(current_state, self.v,self.R, delta) * self.dt
			#next_state[2] = np.fmod(next_state[2],360) # Apply mod 360 function to theta to ensure 360 deg = 0 deg
			states.append(next_state)
            # Publish State
			publish_state = State()
			publish_state.x = next_state[0]
			publish_state.y = next_state[1]
			publish_state.theta = next_state[2]
			self.pub.publish(publish_state)
			rospy.loginfo(f"Publishing States: {publish_state}")
			rospy.sleep(self.dt)  # Sleep to control publishing rate
			if current_state[0] == self.initial_state[0] and current_state[1] == self.initial_state[1]:
				break # As the trajectory is a circle, if the car loops back on itself then finish simulation
		return np.array(states)



if __name__ == '__main__':
	try:
		node = SensorNode()
		states = node.integrate_dynamics_euler(dubins_car_dynamics)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

