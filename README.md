# ENGM020_DubinsCar
Submission for the Second Coursework Assignment for the 2024 ENGM020 Robotics and Automation Module at the University of Exeter

This repository contains a ROS1 noetic Python project designed for simulating a car model and visualizing it in Rviz. The project utilizes ROS messages, launch files, and nodes to facilitate the simulation and visualization process.

## Project Structure

- **launch/**: Contains ROS launch files.
- **msg/**: Contains custom ROS message definitions.
- **scripts/**: Contains Python scripts for ROS nodes.
- **rviz/**: Contains Rviz configuration files.

## Dependencies

Ensure you have ROS1 installed on your Ubuntu 20 system. Additionally, you'll need the following ROS packages:

- `ros-noetic-desktop-full`: ROS noetic Desktop Full installation.
- `ros-noetic-tf2`: ROS noetic TF2 library.
- `ros-noetic-rviz`: ROS noetic Rviz visualization tool.

## Building the Project

To build the project, follow these steps:

1. Create a catkin workspace (if you haven't already):

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make

2. Clone this repository into the `src` directory:

    ```bash
    cd ~/catkin_ws/src
    git clone <repository_url>

3. Build the workspace:

    ```bash
    cd ~/catkin_ws/
    catkin_make


## Running the Simulation

Once the project is built, you can run the simulation by executing the launch file `simulation.launch`:

    ```bash
    source devel/setup.bash # If you havent already
    roslaunch <package_name> simulation.launch


## General ROS System Layout

In this project, the general ROS system layout involves:

- A `tf_ros` fixed frame that publishes to the `/tf` topic for visualization in Rviz.
- The `/timer_data` node publishes three key simulation parameters:
  - Float64 simulation time interval to the topic `/simulation_dt`.
  - Int64 total number of time steps for the simulation to the topic `/simulation_num_steps`.
  - Boolean simulation ended flag sent through the topic `/simulation_ended`.

Here is a visualisation of this system using `rqt_graph`:
![alt text](<assets/graph.png>)
## Nodes and Topics

Two nodes subscribe to the aforementioned topics:

1. **sensor_data node**: This node generates the model for the car, applies a pure pursuit controller algorithm to control the model along a specific predetermined path, and terminates if a goal is achieved or if `simulation_ended` is `true`. It publishes a custom `State` message to the `/sensor_data` topic.
2. **sensor_data_processor node**: This node processes the `State` message published by the `sensor_data` node and publishes an Rviz marker for data visualization.

## Custom ROS Message

The project defines a custom ROS message named `State.msg` located in the `msg/` directory. This message is used for communicating the state information of the simulation.

## Notes

- Ensure that all necessary dependencies are installed before running the simulation.
- Customize the project parameters and settings as needed for your specific simulation scenario.
- To run the visualisation using RViz simply:
    ```bash
    cd ~/catkin_ws/
    source devel/setup.bash
    rviz