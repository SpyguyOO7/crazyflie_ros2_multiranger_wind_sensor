# crazyflie_ros2_multiranger_wind_sensor

Based on the original multiranger node by knmcguire.

This repository contains different ROS 2 nodes to interact with the multiranger on the Crazyflie for both simulation and the real Crazyflie, along with wind mapping.

> Disclaimer: This project is experimental. Please start a ticket if you encounter any issues.

## Installation (ROS 2 Jazzy)

First, install the required dependencies:
```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get install ros-jazzy-motion-capture-tracking ros-jazzy-tf-transformations
sudo apt-get install ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard
```

Start a workspace:
```bash
mkdir -p ~/crazyflie_mapping_demo
cd ~/crazyflie_mapping_demo
mkdir simulation_ws
mkdir -p ros2_ws/src
```

Clone the repositories into their correct locations, starting with the simulation models:
```bash
cd ~/crazyflie_mapping_demo/simulation_ws
git clone [https://github.com/bitcraze/crazyflie-simulation.git](https://github.com/bitcraze/crazyflie-simulation.git)
# Switch to the correct branch for multiranger models
cd crazyflie-simulation
git checkout gazebo-multiranger
```

Then navigate to the ROS 2 workspace source folder and clone the 3 required projects:
```bash
cd ~/crazyflie_mapping_demo/ros2_ws/src
git clone [https://github.com/SpyguyOO7/crazyflie_ros2_multiranger_wind_sensor.git](https://github.com/Spyguy007/crazyflie_ros2_multiranger_wind_sensor.git)
git clone [https://github.com/knmcguire/ros_gz_crazyflie.git](https://github.com/knmcguire/ros_gz_crazyflie.git)
git clone [https://github.com/SpyguyOO7/crazyswarm2_wind_sensor.git](https://github.com/Spyguy007/crazyswarm2_wind_sensor.git) --recursive
```

Then build the workspace:
```bash
cd ~/crazyflie_mapping_demo/ros2_ws/
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
```

Building will take a few minutes. `crazyswarm2_wind_sensor` may show warnings and `stderr` output; unless the package build explicitly says 'failed', you can ignore them.

If the build passes, continue to the next step.

## Usage

Every terminal where you run the examples needs to have your built workspace sourced:
```bash
source ~/crazyflie_mapping_demo/ros2_ws/install/setup.bash
```

Also, the simulation model needs to be sourced in every terminal where you run the simulation:
```bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
```

### Simulated Crazyflie with simple mapper, wall following, and wind measurements

To run the simulation with the simple mapper while it autonomously follows walls, run:
```bash
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py
```

You don't have to control the simulated Crazyflie as it is autonomously wall following. You can see the map being created in RViz.

You can make the simulated Crazyflie stop by calling the following service:
```bash
ros2 service call /crazyflie/stop_wall_following std_srvs/srv/Trigger
```
It will now stop moving and land.

### Real Crazyflie with simple mapper, wall following, and wind measurements

Go to `crazyflie_ros2_multiranger_bringup/config/` and edit the `crazyflie_real_crazyswarm2.yaml` file to set the URI of the Crazyflie to your drone's specific address.

Then run:
```bash
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_real.launch.py
```

You don't have to control the real Crazyflie as it is autonomously wall following. You can see the map being created in RViz.

You can make the real Crazyflie stop by calling the following service:
```bash
ros2 service call /crazyflie_real/stop_wall_following std_srvs/srv/Trigger
```
The Crazyflie will now stop moving and land.
