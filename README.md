# crazyflie_ros2_multiranger
Based on the original multiranger node by knmcguire
This repository contains different ROS 2 nodes to interact with the multiranger on the Crazyflie for both simulation as the real Crazyflie.


## Installation

Start a workspace:
```bash
mkdir ~/crazyflie_mapping_demo
cd crazyflie_mapping_demo
mkdir simulation_ws
mkdir ros2_ws
cd ros2_ws
mkdir src
```
Clone the repositories in their right location, starting with simulation

```bash
cd ~/crazyflie_mapping_demo/simulation_ws
git clone https://github.com/bitcraze/crazyflie-simulation.git
```
Then navigate to the ROS2 workspace source folder and clone 3 projects:
```bash
cd ~/crazyflie_mapping_demo/ros2_ws/src
git clone https://github.com/SpyguyOO7/crazyflie_ros2_multiranger_wind_sensor.git
git clone https://github.com/knmcguire/ros_gz_crazyflie
git clone https://github.com/SpyguyOO7/crazyswarm2_wind_sensor.git --recursive
```

Then build the workspace:
```bash
cd  ~/crazyflie_mapping_demo/ros2_ws/
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
```

Building will take a few minutes. Especially Crazyswarm2 will show a lot of warnings and std_err, but unless the package build has ‘failed’, just ignore it for now until we have proposed a fix to that repository.

If the build of all the packages passes and non failed, please continue to the next step!

## Usage

Every terminal were you run the examples in needs to have the setup.bash sourced with:
```bash
source ~/ros2_ws/install/setup.bash
```

Also the simulation model needs to be sourced in every terminal where you run the simulation with:
```bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
```


### Simulated Crazyflie with simple mapper, wall following, and wind measurements

To run the simulation with the simple mapper while it is autonomously wall following run:

```bash
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py
```

You don't have to control the  simulated Crazyflie as it is autonmously wallfollowing. You can see the map being created in Rviz.

You can make the simulated Crazyflie stop with calling the following service:
```bash
ros2 service call /crazyflie/stop_wall_following std_srvs/srv/Trigger
```

It will now stop moving and land.

### Real Crazyflie with simple mapper, wall following, and wind measurements

Go to crazyflie_ros2_multiranger_bringup/config/  and edit the crazyflie_real_crazyswarm2.yaml file to set the uri of the Crazyflie to the correct one.

Then run:
```bash
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_real.launch.py
```

You don't have to control the  simulated Crazyflie as it is autonomously wall following. You can see the map being created in Rviz.

You can make the simulated Crazyflie stop with calling the following service:
```bash
ros2 service call /crazyflie_real/stop_wall_following std_srvs/srv/Trigger
```

The crazyflie will now stop moving and land.
