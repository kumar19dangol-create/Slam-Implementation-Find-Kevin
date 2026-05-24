# Operation Find Kevin - Succulence Rover

This repository contains the ROS 2 package used for the Algorithmic Robotics Grand Challenge, **Operation Find Kevin**. The system integrates pose-graph SLAM, A* path planning, and path-following navigation for the Succulence rover in the Unity Mars simulation.

## Project Overview

The final system connects mapping, planning, and navigation into one complete ROS 2 pipeline.

Main ROS 2 nodes used in the final run:

- `/slam_node` - builds the SLAM map and corrected trajectory
- `/planner_node` - runs A* path planning on the SLAM occupancy grid
- `/navigator_node` - follows the planned path and sends velocity commands
- `/map_to_odom_publisher` - publishes the static transform between map and odometry
- `/base_to_lidar_publisher` - publishes the static transform between robot base and LiDAR

Main topics used:

| Topic | Message Type | Purpose |
|---|---|---|
| `/succulence/odom` | `nav_msgs/msg/Odometry` | Raw odometry from the rover |
| `/succulence/scan` | `sensor_msgs/msg/LaserScan` | LiDAR scan data |
| `/succulence/map` | `nav_msgs/msg/OccupancyGrid` | SLAM occupancy grid map |
| `/succulence/slam/odometry` | `nav_msgs/msg/Odometry` | Corrected SLAM odometry |
| `/succulence/slam/path` | `nav_msgs/msg/Path` | SLAM trajectory for RViz2 visualisation |
| `/succulence/plan` | `nav_msgs/msg/Path` | Planned A* path |
| `/succulence/plan/inflated` | `nav_msgs/msg/OccupancyGrid` | Inflated planning grid for obstacle safety |
| `/succulence/plan/reachable` | `nav_msgs/msg/OccupancyGrid` | Reachable planning grid |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command sent to the rover |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transform information |

## System Pipeline

The Unity Mars simulation publishes raw odometry and LiDAR scan data. The `/slam_node` uses these inputs to build a pose-graph SLAM map and publish corrected odometry. The `/planner_node` uses the SLAM map and corrected pose to generate an A* path. The `/navigator_node` follows the planned path and publishes `/cmd_vel` commands back to the simulated rover.

Basic flow:

```text
Unity Mars Simulation
    ↓ /succulence/odom, /succulence/scan

/slam_node
    ↓ /succulence/map, /succulence/slam/odometry

/planner_node
    ↓ /succulence/plan

/navigator_node
    ↓ /cmd_vel

Unity Mars Simulation
```
## Build Instructions For Simulation

Clone the Simulation docker first:
```bash
git clone https://github.com/CollaborativeRoboticsLab/algorithmic-robots-world.git
cd algorithmic-robots-world
```

Now, inside the 'algorithmic-robots-world' folder create these folders '/workspace/succulence_ws/src' and finally clone this ros2 package into the ros 2 workspace:

```bash
cd ~/algorithmic-robots-world/workspace/succulence_ws/src/
git clone https://github.com/kumar19dangol-create/Slam-Implementation-Find-Kevin
cd ..
```

## Launch Instructions

This package supports both the Unity Mars simulation and the physical TurtleBot setup. The simulation uses `params_sim.yaml`, while the physical robot uses `params_physical.yaml`.

### Simulation Launch

Start the Unity Mars simulation in the first terminal:

```bash
cd ~/algorithmic-robots-world
xhost +local:root
docker compose -f compose-simulation.yaml up
```

In the Unity simulation, Press 'R' to switch the rover to Autonomous mode before launching the mission in the second terminal.

Launch the full simulation pipeline/misson in a second terminal:

```bash
cd ~/algorithmic-robots-world/workspace/succulence_ws
colcon build --packages-select succulence_rover_ros --symlink-install
source install/setup.bash
ros2 launch succulence_rover_ros mission_sim.launch.py
```

Open preconfigured Rviz2 in a third terminal:

```bash
rviz2
```
Then inisde the rviz2 interface open 'file/open config', look for the folder 'config/succulance_slam.rviz'.

The simulation mission should run the full SLAM, A* planning, and navigation pipeline. The rover should build the SLAM map, publish a planned path on `/succulence/plan`, and start to drive to the point. 

### To stop the simulation 

```bash
docker compose -f compose-simulation.yaml stop
docker compose -f compose-simulation.yaml down
```

This stops the container neatly. 

### To stop the Mission and Rviz2 in the other terminals 

```bash
CTRL + C
```

### Physical Robot Launch

Before using the physical robot, make sure there are no other simualtion/docker container running in the background. To check: 

```bash
docker ps
```

If there is anything running then kill it. Then, Connect to the physical rover/TurtleBot network and make sure the required robot drivers are running. All the lights on the bot should be on. 
Ip address will be displayed on the robot's screen.
Login to your Physcial bot: 

```bash
ssh @theipaddressoftherobot
```

To run mission:

```bash
cd ~/algorithmic-robots-world/workspace/succulence_ws
colcon build --packages-select succulence_rover_ros --symlink-install
source install/setup.bash
ros2 launch succulence_rover_ros mission_physical.launch.py
```

The physical launch file uses the physical parameter file and starts with the `/reset_pose` service so the robot odometry begins from zero. After the reset returns successfully, the static TF publishers, SLAM node, planner node, and navigator node start.

Open preconfiguredRViz2 with the SLAM configuration:

```bash
rviz2 -d src/succulence_rover_ros/config/succulance_slam_physical.rviz
```

You will see your path to the goal and new area being discorved as your turtle bot moves through the area in real world. 
