# Operation Find Kevin: Autonomous SLAM and Navigation System

This repository contains the complete ros 2 autonomy stack developed for the Operation Find Kevin robotics challenge. The project focuses on autonomous mapping, localisation, path planning, and navigation for the Succulence rover operating in a simulated Martian environment.

The system enables the rover to explore an initially unknown environment using Lidar and wheel odometry, construct an occupancy-grid map using pose-graph Slam, estimate its corrected position, and autonomously navigate toward a target location using A* path planning and waypoint-following control.

The full robotics pipeline integrates sensing, localisation, mapping, planning, and motion control into a continuous real-time autonomy loop. As new sensor data is received, the rover continuously updates its map, corrects localisation drift, replans safe paths, and publishes velocity commands for navigation.

---

## Core System Components

### SLAM and Localisation
- Motion-model based dead reckoning
- Correlation-based Lidar scan matching
- Pose-graph optimisation using Gauss-Newton least squares
- Occupancy-grid map reconstruction
- Corrected Slam odometry estimation

### Autonomous Navigation
- A* shortest-path planning
- Obstacle inflation for collision avoidance
- Reachability analysis
- Waypoint-following navigation controller
- Continuous replanning during exploration

### ROS 2 Integration
- Modular Ros 2 node architecture
- Rviz2 visualisation support
- TF transform broadcasting
- Configurable simulation and physical robot parameters

---

## Main ROS 2 Nodes

| Node | Purpose |
|---|---|
| `/slam_node` | Full SLAM pipeline integrating localisation, scan matching, optimisation, and map reconstruction |
| `/planner_node` | Generates collision-safe A* paths from the Slam occupancy grid |
| `/navigator_node` | Follows planned trajectories and publishes rover velocity commands |
| `/map_to_odom_publisher` | Publishes the static transform between map and odometry frames |
| `/base_to_lidar_publisher` | Publishes the static transform between rover base and Lidar frame |

---

## Main ROS Topics

| Topic | Message Type | Description |
|---|---|---|
| `/succulence/odom` | `nav_msgs/msg/Odometry` | Raw rover odometry |
| `/succulence/scan` | `sensor_msgs/msg/LaserScan` | 2D Lidar scan data |
| `/succulence/map` | `nav_msgs/msg/OccupancyGrid` | Corrected Slam occupancy map |
| `/succulence/slam/odometry` | `nav_msgs/msg/Odometry` | Corrected Slam pose estimate |
| `/succulence/slam/path` | `nav_msgs/msg/Path` | Optimised Slam trajectory |
| `/succulence/plan` | `nav_msgs/msg/Path` | Planned A* navigation path |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Rover velocity commands |

---

## System Pipeline

```text
Lidar + Wheel Odometry
            ↓
        SLAM System
            ↓
 Corrected Map + Pose
            ↓
       A* Planner
            ↓
     Navigation Control
            ↓
        Rover Motion
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

In the Unity simulation, Press `R` to switch the rover to Autonomous mode before launching the mission in the second terminal.

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
Then inisde the rviz2 interface open `file/open config`, look for the file called `config/succulance_slam.rviz`.

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

If there is anything running then kill it. Then, Connect to the physical rover/TurtleBot network first and make sure the required robot drivers are running/all the led lights should be on. 
Then, edit the .env file and make sure to change the `ROBOT_ROS_DOMAIN_ID` and `ROBOT_ROS_DISCOVERY_SERVER` to match your Turtlbot's ID and its ip address. 
Robot ID will be on the bot and the Ip address will be displayed on the robot's screen. 
Finally, Login to your Physcial bot: 

```bash
ssh @theipaddressoftherobot
```
Now in a new terminal, To start the physcial bot environmet/initinalize:

```bash
cd ~/algorithmic-robots-world
xhost +local:root
docker compose -f compose-physical.yaml up
```
Open the broswer IDE at `http://127.0.0.1:8080`. This will open Web based Visual Studio that is connected to the robot aready. 

Now, To run mission, open new terminal inside that web-based VS:

```bash
cd ~/algorithmic-robots-world/workspace/succulence_ws
colcon build --packages-select succulence_rover_ros --symlink-install
source install/setup.bash
ros2 launch succulence_rover_ros mission_physical.launch.py
```

The physical launch file uses the physical parameter file and starts with the `/reset_pose` service so the robot odometry begins from zero. After the reset returns successfully, the static TF publishers, SLAM node, planner node, and navigator node start.

Open preconfigured Rviz2 with the SLAM configuration in that web-based VS:

```bash
rviz2 -d src/succulence_rover_ros/config/succulance_slam_physical.rviz
```

You will see your path to the goal and map being initialized and new area being discorved as well as the path to gaol being updated as your turtle bot moves through the area in real world. 

### To stop the Turtlebot and the rviz2

```bash
CTRL + C the misson running terminal in the web-based VS
```
Finally kill the docker container for the turtlebot being run in your main computer terminal just like you kill the siumualtion and also close the terminal where you logged into the turtlebot.

### Team Contributions: Group 3
| Team Member | Contribution |
|---|---|
| **Bishow** | I did all the parameter tuning and optimisation in `params_physical.yaml` for the physical rover during the final competition. Also contributed to testing, validation, and overall system performance improvements. |
| **Mac** | Assisted with running and monitoring the final ROS 2 SLAM and navigation system during the competition and helped coordinate execution of the rover pipeline. |
| **Nima** | Assisted with robot setup, integration, and preparation of the final system during the competition. |
| **Joe** | Assisted with robot setup, deployment, and support during the final testing and competition stages. |

All of us maintained our own simulation environments in our computers during development and testing. The final system integration and deployment for the competition were completed collaboratively as a team.
