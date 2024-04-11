# ROS2 Workshop
This repo provides a walk-through for the ROS2 Workshop. The slides for the workshop can be found [here](https://docs.google.com/presentation/d/16l8FBTwdDo79FANmzRIPsMCFvNq-8jPfQ0DPoVmW-qQ/edit?usp=sharing).

These instructions are designed for someone with access to the Isengard server on the Colorado School of Mines campus but can also be used with a locally installed version of ROS2. Following these steps should set one up to use ROS2 on Isengard with Gazebo. If you would like to run ROS locally, then please follow the [install instructions](https://docs.ros.org/en/foxy/Installation.html) from ros.org. Note that we are using ROS 2 Foxy Fitzroy on Ubuntu 20.04. 

## Setting-up Basic ROS2 Workspace
We will first create a workspace. Everything in ROS2 is done in what is called a workspace. A workspace is a directory (folder) where we store source code, build ROS2 packages, and run ROS2 nodes.

If you are not using Isengard, then skip to **Make a new workspace**

### Accessing Isengard
To use ROS2 on Isengard you will need to ssh into Isengard. This can be done from a computer in CTLM B60 or on any machine that is connected to the Mines WiFi.

```
ssh -Y <your-login>@isengard.mines.edu
```

(Optional) Start byobu to make running multiple terminals in Isengard easier.

```
byobu
```

In byobu, you can open new tabs using F2. Cycle through tabs using F3/4. To close a tab or exit byobu, run `exit`.

### Make a new workspace
Let's create a new ROS2 workspace in the user's home directory.

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Checkout ROS2 Packages
Pull this repository into the src folder. The repository has a few already-created ROS2 packages.

```
git clone https://github.com/JonD07/ros2_tutorial.git src/ros2_tutorial
```

Packages are where we store source code for nodes. Take a minute to explore the file structure of the `basic_tutorial` package, located at `~/ros2_ws/src/ros2_tutorial/basic_tutorial`. Notable elements of the package include:

- `basic_tutorial/` - Folder (with the same name as the package) that holds python code for nodes
- `package.xml` - Package description and dependencies
- `setup.cfg` - Build/Install configurations (mostly boiler plate stuff)
- `setup.py` - More package information, install instructions, and list of package nodes

### Building
To build ROS2 packages, we first need to source the basic ROS2 tools that are installed on Isengard.

```
source /opt/ros/foxy/setup.bash
```

We build ROS2 packages using a build tool called colcon. To run colcon and build all of the packages that we just downloaded into our workspace, run:

```
colcon build
```

## Running Basic ROS2 Nodes
When colcon has finished building and installing all of our ROS2 packages, we can experiment with some basic nodes that demonstrate how both topics and services work. This repository contains code examples for creating publishers, subscribers, service servers, and service clients. The details on how to implement all of these from scratch can be found in the [ROS2 Foxy tutorials](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries.html).

First, we need to source the packages that we just built in our workspace so that ROS2 knows where to find them. 

```
source install/setup.bash
```

### Running basis Publisher/Subscriber
This repository contains both a basic publisher and subscriber node. From the root of our workspace, the source code for these nodes is located in `src/ros2_tutorial/basic_tutorial/basic_tutorial/`, where you will find `publisher_member_function.py` and `subscriber_member_function.py`.

To run the topic publisher node, run the following:

```
ros2 run basic_tutorial talker
```

This node will publish messages to the `/chatter` topic. The message type is `std_msgs/msg/String`. I will show you how to find out all of this information shortly.

Let's launch a subscriber node that will receive the `std_msgs/msg/String` messages on the `/chatter` topic from the `talker` node. In a new tab (F2 if using byobu), source the workspace and start up the subscriber. 

```
source install/setup.bash
ros2 run basic_tutorial listener
```

You should now see the messages coming from `talker`. To stop the `listener` node, use Ctrl+C.

#### Useful ROS2 Commands
The ROS2 tool provides a variety of commands that are very helpful. From the same tab that you ran the `listener` node in and with the `talker` still running, let's explore what else ROS2 can do.

To view all ROS2 commands, run:

```
ros2 --help
```

You will see all of the various commands that ROS2 takes. Let's start by looking at nodes.

```
ros2 node --help
```

You will see a variety of sub-commands that the `node` command can take — the `list` sub-command, which will list all nodes currently running.

```
ros2 node list
```

You will see our publisher node listed, `/minimal_publisher`. This is the actual name of our `talker` node. Note that the node is not named `talker`. This is because `talker` is the name of our entry point, which creates a node named `/minimal_publisher` and could have created more than one node. More details on this can be found in the ROS tutorials.

Let's get more info on the node.

```
ros2 node info /minimal_publisher
```

You will see all of the topics that the node publishes and subscribes to and all of the services the node provides. Take a minute to explore other ROS2 commands. Here are examples of a few that are usually helpful:

```
ros2 topic list
```

```
ros2 topic info /chatter
```

Move back to the tab with the publisher node (F3) and stop it using Ctrl+C.

### Running basis Service/Client
This repository also contains a basic service node and a client node. From the root of our workspace, the source code for these nodes is located in `src/ros2_tutorial/basic_tutorial/basic_tutorial/`, where you will find `service_member_function.py` and `client_member_function.py`. To run these:

```
ros2 run basic_tutorial service
```
 
Move to the next tab (F4) and start the client.

```
ros2 run basic_tutorial client 2 3
```

## Using Gazebo
Gazebo is a robot simulator that works with ROS2. Gazebo is already installed on Isengard and can easily be installed on other machines following these [instructions](https://classic.gazebosim.org/tutorials?tut=ros2_installing).

To use Gazebo on Isengard, you will need several supporting packages. On most machines, we can get new ROS packages using `apt`. To download a new package, you simply run:

`sudo apt install ros-foxy-<some-neat-package>`

However, we do not have sudo privileges on Isengard, so we need to checkout packages from source into our workspace. Clone the following into your workspace:

**Note**: You do not need to manually install packages on a machine that you have administrative access to! Manually the following is only required to work on Isengard. ROS2 desktop version already comes with most of these packages. 

```
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git src/gazebo_ros_pkgs -b foxy
git clone https://github.com/ros-perception/image_common.git src/image_common -b foxy
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv -b foxy
git clone https://github.com/ros2/rviz.git src/rviz -b foxy
git clone https://github.com/ros/resource_retriever.git src/resource_retriever -b foxy
git clone https://github.com/ros-visualization/interactive_markers.git src/interactive_markers -b foxy
git clone https://github.com/ros-perception/laser_geometry.git src/laser_geometry -b foxy
git clone https://github.com/ros-planning/navigation_msgs.git src/nav_msgs -b foxy
```

Build all of the new packages.

```
colcon build
```

This may take some time... Now is a good time to go through the workshop slides and learn more about how ROS2 works.

Once complete, be sure to source the install again.

```
source install/setup.bash
```

### Launch a basic robot
Launch the demo robot in a warehouse that comes with this repository. More information on how to build robots and Gazebo environments can be found at this [helpful tutorial}(https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/) by Automatic Addison.

```
gazebo --verbose src/ros2_tutorial/worlds/warehouse_track.world
```

You should see Gazebo launch with a small warehouse robot sitting in a rustic-looking racetrack. Note that you can find other Gazebo worlds in `install/gazebo_plugins/share/gazebo_plugins/worlds/`.

In a different terminal, try driving the robot.

```
ros2 topic pub /demo/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}' -1
```

You can also watch the lidar scans from the robot get published. Run the following on a different terminal tab:

```
ros2 topic echo /demo/laser/out
```

Stop both the topic echo and the simulation with Ctrl+C

We will now use a basic reactive controller to drive our robot around the warehouse race track in the simulator. To do this we will use a launch file to run multiple nodes at once. First, launch the simulation again.

```
gazebo --verbose src/ros2_tutorial/worlds/warehouse_track.world
```

The launch file and reactive controller are located in the `src/ros2_tutorial/basic_tutorial/robot_control/` package. The controller uses an algorithm called "Follow the gap", which takes in lidar scans, searches for the largest consecutive distance measurements that are longer than a preset value (the largest "gap"), and then steers the robot towards the center of the gap. The launch file will launch our follow-the-gap node, a rviz2 node, and some static transforms. To learn more about launch files, head over the the [intermediate ROS2 tutorials](https://docs.ros.org/en/foxy/Tutorials/Intermediate.html).

Launch our follow-the-gap launch file in a new tab:

```
ros2 launch robot_control follow_gap_launch.py
```
