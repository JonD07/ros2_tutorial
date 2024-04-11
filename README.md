# ROS2 Workshop
This repo provides a walk-through for the ROS2 Workshop. These instructions are designed by someone with access to the Isengard server on the Colorado School of Mines campus. Following these steps should set one up to use ROS2 on Isengard with Gazebo.

## Setting-up Basic ROS2 Workspace
You will need to ssh into Isengard. This can be done from a computer in CTLM B60 or on any machine that is connected to the Mines WiFi.

### Accessing Isengard
```
ssh -Y <your-login>@isengard.mines.edu
```

(Optional) Start byobu to make running multiple terminals in Isengard easier.

```
byobu
```

In byobu, you can open new tabs using F2. Cycle through tabs using F3/4. To close a tab or exit byobu, run `exit`.

### Make a new workspace
Everything in ROS2 is done in what is called a workspace. A workspace is a directory (folder) where we store source code, build ROS2 packages, and run ROS2 nodes.

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Checkout ROS2 Packages
We will need a variety of ROS packages. Pull this repository into the src folder.

```
git clone https://github.com/JonD07/ros2_tutorial.git src/ros2_tutorial
```

(Optional) To use Gazebo on Isengard, you will also need to checkout the following:

```
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git src/gazebo_ros_pkgs -b foxy
git clone https://github.com/ros-perception/image_common.git src/image_common -b foxy
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv -b foxy
```

### Building
To build ROS2 packages, we first need to source the basic ROS2 tools that are installed on Isengard.

```
source /opt/ros/foxy/setup.bash
```

We build ROS2 packages using a build tool called colcon. To run colcon and build all of the packages that we just downloaded into our workspace, run:

```
colcon build
```

If you cloned the needed Gazebo repositories, building all of the packages will take some time. This is a good time to go through the workshop slides and learn more about how ROS2 works.

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
If you cloned the `gazebo_ros_pkgs` repository and its dependencies and successfully built all of their packages, then you can run the Gazebo simulator and use ROS2 to interact with robots within the simulator. Start by sourcing the workspace. 

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
ros2 topic pub /demo/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -1
```
