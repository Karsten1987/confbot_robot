# How to run the demo

## Prerequisites

- Have ROS 2 installed, here assumed ROS Crystal located at `/opt/ros/crystal`
- ROS 2 development tools (`sudo apt install python3-colcon-common-extensions python3-rosdep`)

## Getting and building the code

```bash
export CONFBOT_WS=$HOME/confbot_ws
mkdir -p $CONFBOT_WS/src && cd $CONFBOT_WS && git clone https://github.com/Karsten1987/roscon2018 src/roscon2018
```

### Adapting the code for your machine
```bash
cd $CONFBOT_WS
sed -i s@/Users/karsten/workspace/osrf/roscon2018_ws/@$CONFBOT_WS/@g `grep -lr "/karsten/"  --exclude README.md`
```

### Installing the dependencies
```bash
sudo rosdep init
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro crystal
```

### Source crystal environment
```bash
source /opt/ros/crystal/setup.bash
```

### Build the workspace
```bash
colcon build --merge-install --cmake-args --no-warn-unused-cli -DSECURITY=ON
```

### Run the demo
In each terminal from now on, always assume use a sourced environment:
```bash
source $CONFBOT_WS/install/setup.bash
```

#### Use CLI to introspect and interact with the system

##### Launch the demo
Terminal 1:
```bash
ros2 launch confbot_bringup confbot_bringup.launch.py
```

##### Introspect the system
Terminal 2:
```bash
$ ros2 node list

/launch_ros
/robot_state_publisher
/confbot_driver
/twist_publisher
/confbot_laser
/safe_zone_publisher
```

```bash
$ ros2 topic list -t

/cmd_vel [geometry_msgs/Twist]
/confbot_laser/transition_event [lifecycle_msgs/TransitionEvent]
/danger_zone [visualization_msgs/Marker]
/joint_states [sensor_msgs/JointState]
/parameter_events [rcl_interfaces/ParameterEvent]
/robot_description [std_msgs/String]
/rosout [rcl_interfaces/Log]
/safe_zone [visualization_msgs/Marker]
/tf [tf2_msgs/TFMessage]
/tf_static [tf2_msgs/TFMessage]
```

```bash
$ ros2 topic echo /cmd_vel

linear:
  x: 0.10000000149011612
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.10000000149011612

linear:
  x: 0.10000000149011612
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.10000000149011612

^C
```


##### Visualize the data
Terminal 2:
```bash
ros2 run rviz2 rviz2 -d `ros2 pkg prefix confbot_bringup --share`/config/confbot.rviz
```

![RViz confbot_bringup](images/rviz_confbot_bringup.png)

##### Set parameters
Terminal 3:
The node `/twist_publisher` has a speed parameter.

```bash
$ ros2 param set /twist_publisher speed 0.2
Set parameter successful
```

In Terminal 1 we see:
```bash
[INFO] [twist_publisher]: set parameter 'speed' to "0.200000"
```

In RViz: robot goes twice as fast

Terminal 3:
```bash
$ ros2 param get /twist_publisher speed
Double value is: 0.2
$ ros2 param set /twist_publisher speed 0.1
Set parameter successful
$ ros2 param get /twist_publisher speed
Double value is: 0.1

```

##### Introspecting lifecycle nodes
Temrinal 3:
```bash
$ ros2 lifecycle nodes
/confbot_laser
```

```bash
$ ros2 lifecycle list /confbot_laser
- configure [1]
	Start: unconfigured
	Goal: configuring
- shutdown [5]
	Start: unconfigured
	Goal: shuttingdown
```

```bash
$ ros2 lifecycle set /confbot_laser 1
Transitioning successful
```

```bash
$ ros2 lifecycle list /confbot_laser
- cleanup [2]
	Start: inactive
	Goal: cleaningup
- activate [3]
	Start: inactive
	Goal: activating
- shutdown [6]
	Start: inactive
	Goal: shuttingdown
```

```bash
$ ros2 lifecycle set /confbot_laser 3
```
In RViz: the laser node is now publishing fake laser data!

![RViz with laser](images/rviz_with_laser.png)

##### Using components

Terminal 3:
Listing running components:
```bash
$ ros2 component list
/confbot_driver_container
  1  /confbot_driver
  2  /twist_publisher
```

Unload the `twist_publisher` component:
```bash
$ ros2 component unload /confbot_driver_container 2
Unloaded component 2 from '/confbot_driver_container' container node
```
In RViz the robot stops moving

List available components:
```bash
$ ros2 component types
confbot_driver
  confbot_driver::nodes::ConfbotDriver
  confbot_driver::nodes::TwistPublisher
```

Load a component:
```bash
$ ros2 component load /confbot_driver_container confbot_driver confbot_driver::nodes::TwistPublisher
Loaded component 3 into '/confbot_driver_container' container node as '/twist_publisher'
```
Robot starts moving again in RViz

```bash
$ ros2 component list
/confbot_driver_container
  1  /confbot_driver
  3  /twist_publisher
```


##### Compromise the system

Terminal 3:
Tempering with the system:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0}" -r 100
```
Robot stops moving


#### Use ROS 2 security

We will now use ROS2 Security to harden the system.

Prerequisite: Terminate all the running nodes by Ctrl+C them (Terminals 1, 2 and 3).


##### Create security artifacts

```bash
export ROS_SECURITY_ROOT_DIRECTORY=$CONFBOT_WS/confbot_keystore
cd $CONFBOT_WS
colcon build --merge-install --cmake-args -DSECURITY=ON -DPOLICY_FILE=$CONFBOT_WS/src/roscon2018/confbot_security/safe_zone_publisher_policies.xml --packages-select confbot_security --cmake-force-configure
```

The `confbot_keystore` directory content will look similar to:

```bash
/tmp/confbot_ws/confbot_keystore
├── 1000.pem
├── 1001.pem
├── 1002.pem
├── 1003.pem
├── 1004.pem
├── 1005.pem
├── 1006.pem
├── ca.cert.pem
├── ca_conf.cnf
├── ca.key.pem
├── confbot_driver
│   ├── cert.pem
│   ├── ecdsaparam
│   ├── governance.p7s
│   ├── identity_ca.cert.pem
│   ├── key.pem
│   ├── permissions_ca.cert.pem
│   ├── permissions.p7s
│   ├── permissions.xml
│   ├── req.pem
│   └── request.cnf
├── confbot_laser
│   ├── cert.pem
│   ├── ecdsaparam
│   ├── governance.p7s
│   ├── identity_ca.cert.pem
│   ├── key.pem
│   ├── permissions_ca.cert.pem
│   ├── permissions.p7s
│   ├── permissions.xml
│   ├── req.pem
│   └── request.cnf
├── ecdsaparam
├── governance.p7s
├── governance.xml
├── index.txt
├── index.txt.attr
├── index.txt.attr.old
├── index.txt.old
├── launch_ros
│   ├── cert.pem
│   ├── ecdsaparam
│   ├── governance.p7s
│   ├── identity_ca.cert.pem
│   ├── key.pem
│   ├── permissions_ca.cert.pem
│   ├── permissions.p7s
│   ├── permissions.xml
│   ├── req.pem
│   └── request.cnf
├── robot_state_publisher
│   ├── cert.pem
│   ├── ecdsaparam
│   ├── governance.p7s
│   ├── identity_ca.cert.pem
│   ├── key.pem
│   ├── permissions_ca.cert.pem
│   ├── permissions.p7s
│   ├── permissions.xml
│   ├── req.pem
│   └── request.cnf
├── rviz
│   ├── cert.pem
│   ├── ecdsaparam
│   ├── governance.p7s
│   ├── identity_ca.cert.pem
│   ├── key.pem
│   ├── permissions_ca.cert.pem
│   ├── permissions.p7s
│   ├── permissions.xml
│   ├── req.pem
│   └── request.cnf
├── safe_zone_publisher
│   ├── cert.pem
│   ├── ecdsaparam
│   ├── governance.p7s
│   ├── identity_ca.cert.pem
│   ├── key.pem
│   ├── permissions_ca.cert.pem
│   ├── permissions.p7s
│   ├── permissions.xml
│   ├── req.pem
│   └── request.cnf
├── serial
├── serial.old
└── twist_publisher
    ├── cert.pem
    ├── ecdsaparam
    ├── governance.p7s
    ├── identity_ca.cert.pem
    ├── key.pem
    ├── permissions_ca.cert.pem
    ├── permissions.p7s
    ├── permissions.xml
    ├── req.pem
    └── request.cnf
```

This means only the following nodes have been allowed to be used in a secured environment:
 - confbot_driver
 - confbot_laser
 - launch_ros
 - robot_state_publisher
 - rviz
 - twist_publisher
 - safe_zone_publisher

##### Launch the demo in secure mode

All terminals are assumed to have the environment setup and security environment variables set
```bash
source $CONFBOT_WS/install/setup.bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_ROOT_DIRECTORY=$CONFBOT_WS/confbot_keystore
```

Terminal 1: launch the system
```bash
ros2 launch confbot_bringup confbot_bringup_activated.launch.py
```

Terminal 2: visualize the system
```bash
ros2 run rviz2 rviz2 -d `ros2 pkg prefix confbot_bringup --share`/config/confbot.rviz
```
In RViz we will see the robot with the laser points but no sharks.


![RViz no shark](images/rviz_no_safe_zone.png)

##### Attempt tempering with the system

```bash
$ ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0}" -r 100
Unknown error creating node: SECURITY ERROR: unable to find a folder matching the node name in the "ROS_SECURITY_NODE_DIRECTORY" or "ROS_SECURITY_ROOT_DIRECTORY" directories while the requested security strategy requires it, at /tmp/binarydeb/ros-crystal-rcl-0.6.5/src/rcl/node.c:391
```
This fails because the node used by `ros2 topic` has not been allowed to join the secured network

```bash
$ ros2 run confbot_driver twist_publisher __node:=my_hacky_node
terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
  what():  failed to initialize rcl node: SECURITY ERROR: unable to find a folder matching the node name in the "ROS_SECURITY_NODE_DIRECTORY" or "ROS_SECURITY_ROOT_DIRECTORY" directories while the requested security strategy requires it, at /tmp/binarydeb/ros-crystal-rcl-0.6.5/src/rcl/node.c:391
```

This fails because, while the node `/twist_publisher` is allowed to join the network, the node `/my_hacky_node` is not.

##### Restrict individual node permission

Let's now see how to specify what actions a given node is allowed to perform.
Here we will look at the `/safe_zone_publisher` node's permissions.

This node's sole function is to publish the safe zone where the robot is operating (green zone in RViz) and the danger zone (red shark).

Let's have a look at the `safe_zone_publisher_policies.xml` file
`cat $CONFBOT_WS/src/roscon2018/confbot_security/safe_zone_publisher_policies.xml`

![Node security policies](images/safe_zone_security_policies.png)

Imagine an attacker is able to inject code into our `/safe_zone_publisher` node to make it send velocity commands to the robot.

```bash
cd $CONFBOT_WS/src/roscon2018/confbot_tools/confbot_tools
cp safe_zone_publisher_hacked.py safe_zone_publisher.py
git diff safe_zone_publisher.py
```

![diff injected code](images/safe_zone_git_diff.png)

Let's try to run this compromised node:

```bash
$ python3 safe_zone_publisher.py

2019-03-26 15:53:06.721 [SECURITY Error] Error checking creation of local writer d0.9b.b9.b.37.3.bc.cc.79.5c.ec.3a|0.0.10.3 (rt/cmd_vel topic not found in allow rule. (/tmp/binarydeb/ros-crystal-fastrtps-1.7.0/src/cpp/security/accesscontrol/Permissions.cpp:1111))
```

Here it fails with `rt/cmd_vel topic not found in allow rule.` meaning that the not is not allowed to publish on the ROS topic `/cmd_vel`.
