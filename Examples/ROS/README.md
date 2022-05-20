# Setting up ROS on Linux

1. Select a distribution and install it

    There is more than one stable ROS distribution. 
    Some are older releases with LTS, meaning they are more stable. Some
    are newer with shorter support lifetimes. 

    For Ubuntu 20.04, `Noetic Ninjemys` is recommended, with LTS until 2025. 
    Download + install link below, follow the instructions.

    ``` 
    http://wiki.ros.org/noetic/Installation/Ubuntu
    ```

2. Check installation 
    ```
    printenv | grep ROS
    ```
    You should see soemthing along the lines of  
    ```
    ROS_VERSION=1
    ROS_PYTHON_VERSION=3
    ROS_PACKAGE_PATH=/opt/ros/noetic/share
    ROSLISP_PACKAGE_DIRECTORIES=
    ROS_ETC_DIR=/opt/ros/noetic/etc/ros
    ROS_MASTER_URI=http://localhost:11311
    ROS_ROOT=/opt/ros/noetic/share/ros
    ROS_DISTRO=noetic
    ```

# Creating a ROS workspace 
Create workspace and overlay on current terminal 
```
mkdir -p {somewhere}/catkin_ws/src
cd {somewhere}/catkin_ws
catkin_make
source devel/setup.bash
```

Ensure workspace is active
```
echo $ROS_PACKAGE_PATH
```

# Navigating the ROS filesystem
Install tutorial package 
```
sudo apt-get install ros-noetic-tutorial
```

`Package`: Software organization unit of ROS code. Each package can contain libs, executables, scripts, or other artifacts.

`Manifest`: Description of a package. Defines dependencies between packages and to capture metadata about the package.  


Since code is spread across packages at varying disk locations, ros provides tools for easily navigating around packages. 


- `rospack`: Allows you to get info about packages. Ex. `rospack find [package_name]` to find the abs path to a package
- `roscd`: Allows you to change dirs. directly to a package. Can only access packages installed in locations specifed in $ROS_PACKAGE_PATH. `roscd log` will take you to the folder where ROS stores log files. 
- `rosls`: Like roscd, will ls a package contents using the package name instead of its path
- Tab completion for package names is also available when using these commands. 

# Creating a ROS Package 
What does a catkin package consist of?  
- package.xml: package metadata (manifest)
- CMakeLists.txt
- Each package must be in its own folder, no nested packages or sibling packages

It is best practice to create a catkin workspace, and then place all catkin packages inside that folder. Realize 
that every new terminal will need to "activate" the workspace, similar to how a python environment works. 

For convenience, add the following line to your bashrc to ensure the workspace is activated in every terminal.
```
source {somewhere}/catkin_ws/devel/setup.bash
```

Move to the workspace/src
```
cd {somewhere}/catkin_ws/src
```

Create a catkin package using the `catkin_create_pkg` script, which depends on std_msgs, roscpp, and rospy
```
catkin_create_pkg beginner_tutorials std_msgs roscpp rospy
```

```
# For example
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

catkin_create_pkg also has more advanced functionality. 

Now, the packages in the workspace must be built. 
```
cd {somewhere}/catkin_ws
catkin_make
```

## ROS Graph Concepts
`Node`: executable that uses ROS to communicate with other nodes

`Messages`: ROS data type used when publishing/subscribing to a topic 

`Topic`: Nodes can publish messages to a topic, and also subscribe to topics to recieve those messages   

`Master`: Name service for ROS (helps nodes find each other)

`rosout`: ROS equivalent of stdout/stderr

`roscore`: Master + rosout + parameter server

## Nodes
A node is an executable in a ROS package. Nodes use a ROS client library to communicate
with other nodes.

Nodes can also provide, or use a Service (?).

## Client Libraries
ROS provides the following client libraries:
- `roscpp`: C++ 
- `rospy`: Python

## roscore, rosnode, rosrun
These are the primary utilities for deploying and managing ROS nodes. 

Roscore starts the ros "runtime". 
```
roscore
```

Rosnode can be used to list the active nodes and view information about them.  
```
# List:  
rosnode list

# Details: rosnode info [node-name]
rosnode info /rosout
```

Rosrun is used to run a node. You do not need to know the path of the package and the node, just the names. 
```
# rosrun [package-name] [node-name]
rosrun turtlesim turtlesim_node
```

Node names can also be re-assigned.
```
rosrun turtlesim turtlesim_node __name:=my_turtle
```

## Topics 
Nodes can publish messages on a given topic. Other nodes can subscribe to that topic to receive those messages. 
ROS provides tools for debugging/viewing pub/sub relationships. 

`rqt_graph`: GUI for visualising registered pubs/subs for all topics 
```
sudo apt-get install ros-noetic-rqt
sudo apt-get install ros-noetic-rqt-common-plugins
rosrun rqt_graph rqt_graph
```
`rostopic`: Command line tool for viewing information about active topics 
```
# View all available commands
rostopic -h

# List all topics currently in use
rostopic list

# Display data published on a topic 
rostopic echo [topic]

# View the message type of a topic 
rostopic type [topic]

# View the rate at which data is published
rostopic hz [topic]

# View the bandwidth a topic is using
rostopic bw [topic]

# Use rosmsg to view details about that type (each field)
rosmsg show [message-type]

# Publish messages to topics from the command line (most basic use)
rostopic pub [topic] [payload]

```

## Services
Services are another way that ROS nodes can communicate. Services allow nodes to send a request 
and receive a response (like an HTTP req.).

```
rosservice list         print information about active services
rosservice call         call the service with the provided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri
```

`rosservice`: Like the rostopic pub command, can interact with services from the command line. 
```
# View all services 
rosservice list

# Get the type of a service
rosservice type [service]

# Get more details about the params required for a service call (fields)
rosservice type [service] | rossrv show

# Make a service call
rosservice call [service] [tags (payload)]
```

## Parameters
`rosparam`: Allows you to store and manipulate data on the "Parameter Server", using YAML syntax. 

Supported types: int, float, boolean, dictionary, list

Straight forward and very analogous to the previous commands
```
rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           dump parameters to file
rosparam delete         delete parameter
rosparam list           list parameter names
```

## rqt_console
Console for interacting with logs generated by any and all ros nodes. Can also the log level
for individual nodes, export logs, import logs, and apply filteres to parse logs. 

Install:
```
sudo apt-get install ros-<distro>-rqt ros-<distro>-rqt-common-plugins
```

Run: 
```
rosrun rqt_console rqt_console
```

## roslaunch 
Roslaunch is a tool used to start nodes as configured in a `launch` file. 

```
roslaunch [package] [filename.launch]
```

Note: It is best practice to store launch files in a folder called `launch`, although this 
is not required. Roslaunch will search for and use any launchfiles in a package. 

Example launch file below, use rqt_graph tool to view/better understand the topic configuration
generated by this launch file. 
```
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```

## rosed
Rosed allows you to edit files within packages with 0 knowledge about the location of the 
package or the file. Interestingly, this command supports tab auto complete for the filename 
(and package name as seen previously with rospack). This i s a deadly way to view all files in
a package without caring about their location at all. 

```
# rosed [package-name][file-name]
rosed roscpp Logger.msg
```

## msg, srv
**`Msg`**: Text file describing the fields of a ROS message. Used to generate source code. Stored in the `msg` directory of a package.
 
**`Srv`**: Text file describing a service. Composed of a request and a response. Stored in the `srv` directory of a package. 

Field Types:
- int8, int16, int32, int64 (plus uint*)
- float32, float64
- string
- time, duration
- other msg files (cool!)
- variable-length array[] and fixed-length array[C]
- Header (special ROS type with a timestamp and coordinate frame. Many msgs will start with a header)

Src format, seperate request from response with ---.
```
# Request
int64 A
int64 B
---
# Response
int64 Sum
```

## rosdep
To check where packages can be found: 
```
rosdep resolve --rosdistro=noetic ros_canopen
```

Install dependencies of all packages in {folder} that are not installed:
```
rosdep install --from-paths {folder} --ignore-src -r -y
```
