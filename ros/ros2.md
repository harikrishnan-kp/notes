# ROS2
rose is a series of libraries, tools and frameworks for simplifying robot development
* in ros `node` is a unit of computation that do a specific task 
* nodes can be written in any language, but python and c++ is the default languages
* ROS nodes typically communicate through interfaces of one of three types: `topics`, `services`, or `actions`
* ros allows communication between node on the same system or different systems on the same network
* ROS 2 relies on the notion of combining workspaces using the shell environment. This allows the installation of several ROS 2 distributions on the same computer and switching between them.
* `Workspace` is a ROS term for the location on your system where you’re developing with ROS 2.
* The core ROS 2 workspace(built in) is called the `underlay`. Subsequent local workspaces(user created) are called `overlays`
* every package in ros2 contain a shell configuration file name `setup.bash`
* we need to source this file every time we use a shell
```bash
#syntax:
    source /opt/ros/<PackageName>/setup.bash
#eg:
    source /opt/ros/jazzy/setup.bash
```
or we can automate this by appending this sourcing command to .bashrc


## useful commands
* to run a node using in ros2
```bash
#syntax:
    ros2 run <PackageName> <NodeName>

#example: 
    ros2 run turtlesim turtlesim_node
```
* list all running nodes
```bash
ros2 node list
```
* list all topics
```bash
ros2 topic list
```
* list all services
```bash
ros2 service list
```
* directory for ros in ubuntu
```
/opt/ros
```
you can see all distributions(versions like jazzy, foxy etc) of ros installed in this directory

## reference links
* ros2 doc : https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html

## doubts 
- what is a processes in ros
- what is 'type' in ros ,it is coming in every node interfacing methods
- in environment setting
    - what is domain id 
    - what is automatic discovery range

## to learning index
- node
    - parameters of node
    - launching node
- node communications
    - topic    
    - service
    - action
- ros tools
    - `colcon` is the build tool in ros2
    - `Turtlesim` is a lightweight simulator for learning ROS 2
    - `rqt` is a GUI tool for ROS2
        - rqt graph
        - rqt console

Learning resources 
- https://www.youtube.com/@RobotLabs
- https://youtu.be/ZfPMXft3yoc?si=ylFvIfWd6yn1iLG-