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

## ros2 msg recording (msg baging)
https://www.youtube.com/watch?v=a-O1qM9_S7k&t=556s

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
## Setting up virtual environment for ROS2 Python packages
Move to your package directory
```
cd <your_package_path>
```
Setup virtual environment
```
python3 -m venv .venv --system-site-packages --symlinks
```
The --system-site-packages flag allows the virtual environment to access system-wide Python packages.
The --symlinks flag ensures compatibility by using symbolic links for Python executables
Activate virtual environment
```
source .venv/bin/activate
```
Install dependencies
```
pip install <package_name>
```
Add the following lines to the setup.cfg of your package to use the virtual environment’s python.
```
[build_scripts]
executable = /usr/bin/env python3
```
Add the following shebang line at the starting of your node’s Python script
```
#!/usr/bin/env python3
```
Source the ros2 underlying workspace (Igonore if you already added this in .bashrc)
```
source /opt/ros/jazzy/setup.bash
```
Prevent colcon from building the virtual environment directory
```
touch .venv/COLCON_IGNORE
```
Move to the parent directory of your ros2 workspace 
```
cd ../../
```
Build package
```
colcon build --packages-select <your package name>
```
Source the ros2 overlay workspace
```
source install/setup.bash
```
Run package using the following command (or create a launch file)
```
ros2 run <package name> <node name>
```
Deactivate the virtual environment when execution is completed
```
deactivate
```

Learning resources 
- https://www.youtube.com/@RobotLabs
- https://youtu.be/ZfPMXft3yoc?si=ylFvIfWd6yn1iLG-

## ros2 organization pages in github
- https://github.com/ros
- https://github.com/ros2
- https://github.com/ros-navigation/
- https://github.com/ros-controls
- https://github.com/ros-planning
- https://github.com/ros-teleop
- https://github.com/ros-realtime
- https://github.com/ros-drivers
- https://github.com/ros-industrial
- https://github.com/ros-perception
- https://github.com/ros-visualization
- https://github.com/gazebosim
- https://github.com/ros-simulation
- https://github.com/ros-ai
  


