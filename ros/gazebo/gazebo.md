# gazebo
gazebo is a simulation package in ros2, but maintained separately

## installation
- Before installation ensure version compatibility with ubuntu, ros2 and gazebo
- installation:https://gazebosim.org/docs/harmonic/install_ubuntu/
- latest version of gazebo is harmonics which is suitable for jazzy
- you can install gazebo harmonic from apt too
```bash
sudo apt-get install ros-jazzy-ros-gz 
```
- to verify all the packages installed with gazebo
```bash
ros2 pkg list | grep gz
```
### installation errors
```bash
# note
if there is a issue like you can`t run all gz command without using sudo, consider uninstalling and installing again.command to removing gazebo completely from our system is in installation link
```

## running gazebo harmonics
- use GUI or 
- using gazebo CLI (this will start gazebo server and GUI)
```bash
gz sim
```
- running gazebo with a custom world (using the same command we can run default worlds too)
```bash
gz sim <path of your world>
```
- launching gazebo with a custom robo model described by urdf file (not working as expected)
```bash
gz sim -v 4 -r <path of your robo>
```
## running gazebo using ros2
In ROS 2 Harmonic, the `ros_gz_sim` package provides integration between ROS 2 and Gazebo Sim. However, the `gz_sim` executable is not directly invoked as a ROS 2 node. Instead, the package offers a convenient launch file to start Gazebo Sim with specified arguments.
```bash
ros2 launch ros_gz_sim gz_sim.launch.py

#launch with specific sdf
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="path/to/your/world.sdf"
```

## notes
- world is the environment where we simulate our robot, this written as .sdf file (simulation description format). it is an XML-based format
- robots are described as .urdf file, this is also a XML based format. to run .urdf file in gazebo it should convert into sdf file
- some times the gazebo node will run even after closing GUI, you can check this by
```bash
ps aux | grep gz
```
if there is an unwanted process ,kill it
```bash
kill -9 <PID>
```
## generating robot urdf or xacro
- tutorial for urdf: https://github.com/ros/urdf_tutorial/tree/ros1
- gazebo coordinate system
    - red: x axis
    - blue: z axis 
    - green: y axis
- gazebo as well as ros is using SI units (meter,kg,sec)
- angles are in radian

