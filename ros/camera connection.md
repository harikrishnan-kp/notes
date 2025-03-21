Connect the Camera in raspberry pi and visualise in rviz

Install the camera driver packages. We use i ball 5G lens digital zoom night vision camera, for this camera we need v4I2-camera and usb-cam driver packages. 
Note: If we are using the realsense camera (binocular depth camera), then we need realsense2-camera and usb-cam driver packages. 
	sudo apt install ros-jazzy-v4l2-camera
	sudo apt install ros-jazzy-usb-cam 
	sudo apt install ros-jazzy-realsense2-camera
Connect the camera to the system USB
Then we want to see the list of devices connected 
v4l2-ctl --list-devices
 The output would look something like below. 
USB 2.0 Camera: USB Camera (usb-0000:00:14.0-3):
	/dev/video0
	/dev/video1
	/dev/media0
To run the camera node in the terminal
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
To launch the rviz2, give the below command in another terminal 
	ros2 run rviz2 rviz2
Then add the camera topic in the display.

If the below error occurs when you connect the camera to the raspberry pi
Error:- pi4@pi4-desktop:~$ v4l2-ctl --list-devices Failed to open /dev/video0: Permission denied
Solution:- Enter the command the in the terminal 
sudo usermod -aG video $USER
    Then reboot the system and check again 
v4l2-ctl --list-devices
    Now we can see the list of devices connected.


