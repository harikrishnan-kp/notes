# Stereo Vision
## Aim
- for objects depth calculation and point cloud creation from stereo camera images
## Work flow
- use two identical cameras to get images
- run both cameras as ros2 node, so that it will publish images to a topic
- create another node to subscribe images from the topics and find disparity map. then publish this disparity map to another topic
- create another node to construct depth map from disparity map
- create another node to construct point cloud (3d view) from depth map

## depth calculation
- stereo vision method requires two identical cameras with parallel optical axis and both cameras should be in the same plane
- The origin of the camera's coordinate system is at the `optical center` of the camera's lens
- the distance between optical centers of the camera is called as `baseline`,ie baseline is the distance between centers of two camera lenses
- `disparity` is the horizontal displacement between corresponding pixels in the stereo image pair
- disparity increases when object is closer to the camera
- the map obtaining from disparity calculation is called as disparity map. it is a single grayscale image
- using the disparity information we can calculate the depth of objects,ie how far the object are from our camera plane.
    - depth = (focal length * baseline)/ disparity
        - where depth and baseline is in meter and focal length is in pixels (need to verify)
- this depth map is the output of depth calculation. which is a grayscale image.in this map object near to the camera plane are brighter

## opencv 
- disparity calculation algorithms in opencv
    - stereoBM (block matching)
    - stereoSGBM
- algorithms like StereoBM or StereoSGBM, the computed disparity values are stored in a fixed-point format where the actual disparity is multiplied by 16 to get actual disparity value for further calculations , we need need to scale it down by (1/16)
- cv::normalize(_src, dst, 0, 255, NORM_MINMAX, CV_8UC1);
    - When the normType is NORM_MINMAX, cv::normalize normalizes _src in such a way that the min value of dst is alpha and max value of dst is beta.
    - cv::normalize does its magic using only scales and shifts (i.e. adding constants and multiplying by constants).
    - how normalization in opencv works with cv::NORM_MINMAX flag (need to verify)
    ```
    normalized_value = ( ((original_value−min_original)×(β−α)​)/(max_original−min_original) ) + α
    ```
    - original_value is a value from your original disparity map.
    - min_original and max_original are the minimum and maximum disparity values in the original disparity map.
    - cv::NORM_MINMAX flag, the function automatically computes the min and max values of the source array
    - alpha and beta are the desired minimum and maximum values for the normalized output.
    - reversing normalization
    ```
    original_value = ( (normalized_value−α)×(max_original−min_original)/(β−α) ) ​+ min_original
    ```
- noises in image processing 
    - speckle noise
- if the img output is not good as expected, we can use filtering algorithms
    - WLS filter
    - box filter
    - gaussian blur

### stereoBM
- StereoBM computes disparity by comparing `image blocks` between stereo image pairs
- parameter needed to be set for disparity computation
    - `numDisparities`:
        - it is the max value of disparity, algorithm can compute.this parameter define the range of disparity calculation.
        - expressed in pixels
        - Must be a multiple of 16 for efficiency in OpenCV (this is how the algorithm works)
        - eg: If numDisparities = 64, it means the algorithm will search for matches up to 64 pixels apart.
        - Set numDisparities based on the expected depth range in your scene.
    - `Blocksize`:
        - Defines the size of the square block (kernel) used for matching between left and right images.
        - it is expressed in pixels.
        - Must be an odd number (e.g., 5, 9, 15, 21, etc.).
        - eg: blocksize = 15 means block is a square of 15*15 size 
        - Affects the accuracy and smoothness of the disparity map.      
- cv::StereoBM class calculates a disparity map, which represents the horizontal displacement between corresponding pixels in a stereo image pair
- The output disparity map can be of type CV_16S (16-bit signed) or CV_32F (32-bit float)
- When the output type is CV_16S, the disparity values are scaled by 16, meaning each disparity value in the map is 16 times the actual disparity
- stereoBM algorithm can't really provide an accuracy higher than 1/16px


## optics
- focal length it is the distance between point of convergance and sensor of camera (or distance between canter of lense and sensor).is measured in mm
- Determines the camera's field of view and the scale of the image.
- relation with field of view: field of view decreases with increase in focal length
- we can find focal length from field of view if we know width of our sensor
- for depth calculations we need focal length in pixels.usually cameras have square pixels, leading to equal focal lengths in both directions (fx = fy)

## camera parameters 
camera parameters are broadly classified into two 
    - intrinsic 
    - extrinsic
- extrinsics: Extrinsic camera parameters define the position and orientation of a camera with respect to world coordinate system. which include the following details
    - x,y,z,alpha,beta,gama
- intrinsics: these are the parameters of a camera that define how a 3D objects is convert to 2D images.they are 
    - `fₓ` : focal length expressed in terms of pixels along x direction (horizontal)
        - focal length in mm = fx * width of pixel
        - width of pixel is equal to horizontal distance between center of two pixels as there is no gap
    - `fᵧ`: focal length expressed in terms of pixels along y direction (vertical)
        - focal length in mm = fx * height of pixel
        - height of pixel is equal to vertical distance between center of two pixels as there is no gap
    - Principal Points (`cₓ, cᵧ`): Represents the coordinates of the optical center of the image sensor
        - typically near the center of the image.​
        - example for a picture of 640*480 resolution, cₓ = 320 and cᵧ = 240
    - Skew Coefficient (`γ`): Describes the angle between the x and y axes of the sensor; in most cameras, this is zero, indicating perpendicular axes.​
    - Pixel Aspect Ratio: The ratio of the width to the height of a pixel, usually equal to 1 for square pixels.

## camera calibration
- why we need camera calibration: to get camera parameters.this camera parameters are required for error correction and depth calculations
https://www.youtube.com/watch?v=H5qbRTikxI4
https://www.youtube.com/watch?v=Wcnb197g2i0
https://www.youtube.com/watch?v=A3nw2M47K50
- add camera paramters in yaml format in the directory /home/hari/.ros/camera_info, check terminal while running the camera

## try to understand
- how msg format "sensor_msgs::msg::Image" works,is there any alternatives msg formats to send disparity img in ros
- what is camera correspondence, focal length,baseline, intrinsic and extrinsic parameters of camera, coordinate system of camera (epi polar)
- what is pin hole camera

## TODO
- filter and smooth disparity map (remove noises)
- create depth map from the this processed disparity map
- create point cloud data from disparity map
- calibration of logitech c270 camera
    - use checker board and ubuntu packages
    
## Possibility of errors
- if we are running two parallel nodes for camera,check there is any issues with time synchronization

## fix errors
- create an img publisher using known stereo image, then check the output
- understand how disparity calculation works in opencv
- heared that stereoBM and stereoSGBM are very sensitive, so camera calibration is necessary

## ref links
- https://www.youtube.com/watch?v=5LrAhSHNIJU
- https://medium.com/%40me.muhammed.dinc/part-vi-ros2-jazzy-jalisco-with-webcam-python-ffeed9d04551
- https://www.youtube.com/watch?v=WhkiPYPIO9M
- https://youtu.be/WhkiPYPIO9M 

## note
print an opencv matrix 
- RCLCPP_INFO_STREAM(this->get_logger(), "Matrix values:\n" << disparity);

