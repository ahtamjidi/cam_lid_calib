# Task 1
These are the commands that I use. I am planning to write some launch files and make things tidier later. Please let me know if I am doing something wrong. I installed this package which seems to have all the things that I need for the first part
sudo apt-get install ros-indigo-perception 
camera_calibration
image_proc
image_view
And the following package seems to be able to take care of the lidar to camera calibration
lidar_camera_calibration

to get info about the bagfile I use
rosbag info BAG_FILE
to play it so that topics are published for the camera calibration I use
rosbag play ../2016-11-22-14-32-13_test.bag 
given the info in the assignment the calibration can be done
rosrun camera_calibration cameracalibrator.py --size 5x7 --square 0.05 image:=/sensors/camera/image_color
after collecting enough images the calibration result is 

```
image_width: 964
image_height: 724
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [483.761692, 0.000000, 456.184555, 0.000000, 483.550078, 365.883083, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.196606, 0.064156, 0.002826, -0.000069, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix:
  rows: 3
  cols: 4
  data: [420.646484, 0.000000, 459.177477, 0.000000, 0.000000, 431.080780, 369.612387, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```
I use the following command to replace the camera_info of the original bag file with the result of the calibration and then save the result in a new bagfile

```
python /opt/ros/indigo/lib/python2.7/dist-packages/bag_tools/change_camera_info.py ../2016-11-22-14-32-13_test.orig.bag ../2016-11-22-14-32-13_test.newcalib.bag /sensors/camera/camera_info=./ost.yaml
```
I have saved the images used for camera calibration in the `data` folder
```
calibration_attempt_1	added vudei and data	
calibration_attempt_2	added vudei and data	
```
I used `rect2.launch` to replay and rectify the images. The resulting video is saved in 
`rectified_image_video.avi`	.

# Task 2

To accomplish this task I first manually found some associated pairs of (2D, 3D) points and then solved a pnp problem. The code can be found in `lidar_camera_calib.cpp`. The rviz config file is also located in data folder and it is named `lidar_cam.rviz`

```
frame0034.jpg
lu = [1.4020, -0.597752, -0.133348] uv = [597, 323]
ru = [4.9928, -1.7213, -1.0266]  uv = [809, 345]
rb = [1.3947, -1.1233, -0.47985]  uv = [788, 472]
lb = [1.492, -0.58211, -0.42912]  uv = [577, 454]

frame0000.jpg
lu = [1.2935, -0.16672, 0.068351] uv = [272, 284]
ru = [1.3064, -0.37849, 0.071282]  uv = [515, 292]
lb = [1.3755, 0.17589, -0.21963]  uv = [263, 432]

frame0002.jpg
lu = [1.3547, 0.14028, 0.36493] uv = [290, 59]
lb = [1.49293, 0.15089, 0.17647]  uv = [278, 194]
ru = [1.2479, -0.39149, 0.35044]  uv = [513, 50]
```
Something I initially forgot was to consider the difference between the default orientation of camera and lidar
camera
https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
velodyne
https://github.com/ros-drivers/velodyne/issues/113

Therefore in my code I considered the following transformation before running solvePNP
```
x_c = -y_l
y_c = -z_l
z_c = x_l
```
running the pnp algorithm results in 
```
rosrun calib calib_lid2cam
Camera Matrix
[483.761692, 0, 456.184555;
  0, 483.550078, 365.883083;
  0, 0, 1]
Rotation Vector
[0.2096623278002438; 0.3710162110018459; 0.09669619186989108]
Translation Vector
[-0.7380530632718042; -0.02821639983304844; 0.01461670013504489]
```

During the development of lidar to image overlay code I started visualizing the reprojection error of camera lidar calibration. I tried to fix these errors. One of the great mistakes that I did in the beginning was neglecting the rotation between camera and lidar coordinates. I think a more accurate way of doing the calibration is to extract planes from chessboard both using camera and lidar. I wrote some code but it is not tested and not complete. I decided to first try the manual method.

These are my thoughts and progress on lidar to image overlay. I have to filter the point cloud to only project  Node `calib__lidonimage` can use the project function defined in `project_pcl_to_image.cpp` to do a simple filtering. `lidar_to_image.cpp` has subscribers to camera image, lidar and cam_info. It also has callback functions for each of these subscribers. A proper method to handle the lidar overlay seems to be
1- receive cam_info, image and lidar
2- filter the point cloud to only keep the relevant part of the point cloud (field of view of the camera)
3- transform and project point cloud onto image plane.

For the last part I experimented with `image_transform` package which seemed to be the recommended way of doing it. However I had compile problems related to boost which I could not solve. I abandoned image_transform to work on something that I can test.

