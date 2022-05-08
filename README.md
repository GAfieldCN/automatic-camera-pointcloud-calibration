# Automatic-Camera-Pointcloud-Calibration Manual

This package provides a method for automatically calibrating the extrinsic parameters between 
pointcloud sensors (especially Livox LiDAR) and camera.

![image](https://github.com/GAfieldCN/automatic-camera-pointcloud-calibration/blob/master/figs/east-gate.png)

## 1 Environment configuration

(The following calibration process is under the Ubuntu 64-bit 18.04 and ROS melodic environment )

### 1.1 Install Livox SDK and driver

Install [Livox SDK](https://github.com/Livox-SDK/Livox-SDK) and [livox_ros_driver](https://github.com/Livox-SDK/Livox-SDK-ROS). 
You can skip this step if they are already installed. 

```
# install Livox_SDK
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
sudo ./third_party/apr/apr_build.sh
cd build && cmake ..
make
sudo make install

# install livox_ros_driver
cd ..
git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
cd ws_livox
catkin_make
```

### 1.2 Install dependencies

You can skip this step if they are already installed.

- [PCL installation](http://www.pointclouds.org/downloads/linux.html)

- [Eigen installation](http://eigen.tuxfamily.org/index.php?title=Main_Page)

- [Ceres-solver installation](http://ceres-solver.org/)

### 1.3 Compile  source code of calibration

```
# install calibration package
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
#TODO copy the folder livox_camera_lidar_calibration under the src
#OR git clone https://github.com/GAfieldCN/automatic-camera-pointcloud-calibration.git
cd ..
catkin_make
source devel/setup.bash
```

### 1.4 Program nodes briefs

This project includes the following nodes:

- feature_extraction_camera - obtain the corner of the photo

- feature_extraction_pointcloud - obtain the corner of the LiDAR

- feature_manual_camera - obtain the corners manually

- feature_manual_lidar - obtain the lidar corners manually

- GetExtrinsic - optimization to get the extrinsic parameters

- colorLidar - LiDAR point cloud colorized visualization

You can find the corresponding launch file in the launch folder if you want to modify any launch file.

## 2 Calibration of camera intrinsic parameters

Using MATLAB, OpenCV or Kalibr etc. to get intrinsic parameters. 

## 3 Preparations and data collections

### 3.1 Calibration scene preparation

The calibration board is recommended to be acrylic plate or foam, with 4 square holes symmetrically placed. 
You can take the following as an example:

![image](https://github.com/GAfieldCN/automatic-camera-pointcloud-calibration/blob/master/figs/board.png)

You should make sure that the boards have certain distance from the background objects, and ensure that the LiDAR has a certain distance
from the calibration board, probably 3-5m is the best. Each board could provide 16 points, but it is recommended
to use 2-3 boards and arrange them uniformly in the space with different positions.

- **During the 3.2 and 3.3, make sure the scene is unchanged and the platform remains still**

### 3.2 Connect the LiDAR and record the rosbag
Take the Livox LiDAR as an example (the authors have used the Livox Mid-70)

connect the Lidar and verify the pointcloud
```
roslaunch livox_ros_driver livox_lidar_rviz.launch
```

shut down the terminal, and we need to start the livox_custom_msg 
```
roslaunch livox_ros_driver livox_lidar_msg.launch
```

Record the rosbag for about 20 seconds (Too large bag might take a while to compute)
```
rosbag record -o lidar /livox/lidar
```

### 3.3 Connect the camera and record

Please connect and turn on the camera, then take the photo. 

### 3.4 Collect the photo and LiDAR data

After collecting all the data, put photos in data/photo folder; LiDAR rosbag in data/lidar folder.

## 4 Calibration Process

### 4.1 Intrinsic Setting

Firstly save the intrinsic parameters and distortion correct parameters in the path data/parameters/intrinsic.txt. 
Distortion corresponds to five distortion correction parameters, which in order are: k1, k2 (RadialDistortion), p1, p2(TangentialDistortion) and k3, normally set 0 by default. 
An example is given as intrinsic.txt. 

### 4.2 Photo Feature Extraction

Configure the launch file feature_extraction_camera.launch, make sure the **plane size** accords with the number of boards, and
you might change the **binary_threshold** when the binary process works not well! Also, the **min_area** and
the **max_area** determine the contour area to be extracted. 

```
roslaunch automatic-camera-pointcloud-calibration feature_extraction_camera.launch
```

![image](https://github.com/GAfieldCN/automatic-camera-pointcloud-calibration/blob/master/figs/camera.png)
The program will open the corresponding photo and show the rectified image. Tap any key to enter the 
debug process, i.e., you can remove the false-detected contours. 
If you want to remove the 5 6 7 8 contours, enter

```
5 6 7 8 0
```
And if the contours are correct, tap 0 and enter to finish the process, then the program will calculate the features in more precise float type, 
and save them in default output path data/feature/corner_photo.txt. Then press a random key to end the whole process. 

![image](https://github.com/GAfieldCN/automatic-camera-pointcloud-calibration/blob/master/figs/camera_result.png)

### 4.3 Point Cloud Feature Extraction

Configure the launch file feature_extraction_pointcloud.launch, make sure the **plane size** accords with the number of boards, and
**board_length** is the length of board(m), **square_length** is the length of holes(m). Adjust the **FOV** to fit your LiDAR, 
but you can decrease the value to speed up the process as long as the boards are visible. 

Then, run the command
```
roslaunch automatic-camera-pointcloud-calibration feature_extraction_pointcloud.launch
```
![image](https://github.com/GAfieldCN/automatic-camera-pointcloud-calibration/blob/master/figs/lidar.png)

The detected corners will be saved in the default output path data/feature/corner_pointcloud.txt.

## Step5: Extrinsic calculation

### 5.1 Parameter setting

Extrinsic calculation node will read the calibration data in data/corner_photo.txt and data/corner_lidar.txt to calculate 
the extrinsic parameters. The program will stop reading the data and start calculating when it reads blank lines.


Before the calculation, check the calibration data to make sure that each line corresponds to the same corner and the data amount is the same.

### 5.2 Optimization Process

Configure the launch file GetExtrinsic.launch, and run

```
roslaunch automatic-camera-pointcloud-calibration GetExtrinsic.launch 
```

The cost of each iteration operation will be printed out on the terminal, and the result will be saved in the path data/parameters/extrinsic.txt in the format of homogeneous matrix. 

Note: if the calibration result seems incorrect(i.e., the following colorization works not well), but no error occurs during the optimization process, 
which means the process has got into the local minimum trap. In that case, you should add some points manually by the following nodes.

### 5.3 (Optional) Manual feature extraction

Configure the launch file feature_manual_camera.launch, and run

```
roslaunch automatic-camera-pointcloud-calibration feature_manual_camera.launch
```
![image](https://github.com/GAfieldCN/automatic-camera-pointcloud-calibration/blob/master/figs/manual_camera.png)
The result will be saved in the data folder.

Then, configure the launch file feature_manual_lidar.launch, make sure the **selected_number**
equals to the photo corners you have just picked, and run

```
roslaunch automatic-camera-pointcloud-calibration feature_manual_lidar.launch
```

![image](https://github.com/GAfieldCN/automatic-camera-pointcloud-calibration/blob/master/figs/manual_lidar.png)
Use the **publish point** in the rviz and click the corresponding points, the process will be 
terminated when the total point reach the selected number.

## Step6: Colorized point cloud

Configure the launch file color_lidar_display.launch, and run

```
 roslaunch automatic-camera-pointcloud-calibration colorLidar.launch 
 ```

![image](https://github.com/GAfieldCN/automatic-camera-pointcloud-calibration/blob/master/figs/color.png)

# automatic-camera-pointcloud-calibration
