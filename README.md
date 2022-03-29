## Automatic-Camera-Pointcloud-Calibration Manual

This package provides a method for automatically calibrating the extrinsic parameters between 
pointcloud sensors (especially Livox LiDAR) and camera.

### 1 Environment configuration

(The following calibration process is under the Ubuntu 64-bit 18.04 and ROS melodic environment )

#### 1.1 Install Livox SDK and driver

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

#### 1.2 Install dependencies

You can skip this step if they are already installed.

- [PCL installation](http://www.pointclouds.org/downloads/linux.html)

- [Eigen installation](http://eigen.tuxfamily.org/index.php?title=Main_Page)

- [Ceres-solver installation](http://ceres-solver.org/)

#### 1.3 Compile  source code of calibration

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

#### 1.4 Program nodes briefs

This project includes the following nodes:

- feature_extraction_camera - obtain the corner of the photo

- feature_extraction_pointcloud - obtain the corner of the LiDAR

- GetExtrinsic - optimization to get the extrinsic parameters

- projectCloud - project the LiDAR point cloud on the photo

- colorLidar - LiDAR point cloud colorized visualization

You can find the corresponding launch file in the launch folder if you want to modify any launch file.

### 2 Calibration of camera intrinsic parameters

Using MATLAB, OpenCV or Kalibr etc. to get intrinsic parameters. 

### 3 Preparations and data collections

#### 3.1 Calibration scene preparation

The calibration board is recommended to be acrylic plate or foam, with 4 square holes symmetrically placed. 
You can take the following as an example:

![Aaron Swartz](https://raw.githubusercontent.com/GAfieldCN/automatic-camera-pointcloud-calibration/master/figs/board.png?token=GHSAT0AAAAAABS5QE7UPNFPC72A7UDVVI7YYSCOFUQ)

It will be better to select a relatively empty environment for the calibration scene
to facilitate the identification of the calibration board, and to ensure that the LiDAR has a certain distance
from the calibration board, probably 3-5m are best. Each board could provide 16 points, but it is recommended
to use 2-3 boards, and it is better to arrange them with different positions and different angles.

- **During the 3.2 and 3.3, make sure the scene is unchanged and the platform remains still**

#### 3.2 Connect the LiDAR and record the rosbag
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
rosbag record -O lidar_msg.bag /livox/lidar
```

#### 3.3 Connect the camera and record

Please connect and turn on the camera, then take the photo. 

#### 3.4 Collect the photo and LiDAR data

After collecting all the data, put photos in data/photo folder; LiDAR rosbag(i.e., lidar_msg.bag) in data/lidar folder.

### Step4: Calibration data acquisition

#### 4.1 Parameter setting

Firstly save the intrinsic parameters and distortion correct parameters in the path data/parameters/intrinsic.txt. Distortion corresponds to five distortion correction parameters, which in order are: k1, k2 (RadialDistortion), p1, p2(TangentialDistortion) and k3, normally set 0 by default. An example is given as intrinsic_example.txt. 

#### 4.2 Acquire the corner coordinates in photo

1. Configure the launch file cornerPhoto.launch, and run

```
roslaunch camera_lidar_calibration cornerPhoto.launch
```

2. The program will open the corresponding photo. Close the **source** window, and click 4 points on the **src** window. Then enter the four corners “x y” in order, and there must be a space between x and y (for example: “635 487”). After that, enter “0 0” to end the input process. The program will calculate four points in more precise float type, and save them in default output path data/corner_photo.txt. Then press a random key to end the whole process. 
3. To select at least 12 points, this process must be repeated at least 3 times. However, to avoid confusion, we highly recommend you to finish step 4.3 to find out the corresponding 4 points of LiDAR, then come back to select another 4 points. 


#### 4.3 Acquire the corner coordinates in point cloud

1. Open a terminal (make sure roscore is running)
```
rviz
```
In the **file** panel, choose **open config**, then select **display_lidar_points.rviz** in the folder. 

2. In where the rosbag **livox_rviz.bag** saved, open a terminal and run

```
rosbag play livox_rviz.bag
```
The play process could be paused by tapping a space. 

3. Open a new terminal to subscribe rostopic
```
rostopic echo /clicked_point
```

 4. Find the corresponding 4 points, then choose **publish point** to publish them on the topic. They will be displayed on the termimal in step 3. 


5. Write the xyz coordinates in the data/corner_lidar.txt. An example format is also given. 

Then, return to 4.2 to acquire another 4 points. 

### Step5: Extrinsic calculation

#### 5.1 Parameter setting

Extrinsic calculation node will read the calibration data in data/corner_photo.txt and data/corner_lidar.txt to calculate the extrinsic parameters, the data needs to be saved in a specific format to be correctly read by this node. Referring to the figure below, only the data with more than 10 letters in one line will be read as a calculation data, the title such as the 1 2 3 4, lidar0 and 0.jpg will not be read as calculation data. The program will stop reading the data and start calculating when it reads blank lines.


Before the calculation, check the calibration data to make sure that each line corresponds to the same corner and the data amount is the same.

#### 5.2 Calculation node getExt1

Configure the initial extrinsic value in the getExt1.launch file first, then run the command to calculate the extrinsic parameters.

```
roslaunch camera_lidar_calibration getExt1.launch
```

The cost of each iteration operation will be printed out on the terminal, and the result will be saved in the path data/parameters/extrinsic.txt in the format of homogeneous matrix. 

### Step6: Results verification and related applications

#### 6.1 Briefs

After obtaining extrinsic parameters, we can use two common applications to see the fusion result. The first one is the projection of the point cloud on the photo, the second one is the point cloud coloring.

#### 6.2 projection of point cloud on the photo

Set the rosbag (**lidar_msg.bag**)and photo path in the projectCloud.launch file, run the command to project a certain number of point cloud on the photo

```
roslaunch camera_lidar_calibration projectCloud.launch
```
The parameter **threshold_lidar** determines the density of pointclouds. 


#### 6.3 Point cloud coloring

Set the rosbag (**lidar_msg.bag**) and photo path in the colorLidar.launch file, run the command and check coloring rendering in the rviz.

```
roslaunch camera_lidar_calibration colorLidar.launch
```
# automatic-camera-pointcloud-calibration
