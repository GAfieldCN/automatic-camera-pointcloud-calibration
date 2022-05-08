#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "std_msgs/String.h"
#include <sstream>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <stdio.h>
#include <cmath>
#include <hash_map>
#include <ctime>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include "common.h"
#include "CustomMsg.h"

using namespace std;

geometry_msgs::PointStamped clickedpoint;
std::vector<geometry_msgs::PointStamped> click_points;

typedef pcl::PointXYZRGB PointType;
vector<livox_ros_driver::CustomMsg> lidar_datas;
int threshold_lidar, selected_number;
string input_bag_path, output_path;

void loadPointcloudFromROSBag(const string& bag_path) {
    ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg"));  // message title
    rosbag::View view(bag, rosbag::TypeQuery(types));

    for (const rosbag::MessageInstance& m : view) {
        livox_ros_driver::CustomMsg livoxCloud = *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
        lidar_datas.push_back(livoxCloud);
        if (lidar_datas.size() > threshold_lidar) {
            break;
        }
    }
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path)) {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    }
    if (!ros::param::get("threshold_lidar", threshold_lidar)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
    if (!ros::param::get("output_path", output_path)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("selected_number", selected_number)) {
        cout << "Can not get the value of selected_number" << endl;
        exit(1);
    }
}


void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    clickedpoint = *msg;
    cout << "new point is " << clickedpoint.point.x << " " << clickedpoint.point.y << " " << clickedpoint.point.z << std::endl;
    cout << msg->header.seq + 1 << " points have been selected" << std::endl;
    click_points.push_back(clickedpoint);
}


int main(int argc,char ** argv){
    ros::init(argc,argv,"lidar_record");
    ros::NodeHandle nh;
    getParameters();

//    ros::param::param<std::string>("click_path", path, "/home/ljh/data/click.txt");
//    ros::param::param<int>("selected_number", num, 16);
    std::ofstream out(output_path);

    std::cout << "Writing data to :" << output_path << std::endl;
    std::cout << "**-- Please select : " << selected_number << " points using publish Point tag in RVIZ! --**" << std::endl;

    loadPointcloudFromROSBag(input_bag_path);

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_raw", 1000);
    ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);

    ros::Rate rate(10);

    uint64_t num_iter = 0;

    while (ros::ok()){
        ros::spinOnce();

        if(num_iter < lidar_datas.size()) {
            pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
            cloud->is_dense = false;
            cloud->height = 1;
            cloud->width = lidar_datas[num_iter].point_num; // get the point number of lidar data
            cloud->points.resize(cloud->width);
            for(uint64_t i = 0; i < cloud->points.size() && nh.ok(); ++i) {
                float x = lidar_datas[num_iter].points[i].x;
                float y = lidar_datas[num_iter].points[i].y;
                float z = lidar_datas[num_iter].points[i].z;

                // ignore the invalid point
                if(x == 0 && y == 0 && z == 0) {
                    continue;
                }

                // set coordinate for the cloud point
                cloud->points[i].x = x;
                cloud->points[i].y = y;
                cloud->points[i].z = z;

            }
            // once lidar_datas receive something new, it will transform it into a ROS cloud type
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);

            output.header.frame_id = "livox_frame";
            pub.publish(output); // publish the cloud point to rviz
            rate.sleep();
            ++num_iter;

        }
            // clean the data when the process is finished
//        else if(num_iter >= 10) {
//            lidar_datas.clear();
//            num_iter = 0;
//            ROS_INFO("Finish all the process");
//            break;
//        }

        if (click_points.size() == selected_number){
            for (int i = 0; i < click_points.size(); i++){
                out << click_points[i].point.x << " " << click_points[i].point.y << " " << click_points[i].point.z << std::endl;
            }
            cout << "points are enough, please ctrl+c to exit ... " << std::endl;
            selected_number = 0;
        }

//        ros::spinOnce();
//        rate.sleep();
    }
    return 0;
}














