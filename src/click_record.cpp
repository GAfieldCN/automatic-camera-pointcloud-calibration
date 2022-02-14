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

using namespace std;

geometry_msgs::PointStamped clickedpoint;
std::vector<geometry_msgs::PointStamped> click_points;




void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);


int main(int argc,char ** argv){
    ros::init(argc,argv,"lidar_record");
    ros::NodeHandle nh;

    std::string path;
    int num;
    ros::param::param<std::string>("click_path", path, "/home/ljh/data/click.txt");
    ros::param::param<int>("selected_number", num, 16);
    std::ofstream out(path);


    std::cout << "Writing data to :" << path << std::endl;
    std::cout << "Please select : " << num << " points" << std::endl;

    ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);

    ros::Rate rate(10);

    while (ros::ok()){

        if (click_points.size() == num){


            for (int i = 0; i < click_points.size(); i++){
                out << click_points[i].point.x << " " << click_points[i].point.y << " " << click_points[i].point.z << std::endl;
            }
            cout << "points are enough, please ctrl+c to exit ... " << std::endl;
            num = 0;
        }

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}

void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    clickedpoint = *msg;
    cout << "new point is " << clickedpoint.point.x << " " << clickedpoint.point.y << " " << clickedpoint.point.z << std::endl;
    cout << msg->header.seq + 1 << " points have been selected" << std::endl;
    click_points.push_back(clickedpoint);
}













