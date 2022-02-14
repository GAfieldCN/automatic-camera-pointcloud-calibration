/*------------------------------------------------------------------------------
* function : Publish data to ground or other agents.
* ROS      : data_node
* Author   : Cheng Chi
* version  : 1.0
* history  : 2019/06/18 1.0 new
*-----------------------------------------------------------------------------*/

#include "data.h"

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sensor_msgs/Imu.h>

#define ADDR "127.0.0.1" 
#define INS_PORT 9003

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

FILE * vel_fd;
FILE * imu_fd;
int c_fd;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;
ros::Publisher ground_pub;

ros::Publisher posPub;
ros::Publisher velPub;
ros::Publisher atiPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;

// serial

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);;
  ros::Subscriber velocitySub = nh.subscribe("dji_sdk/velocity", 10, &velocity_callback);
  ros::Subscriber imuSub = nh.subscribe("/imu/raw", 10, &imu_callback);


  // Open data file
  time_t _t = time(NULL);
  struct tm *lc_t = localtime(&_t);
  char data_file_vel[50];
  char data_file_imu[50];
  sprintf(data_file_vel, "/home/ljh/data/uav/dji_%04d%02d%02d_%02d.vel", 1900+lc_t->tm_year, 1+lc_t->tm_mon, lc_t->tm_mday, lc_t->tm_hour);
  sprintf(data_file_imu, "/home/ljh/data/uav/dji_%04d%02d%02d_%02d.imu", 1900+lc_t->tm_year, 1+lc_t->tm_mon, lc_t->tm_mday, lc_t->tm_hour);

  vel_fd = fopen(data_file_vel, "wb");
  imu_fd = fopen(data_file_imu, "wb");

  ros::spin();
  return 0;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
  atiPub.publish(*msg);

  // ROS_INFO("##### attitude_callback: %.2f, %.2f, %.2f", toEulerAngle(current_atti).x, toEulerAngle(current_atti).y, toEulerAngle(current_atti).z);
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
  // ROS_INFO("##### local_position_callback: %.2f, %.2f, %.2f", current_local_pos.x, current_local_pos.y, current_local_pos.z);
}



void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
  // ROS_INFO("##### flight status: %d", flight_status);
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}





void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
//  geometry_msgs::Vector3Stamped vel = *msg;
//  char buf[200];
//  std_msgs::String str;
//
//  // Publish global variable
//  velPub.publish(vel);
//
//  struct timeval tv;
//  gettimeofday(&tv, NULL);
//
//    fprintf(vel_fd, "%010u.%09u %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\r\n", vel.header.stamp.sec, vel.header.stamp.nsec, imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z, imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, Euler.x, Euler.y, Euler.z);
//
//
}
//
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
//    sensor_msgs::Imu imu = *msg;
//    geometry_msgs::Vector3 Euler =  toEulerAngle(imu.orientation);
//    char buf[200];
//    std_msgs::String str;
//
//    struct timeval tv;
//    gettimeofday(&tv, NULL);
//
////    sprintf(buf, "%010ld.%06ld %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\r\n", tv.tv_sec, tv.tv_usec, imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z, imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, Euler.x, Euler.y, Euler.z);
////    fwrite(buf, sizeof(unsigned char), strlen(buf), imu_fd);
//
//    //fprintf(imu_fd, "%010ld.%06ld %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\r\n", tv.tv_sec, tv.tv_usec, imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z, imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, Euler.x, Euler.y, Euler.z);
//    fprintf(imu_fd, "%010u.%09u %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\r\n", imu.header.stamp.sec, imu.header.stamp.nsec, imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z, imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, Euler.x, Euler.y, Euler.z);
//
}
