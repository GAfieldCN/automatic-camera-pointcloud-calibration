#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <fstream>
#include <string>
#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <numeric>
#include <Eigen/Dense>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include "common.h"
#include "result_verify.h"
#include "CustomMsg.h"

using namespace std;

double rad2deg = 180 / 3.1415926;

struct point_type{
    double x, y, z;
    double elevation;
    double azimuth;
    double range;
    double reflectivity;
};

point_type point;
vector<point_type> points, temp_points, frontier_points, plane, filtered_points1, filtered_points2;
vector<double> plane_param;
vector< vector<double> > plane_params;

vector<livox_ros_driver::CustomMsg> lidar_datas;
int threshold_lidar, FOV_vertical, FOV_horizontal;
string input_bag_path, input_photo_path, output_path, intrinsic_path, extrinsic_path, temp_path;
double square_length, board_length;

void SetMarker(visualization_msgs::Marker &point, int id, int type, float scale, float r, float g, float b)
{
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.ns = "visualization";
    point.pose.orientation.w = 1.0;

    point.id = id;

    point.action = point.ADD;

    //point.type  = visualization_msgs::Marker::POINTS;
    point.type  = type;

    point.scale.x = scale; //node:0.08
    point.scale.y = scale;

    point.lifetime = ros::Duration();

    point.color.r = r / 255.0;  //255
    point.color.g = g / 255.0;  //69
    point.color.b = b / 255.0;  //0
    point.color.a = 255.0/255.0;
}

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
        if (lidar_datas.size() > (threshold_lidar/24000 + 1)) {
            break;
        }
    }
    cout << "Loading rosbag successes" << endl;
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path)) {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    }
    if (!ros::param::get("output_path", output_path)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("temp_path", temp_path)) {
        cout << "Can not get the value of temp_path" << endl;
        exit(1);
    }
    if (!ros::param::get("threshold_lidar", threshold_lidar)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
    if (!ros::param::get("square_length", square_length)) {
        cout << "Can not get the value of square_length" << endl;
        exit(1);
    }
    if (!ros::param::get("board_length", board_length)) {
        cout << "Can not get the value of board_length" << endl;
        exit(1);
    }
    if (!ros::param::get("FOV_vertical", FOV_vertical)) {
        cout << "Can not get the value of FOV_vertical" << endl;
        exit(1);
    }
    if (!ros::param::get("FOV_horizontal", FOV_horizontal)) {
        cout << "Can not get the value of FOV_horizontal" << endl;
        exit(1);
    }
    cout << "Loading parameters done!" << endl;
}

bool hor_sort (vector <double> a, vector <double> b){
    return (a[2] > b[2]);
}

bool ver_sort (vector <double> a, vector <double> b){
    return (a[1] > b[1]);
}

vector<int> RANSAC_Plane(int size, int size_old, double threshold_dis, double threshold_std, long int max_iter, int allow_num){

    double a, b, c, d;
    double out_sum, out_accum;
    vector<int> index, index_out;
    while (--max_iter)
    {
        index.clear();
        out_sum = 0, out_accum = 0;
        //cout << "iter = " << max_iter << endl;
        for (int k = 0; k < size; ++k) {
            index.push_back(rand() % filtered_points2.size());
        }

        auto idx = index.begin();
        double x1 = filtered_points2.at(*idx).x, y1 = filtered_points2.at(*idx).y, z1 = filtered_points2.at(*idx).z;
        ++idx;
        double x2 = filtered_points2.at(*idx).x, y2 = filtered_points2.at(*idx).y, z2 = filtered_points2.at(*idx).z;
        ++idx;
        double x3 = filtered_points2.at(*idx).x, y3 = filtered_points2.at(*idx).y, z3 = filtered_points2.at(*idx).z;

        a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        d = -(a * x1 + b * y1 + c * z1);

        if(index[0] == index[1] || index[0] == index[2] || index[1] == index[2])
        {//cout << "choose the same points!" << endl;
            continue;}

        if( (x1 - x2)*(y1 - y3) == (x1 - x3)*(y1 - y2) )
        {//cout << "points lie in same line!" << endl;
            continue;}

        for (auto iter = filtered_points2.begin(); iter != filtered_points2.end(); ++iter) {
            double dis = fabs(a * iter->x + b * iter->y + c * iter->z + d) /
                         sqrt(a * a + b * b + c * c);//distance from point to plane
            if (dis < threshold_dis) index.push_back(iter - filtered_points2.begin());
        }

        for (auto it = 0; it < index.size(); it++) {
            out_sum += filtered_points2[index[it]].y;
        }

        for (auto it = 0; it < index.size(); it++) {
            out_accum += pow(filtered_points2[index[it]].y - (out_sum / index.size()), 2);
        }

        auto out_std = sqrt(out_accum / (index.size() - 1));

        if (index.size() > size_old && out_std < threshold_std) {
            plane_param.clear();
            plane_param.push_back(a);plane_param.push_back(b);plane_param.push_back(c);plane_param.push_back(d);
            size_old = index.size();
            index_out = index;
        }
    }

    plane_params.push_back(plane_param);
    return index_out;
}

vector<double> RANSAC_Line(vector<point_type> &one_plane, vector<double> param, int size, int size_old, double threshold_dis, double threshold_std, long int max_iter){

    double a, b, c;
    double out_sum, out_accum;
    vector<int> index, index_out_line;
    vector<double> out;
    while (--max_iter)
    {
        index.clear();
        for (int k = 0; k < size; ++k) {
            index.push_back(rand() % one_plane.size());
        }

        if(index[0] == index[1])
        {//cout << "choose the same points!" << endl;
            continue;}

        auto idx = index.begin();
        double x1 = one_plane.at(*idx).x, y1 = one_plane.at(*idx).y, z1 = one_plane.at(*idx).z;
        ++idx;
        double x2 = one_plane.at(*idx).x, y2 = one_plane.at(*idx).y, z2 = one_plane.at(*idx).z;

        double dx = x2 - x1;
        double dy = y2 - y1;
        double dz = z2 - z1;
        a = dx / norm3(dx, dy, dz);
        b = dy / norm3(dx, dy, dz);
        c = dz / norm3(dx, dy, dz);


        for (auto iter = one_plane.begin(); iter != one_plane.end(); ++iter) {
            double dis = dis_P2L_3d(iter->x, iter->y, iter->z, x1, y1, z1, a, b, c);
            if (dis < threshold_dis && iter - one_plane.begin() != index[0] && iter - one_plane.begin() != index[1]) index.push_back(iter - one_plane.begin());
        }


        if (index.size() > size_old && abs(a)+abs(b)+abs(c) < 1.2) {
            size_old = index.size();
            index_out_line = index;

            out.clear();
            double xp = ((param[1]*param[1]+param[2]*param[2])*x1-param[0]*(param[1]*y1+param[2]*z1+param[3]))/(param[0]*param[0]+param[1]*param[1]+param[2]*param[2]);
            double yp = ((param[0]*param[0]+param[2]*param[2])*y1-param[1]*(param[0]*x1+param[2]*z1+param[3]))/(param[0]*param[0]+param[1]*param[1]+param[2]*param[2]);
            double zp = ((param[0]*param[0]+param[1]*param[1])*z1-param[2]*(param[0]*x1+param[1]*y1+param[3]))/(param[0]*param[0]+param[1]*param[1]+param[2]*param[2]);
            out.push_back(xp); out.push_back(yp); out.push_back(zp);
            out.push_back(a) ; out.push_back(b) ; out.push_back(c); out.push_back(index.size());
        }
    }

    sort(index_out_line.begin(), index_out_line.end());

    for (int p = index_out_line.size() - 1 ; p > - 1; p --){
        one_plane.erase(one_plane.begin() + index_out_line[p]);
    }
    return out;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "feature_extraction_pointcloud");
    ros::NodeHandle nh;
    ros::Rate rate(0.1);

    getParameters();
    srand(time(0));

    ros::Publisher points_pub = nh.advertise<geometry_msgs::Point>( "/pointcloud", 10000 );
    ros::Publisher pub_marker = nh.advertise<visualization_msgs::Marker>("/points_visualization", 1000);
    ros::Publisher pub_frontier = nh.advertise<visualization_msgs::Marker>("/points_frontier", 1000);
    ros::Publisher pub_filter = nh.advertise<visualization_msgs::Marker>("/points_filter", 1000);
    ros::Publisher pub_plane = nh.advertise<visualization_msgs::Marker>("/planes", 1000);
    ros::Publisher pub_line = nh.advertise<visualization_msgs::Marker>("/lines", 1000);
    ros::Publisher pub_corner = nh.advertise<visualization_msgs::Marker>("/corners", 1000);
    ros::Publisher pub_result = nh.advertise<visualization_msgs::Marker>("/result", 1000);

    const int plane_size = 3, line_size = 12;

    vector<vector<point_type>> planes;
    geometry_msgs::Point point_cloud, frontier, point_plane, point_line, filter;

    vector<double> line_param;
    vector< vector <double> > horizontal_line_params[plane_size];
    vector< vector <double> > vertical_line_params[plane_size];

    vector<geometry_msgs::Point> point_clouds;
    visualization_msgs::Marker marker_pointcloud, marker_frontier, marker_filter, marker_plane[plane_size], marker_line[line_size*plane_size], marker_corners;
    vector<visualization_msgs::Marker> marker_planes;

    SetMarker(marker_pointcloud, 0, 7, 0.01, 255, 200, 8);
    SetMarker(marker_frontier, 0, 7, 0.01, 255, 0, 0);
    SetMarker(marker_filter, 0, 7, 0.01, 0, 255, 0);
    SetMarker(marker_corners, 0, 7, 0.03, 255, 10, 10);

    loadPointcloudFromROSBag(input_bag_path);
    int myCount = 0;
    for (size_t i = 0; i < lidar_datas.size(); ++i) {
        for (size_t j = 0; j < lidar_datas[i].point_num; ++j) {
            // visualization
            point_cloud.x = lidar_datas[i].points[j].x;
            point_cloud.y = lidar_datas[i].points[j].y;
            point_cloud.z = lidar_datas[i].points[j].z;
            point_clouds.push_back(point_cloud);
            marker_pointcloud.points.push_back(point_cloud);

            // Data collection
            point.x = lidar_datas[i].points[j].x;
            point.y = lidar_datas[i].points[j].y;
            point.z = lidar_datas[i].points[j].z;
            point.reflectivity = lidar_datas[i].points[j].reflectivity;
            point.azimuth = rad2deg * atan(point.y / point.x);
            point.elevation = rad2deg * atan(point.z / sqrt(pow(point.x, 2) + pow(point.y, 2)));
            point.range = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
            points.push_back(point);
            //cout << "x = " << point.x << " y = " << point.y << " z = " << point.z << " ref = " << point.reflectivity << endl;
        }
        if (myCount > threshold_lidar) {
            break;
        }
    }

    /***************************
     ** Detect frontier points **
     ****************************/
    double step = 0.2;
    cout << "Detecting frontier points from the raw data...This may take a while..." << endl;
    cout << "********** Processing" << " ---> "  << "0%  " << "**********" << endl;

    double count = 10;
    for (double j = - FOV_horizontal; j < FOV_horizontal; j += step){
        if ( (int)j == -FOV_horizontal + count * FOV_horizontal * 0.02) {
            cout << "********** Processing" << " ---> " << count << "% " << "**********" << endl;
            count = 10.0 + count;
        }

        for (double k = -10; k < 5; k += step) {
            //cout << "new line : " << "j = " << j << " k = " << k << endl;
            temp_points.clear();
            for (size_t i = 0; i < points.size(); i++){
                if ((abs(points[i].azimuth - j) <= 1.4*step) && (abs(points[i].elevation - k) <= 1.4*step)){
                    temp_points.push_back(points[i]);
                    for (int q = 0; q < temp_points.size(); q ++){
                    }
                }
            }
            if (temp_points.size() != 0 ) {
                double sum = 0, average = 0, accum = 0, stdv = 0, min = 50, max = 0, range = 0;
                int min_index = 0;
                for_each(temp_points.begin(), temp_points.end(), [&sum](point_type b) -> void {
                    sum += b.range;
                });
                average = sum / temp_points.size();
                for_each(temp_points.begin(), temp_points.end(), [&](point_type b) -> void {
                    accum += (b.range - average) * (b.range - average);
                });

                stdv = sqrt(accum / (temp_points.size() - 1));


                for (int q = 0; q < temp_points.size(); q++) {
                    if (temp_points[q].range > max) { max = temp_points[q].range; }
                    if (temp_points[q].range < min) { min = temp_points[q].range; min_index = q;}
                }
                range = max - min;


                if (range > 0.3){
//                    cout << "size = " << temp_points.size() << endl;
//                    cout << "std = " << stdv << endl;
//                    cout << "max = " << max << endl;
//                    cout << "min = " << min << endl;
//                    cout << "range = " << range << endl;
                    frontier.x = temp_points[min_index].x;
                    frontier.y = temp_points[min_index].y;
                    frontier.z = temp_points[min_index].z;
                    frontier_points.push_back(temp_points[min_index]);
                    marker_frontier.points.push_back(frontier);
                }
            }
        }
    }
    cout << "Detecting frontier points done!"<< endl;

    step = 5;
    cout << "Starting to filter points..."<< endl;
    for (double j = -FOV_horizontal; j < FOV_horizontal; j += step) {
        for (double k = -10; k < 5; k += step) {
            temp_points.clear();
            for (size_t i = 0; i < frontier_points.size(); i++) {
                if ((abs(frontier_points[i].azimuth - j) <= 1.2 * step) &&
                    (abs(frontier_points[i].elevation - k) <= 1.2 * step)) {
                    temp_points.push_back(frontier_points[i]);
                }
            }
            if (temp_points.size() != 0) {
                double sum = 0, average = 0, accum = 0, stdv = 0, min = 50, max = 0, range = 0;
                int min_index = 0;
                for_each(temp_points.begin(), temp_points.end(), [&sum](point_type b) -> void {
                    sum += b.range;
                });
                average = sum / temp_points.size();
                for_each(temp_points.begin(), temp_points.end(), [&](point_type b) -> void {
                    accum += (b.range - average) * (b.range - average);
                });

                stdv = sqrt(accum / (temp_points.size() - 1));

                for (int q = 0; q < temp_points.size(); q++) {
                    if (temp_points[q].range < average - 0.02) {
                        filtered_points1.push_back(temp_points[q]);
//                        filter.x = temp_points[q].x;
//                        filter.y = temp_points[q].y;
//                        filter.z = temp_points[q].z;
//                        marker_filter.points.push_back(filter);
                    }
                }
            }
        }
    }
    cout << "filtered points number = " << filtered_points1.size() << endl;
    ofstream outfile(temp_path.c_str());
    if (!outfile) {
        cout << "Can not open the output file" << endl;
        exit(0);
    }

    for (auto i = 0; i < filtered_points1.size(); i++){
        temp_points.clear();
        double dist = 0, dist_sum = 0;
        for (auto j = 0; j < filtered_points1.size(); j++){
            dist = sqrt(pow(filtered_points1[i].x - filtered_points1[j].x, 2) +
                        pow(filtered_points1[i].y - filtered_points1[j].y, 2) +
                        pow(filtered_points1[i].z - filtered_points1[j].z, 2));

            if (dist < 0.05 ) {
                dist_sum += dist;
                temp_points.push_back(filtered_points1[j]);
            }
        }

        if (dist_sum / temp_points.size() > 0.02){
            outfile << filtered_points1[i].x << " " << filtered_points1[i].y << " " << filtered_points1[i].z << " " <<
                    filtered_points1[i].range << " " << filtered_points1[i].elevation << " " << filtered_points1[i].azimuth << endl;
            //cout << "dist = "<< dist_sum / temp_points.size() << endl;
            filtered_points2.push_back(filtered_points1[i]);
            filter.x = filtered_points1[i].x;
            filter.y = filtered_points1[i].y;
            filter.z = filtered_points1[i].z;
            marker_filter.points.push_back(filter);
        }
    }

//    //! Debug process.
//    std::ifstream fin("/home/gnc/catkin_ws/src/automatic-camera-pointcloud-calibration/data/temp.txt", std::ios::in);
//    char line[1024]={0};
//    std::string xx = "";
//    std::string yy = "";
//    std::string zz = "";
//    std::string range = "";
//    std::string elevation = "";
//    std::string azimuth = "";
//    while(fin.getline(line, sizeof(line)))
//    {
//        std::stringstream word(line);
//        word >> xx;
//        word >> yy;
//        word >> zz;
//        word >> range;
//        word >> elevation;
//        word >> azimuth;
//
//        point.x = atof(xx.c_str());
//        point.y = atof(yy.c_str());
//        point.z = atof(zz.c_str());
//        filter.x = point.x;
//        filter.y = point.y;
//        filter.z = point.z;
//        marker_filter.points.push_back(filter);
//        point.range = atof(range.c_str());
//        point.elevation = atof(elevation.c_str());
//        point.azimuth = atof(azimuth.c_str());
//        filtered_points2.push_back(point);
//    }
//    fin.clear();
//    fin.close();
//
//    cout << "filtered points size = " << filtered_points2.size() << endl;

    /***************************
     ** Detect plane using RANSAC **
     ****************************/

    cout << "Detecting planes..." << endl;
    vector<int> index;
    vector <vector <int> > plane_index;
    for (int i = 0; i < plane_size; i++){
        index.clear();
        plane.clear();
        index = RANSAC_Plane(3, 3, 0.02, 0.2, 10000, 1000);
        cout << "Plane " << i+1 << " has " << index.size() << " points" << endl;
        plane_index.push_back(index);
        SetMarker(marker_plane[i], i, 7, 0.015, rand() % 255, rand() % 255, rand() % 255);
//        SetPoint(marker_plane[0], 0, 0.01, 255, 239, 213);
//        SetPoint(marker_plane[1], 0, 0.01, 255, 215, 2);
//        SetPoint(marker_plane[2], 0, 0.01, 2, 191, 255);

        for (auto it = 0; it < plane_index[i].size(); it++) {
            plane.push_back(filtered_points2[plane_index[i][it]]);
            point_plane.x = filtered_points2[plane_index[i][it]].x;
            point_plane.y = filtered_points2[plane_index[i][it]].y;
            point_plane.z = filtered_points2[plane_index[i][it]].z;
            marker_plane[i].points.push_back(point_plane);
        }
        planes.push_back(plane);

        sort(index.begin(), index.end());

        for (int p = index.size() - 1 ; p > - 1; p --){
            filtered_points2.erase(filtered_points2.begin() + index[p]);
        }

    }

    /***************************
    ****** Line Detection ******
    ****************************/

    double horizontal_sum[plane_size], vertical_sum[plane_size];
    for (int i = 0; i < plane_size; i++){
        horizontal_sum[i] = 0; vertical_sum[i] = 0;
        for (int j = 0; j < line_size; j++){
            line_param.clear();
            line_param = RANSAC_Line(planes[i], plane_params[i], 2, 2, 0.01, 0.2, 10000);
            //cout << "Plane "  << i+1 << ", line " << j+1 << " direction:\t";
            //printf("%.3f, %.3f, %.3f\n", line_param[3],line_param[4],line_param[5]);
            //cout << "Point:\t";
            //printf("%.3f, %.3f, %.3f, num = %.1f\n", line_param[0],line_param[1],line_param[2],line_param[6]);

            if (abs(line_param[5])-1 > -0.2) {
                if (line_param[5] > 0) {vertical_line_params[i].push_back(line_param);}
                else if (line_param[5] < 0) {line_param[3] = -line_param[3];line_param[4] = -line_param[4];line_param[5] = -line_param[5];
                    vertical_line_params[i].push_back(line_param);}
                vertical_sum[i] += line_param[6];
            }

            if (abs(line_param[4])-1 > -0.2) {
                if (line_param[4] > 0) {horizontal_line_params[i].push_back(line_param);}
                else if (line_param[4] < 0) {line_param[3] = -line_param[3];line_param[4] = -line_param[4];line_param[5] = -line_param[5];
                    horizontal_line_params[i].push_back(line_param);}
                horizontal_sum[i] += line_param[6];
            }

            point_plane.x = line_param[0];
            point_plane.y = line_param[1];
            point_plane.z = line_param[2];
            SetMarker(marker_line[i*line_size+j], i*line_size+j, 4, 0.01, rand() % 255, rand() % 255, rand() % 255);
            marker_line[i*line_size+j].points.push_back(point_plane);
            double line_step = 0.5;
            for (int k = 0 ; k < 5; k++){
                point_plane.x = line_param[0] + line_step * line_param[3];
                point_plane.y = line_param[1] + line_step * line_param[4];
                point_plane.z = line_param[2] + line_step * line_param[5];
                marker_line[i*line_size+j].points.push_back(point_plane);
                point_plane.x = line_param[0] - line_step * line_param[3];
                point_plane.y = line_param[1] - line_step * line_param[4];
                point_plane.z = line_param[2] - line_step * line_param[5];
                marker_line[i*line_size+j].points.push_back(point_plane);
            }
        }
    }

    Eigen::Vector3d horizontal[plane_size], vertical[plane_size], plane_norm[plane_size];
    double hor_x, hor_y, hor_z, ver_x, ver_y, ver_z;
    for (int i = 0; i < plane_size; i++){
        hor_x = 0; hor_y = 0; hor_z = 0;
        for (int j = 0; j < horizontal_line_params[i].size(); j++){
            hor_x += (horizontal_line_params[i][j][3] * horizontal_line_params[i][j][6]/horizontal_sum[i]);
            hor_y += (horizontal_line_params[i][j][4] * horizontal_line_params[i][j][6]/horizontal_sum[i]);
            hor_z += (horizontal_line_params[i][j][5] * horizontal_line_params[i][j][6]/horizontal_sum[i]);
        }
        horizontal[i] << hor_x, hor_y, hor_z;
        plane_norm[i] << plane_params[i][0], plane_params[i][1],plane_params[i][2];
        plane_norm[i] = plane_norm[i].normalized();
        //cout << "Plane " << i+1 << " normal vector:\t" << plane_norm[i].transpose() << endl;
        horizontal[i] = (horizontal[i] - (horizontal[i].dot(plane_norm[i]) * plane_norm[i] / pow(plane_norm[i].norm(),2))).normalized();
        cout << "Plane " << i+1 << " horizontal vector:\t" << horizontal[i].transpose() << endl;

        ver_x = 0; ver_y = 0; ver_z = 0;
        for (int j = 0; j < vertical_line_params[i].size(); j++){
            ver_x += (vertical_line_params[i][j][3] * vertical_line_params[i][j][6]/vertical_sum[i]);
            ver_y += (vertical_line_params[i][j][4] * vertical_line_params[i][j][6]/vertical_sum[i]);
            ver_z += (vertical_line_params[i][j][5] * vertical_line_params[i][j][6]/vertical_sum[i]);
        }
        vertical[i] << ver_x, ver_y, ver_z;
        vertical[i] = (vertical[i] - (vertical[i].dot(plane_norm[i]) * plane_norm[i] / pow(plane_norm[i].norm(),2))).normalized();
        cout << "Plane " << i+1 << " vertical vector:\t" << vertical[i].transpose() << endl;
    }


    Eigen::Vector3d line_direction;
    for (int i = 0; i < plane_size; i++){
        if (vertical_line_params[i].size() > 7 || horizontal_line_params[i].size() > 7){
            ROS_ERROR("Corner extraction failed! Please try again...");
            exit(1);
        }

        double parallel_min = 1; int ho_index, ver_index;
        if (horizontal_line_params[i].size() == 7){
            for (int j = 0; j < horizontal_line_params[i].size(); j++){
                line_direction << horizontal_line_params[i][j][3], horizontal_line_params[i][j][4], horizontal_line_params[i][j][5];
                auto parallel = horizontal[i].dot(line_direction);
                if (parallel < parallel_min) parallel_min = parallel; ho_index = j;
            }
            if (parallel_min < 0.99) horizontal_line_params[i].erase(horizontal_line_params[i].begin() + ho_index);
        }

        parallel_min = 1;
        if (vertical_line_params[i].size() == 7){
            for (int j = 0; j < vertical_line_params[i].size(); j++) {
                line_direction
                        << vertical_line_params[i][j][3], vertical_line_params[i][j][4], vertical_line_params[i][j][5];
                auto parallel = vertical[i].dot(line_direction);
                if (parallel < parallel_min) parallel_min = parallel; ver_index = j;
            }
            if (parallel_min < 0.99) vertical_line_params[i].erase(vertical_line_params[i].begin() + ver_index);

        }
    }

    // To remove coincide lines and the two edges of the board
    vector<int> idx_rm_hor, idx_rm_ver;
    for (int i = 0; i < plane_size; i++) {
        idx_rm_hor.clear();
        for (int j = 0; j < horizontal_line_params[i].size(); j++) {
            for (int k = j + 1; k < horizontal_line_params[i].size(); k++) {
                auto dis_p2l = dis_P2L_3d(horizontal_line_params[i][k][0], horizontal_line_params[i][k][1],
                                          horizontal_line_params[i][k][2],
                                          horizontal_line_params[i][j][0], horizontal_line_params[i][j][1],
                                          horizontal_line_params[i][j][2],
                                          horizontal[i][0], horizontal[i][1], horizontal[i][2]);
                if (dis_p2l < square_length / 3) {
                    idx_rm_hor.push_back(horizontal_line_params[i][j][6] < horizontal_line_params[i][k][6] ? j : k);
                }
                if (dis_p2l > board_length * 0.9) {
                    idx_rm_hor.push_back(j);
                    idx_rm_hor.push_back(k);
                }
            }
        }

        sort(idx_rm_hor.begin(), idx_rm_hor.end());
        for (int p = idx_rm_hor.size() - 1 ; p > - 1; p --){
            horizontal_line_params[i].erase(horizontal_line_params[i].begin() + idx_rm_hor[p]);
        }

        idx_rm_ver.clear();
        for (int t = 0; t < vertical_line_params[i].size(); t++) {
            for (int k = t + 1; k < vertical_line_params[i].size(); k++) {
                auto dis_p2l = dis_P2L_3d(vertical_line_params[i][k][0], vertical_line_params[i][k][1],
                                          vertical_line_params[i][k][2],
                                          vertical_line_params[i][t][0], vertical_line_params[i][t][1],
                                          vertical_line_params[i][t][2],
                                          vertical[i][0], vertical[i][1], vertical[i][2]);
                if (dis_p2l < square_length/3) {
                    idx_rm_ver.push_back(vertical_line_params[i][t][6] < vertical_line_params[i][k][6] ? t : k);
                }
                if (dis_p2l > board_length*0.9) {
                    idx_rm_ver.push_back(t);
                    idx_rm_ver.push_back(k);
                }
                //cout << "ve " << i << " " << t << " " << k << " " << dis_p2l << endl;
            }
        }

        sort(idx_rm_ver.begin(), idx_rm_ver.end());
        for (int p = idx_rm_ver.size() - 1 ; p > - 1; p --){
            vertical_line_params[i].erase(vertical_line_params[i].begin() + idx_rm_ver[p]);
        }
    }

    for (int i = 0; i < plane_size; i++){
        sort(horizontal_line_params[i].begin(), horizontal_line_params[i].end(), hor_sort);
        sort(vertical_line_params[i].begin(), vertical_line_params[i].end(), ver_sort);
//        for (int j = 0; j < horizontal_line_params[i].size(); j ++){
//            cout << "Plane " << i + 1 << " xyz = " << horizontal_line_params[i][j][2] << endl;
//        }
    }

    // Situation that only one side of the board is detected
    vector<Eigen::Vector3d> horizontal_points[plane_size], vertical_points[plane_size];
    for (int i = 0; i < plane_size; i++){
        if (vertical_line_params[i].size() == 5 ){
            vertical_line_params[i].erase(vertical_line_params[i].begin() +
                                          (vertical_line_params[i][0][6] > vertical_line_params[i][4][6] ? 0 : 4));
        }

        if (horizontal_line_params[i].size() == 5 ){
            horizontal_line_params[i].erase(horizontal_line_params[i].begin() +
                                            (horizontal_line_params[i][0][6] > horizontal_line_params[i][4][6] ? 0 : 4));
        }

        if (vertical_line_params[i].size() != 4 || horizontal_line_params[i].size() != 4 ){
            ROS_ERROR("Line extraction failed! Please try again...");
            exit(1);
        }

        for (int j = 0; j < horizontal_line_params[i].size(); j++){
            Eigen::Vector3d temp;
            temp << horizontal_line_params[i][j][0], horizontal_line_params[i][j][1],horizontal_line_params[i][j][2];
            horizontal_points[i].push_back(temp);
        }

        for (int j = 0; j < vertical_line_params[i].size(); j++){
            Eigen::Vector3d temp;
            temp << vertical_line_params[i][j][0], vertical_line_params[i][j][1],vertical_line_params[i][j][2];
            vertical_points[i].push_back(temp);
        }
    }
    cout << "Line detection done! Extracting corners..." << endl;

    /******************************
    ****** Corner Extraction ******
    *******************************/

    vector<Eigen::Vector3d> corners[plane_size];
    ofstream out_file(output_path.c_str());
    if (!out_file) {
        cout << "Can not open the output file" << endl;
        exit(0);
    }

    for (int i = 0; i < plane_size; i++){
        for (int j = 0; j < vertical_points[i].size(); j++){
            for (int k = 0; k < horizontal_points[i].size(); k++){
                Eigen::Vector3d temp = vertical_points[i][j] + (horizontal_points[i][k]-vertical_points[i][j]).cross(horizontal[i]).dot(vertical[i].cross(horizontal[i]))
                                                               / vertical[i].cross(horizontal[i]).dot(vertical[i].cross(horizontal[i])) * vertical[i];
                out_file << temp.x() << "   " << temp.y() << "   " << temp.z() << endl;
                corners[i].push_back(temp);
                point_plane.x = temp[0];point_plane.y = temp[1];point_plane.z = temp[2];
                marker_corners.points.push_back(point_plane);
            }
        }
    }

/*******************************
****** Line Visualization ******
********************************/

    visualization_msgs::Marker marker_result[plane_size*8];
    for (int i = 0; i < plane_size; i++){
        for (int j = 0; j < horizontal_line_params[i].size(); j++){
            SetMarker(marker_result[i*4+j], i*4+j, 4, 0.01, rand() % 255, rand() % 255, rand() % 255);
            point_plane.x = corners[i][j].x();
            point_plane.y = corners[i][j].y();
            point_plane.z = corners[i][j].z();
            marker_result[i*4+j].points.push_back(point_plane);
            point_plane.x = corners[i][j+12].x();
            point_plane.y = corners[i][j+12].y();
            point_plane.z = corners[i][j+12].z();
            marker_result[i*4+j].points.push_back(point_plane);
        }

        for (int j = 0; j < vertical_line_params[i].size(); j++){
            SetMarker(marker_result[i*4+j+4*plane_size], i*4+j+4*plane_size, 4, 0.01, rand() % 255, rand() % 255, rand() % 255);
            point_plane.x = corners[i][4*j].x();
            point_plane.y = corners[i][4*j].y();
            point_plane.z = corners[i][4*j].z();
            marker_result[i*4+j+4*plane_size].points.push_back(point_plane);
            point_plane.x = corners[i][4*j+3].x();
            point_plane.y = corners[i][4*j+3].y();
            point_plane.z = corners[i][4*j+3].z();
            marker_result[i*4+j+4*plane_size].points.push_back(point_plane);
        }
    }
    cout << "Corner extraction succeeded!" << endl;

/*************************************
****** Publishing to ROS & Rviz ******
**************************************/
    while (ros::ok()){
        pub_marker.publish(marker_pointcloud);
        pub_frontier.publish(marker_frontier);
        pub_filter.publish(marker_filter);
        pub_corner.publish(marker_corners);

        for (auto & i : marker_plane){
            pub_plane.publish(i);
        }

        for (auto & i : marker_line){
            pub_line.publish(i);
        }

        for (auto & i : marker_result){
            pub_result.publish(i);
        }

        ROS_INFO( "Publishing to rostopic! Please check the rviz...");

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



