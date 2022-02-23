#include <ros/ros.h>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <string>
#include <numeric>
#include "common.h"
#include <set>

using namespace cv;
using namespace std;

string photo_path, output_name, intrinsic_path;
RNG random_number_generator;
RNG rng(12345);
cv::Mat gray_img, src_img;
float binary_threshold, min_area, max_area, rectangularity_preset, circularity_preset;
int plane_size;

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return (v[i1] < v[i2]); });
    return idx;
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_photo_path", photo_path)) {
        cout << "Can not get the value of input_photo_path" << endl;
        exit(1);
    }
    if (!ros::param::get("output_path", output_name)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("intrinsic_path", intrinsic_path)) {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("binary_threshold", binary_threshold)) {
        cout << "Can not get the value of binary_threshold" << endl;
        exit(1);
    }
    if (!ros::param::get("min_area", min_area)) {
        cout << "Can not get the value of min_area" << endl;
        exit(1);
    }
    if (!ros::param::get("max_area", max_area)) {
        cout << "Can not get the value of max_area" << endl;
        exit(1);
    }
    if (!ros::param::get("rectangularity_preset", rectangularity_preset)) {
        cout << "Can not get the value of rectangularity" << endl;
        exit(1);
    }
    if (!ros::param::get("circularity_preset", circularity_preset)) {
        cout << "Can not get the value of circularity_preset" << endl;
        exit(1);
    }
    if (!ros::param::get("plane_size", plane_size)) {
        cout << "Can not get the value of plane_size" << endl;
        exit(1);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_extraction_camera");
    ros::NodeHandle nh;
    getParameters();
    ros::Rate rate(10);

    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    vector<float> distortion;
    getDistortion(intrinsic_path, distortion);

    // set intrinsic parameters of the camera
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = intrinsic[0];
    cameraMatrix.at<double>(0, 2) = intrinsic[2];
    cameraMatrix.at<double>(1, 1) = intrinsic[4];
    cameraMatrix.at<double>(1, 2) = intrinsic[5];

    // set radial distortion and tangential distortion
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4];

    // Read image & Rectify
    Mat result_img = imread(photo_path);
    Mat srcImage0 = imread(photo_path,0);

    if(srcImage0.empty()) {  // use the file name to search the photo
        cout << "No Picture found by filename: " << photo_path << endl;
        return 0;
    }

    Mat result_view, review, result_map1, result_map2;
    cv::Size result_img_size = result_img.size();

    src_img = Mat::zeros(result_img_size.height, result_img_size.width, CV_8UC3);
    resize(srcImage0, src_img, src_img.size());

    cv::Mat view, rview, map1, map2;
    cv::Size imageSize = src_img.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion
    cv::namedWindow("Rectified Image");
    cv::imshow("Rectified Image", src_img);

    // Binary Process
    src_img = src_img > binary_threshold;
    imshow("Binary Image", src_img);

    // Find contours
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    vector<vector<Point>> contours, RectContours;
    findContours(src_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    vector<vector<Point>> hull(contours.size());
    Mat drawing(src_img.size(), CV_8UC3, cv::Scalar(0));
    vector<float> length(contours.size());
    vector<float> Area_contours(contours.size()), Area_hull(contours.size()), Rectangularity(contours.size()), circularity(contours.size());
    size_t i;

    // Filter contours through preset parameters
    for (i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        length[i] = arcLength(contours[i], true);
        if (length[i] > min_area && length[i] < max_area)
        {
            convexHull(Mat(contours[i]), hull[i], false);
            Area_contours[i] = contourArea(contours[i]);
            Area_hull[i] = contourArea(hull[i]);
            Rectangularity[i] = Area_contours[i] / Area_hull[i];
            circularity[i] = (4 * 3.1415*Area_contours[i]) / (length[i] * length[i]);

            if (Rectangularity[i] > rectangularity_preset && circularity[i] < circularity_preset)
            {
                RectContours.push_back(hull[i]);
                drawContours(drawing, hull, i, color, 1);
            }
        }
    }

    float distance, distanceMax;
    Point cornerPoint1, cornerPoint2, cornerPoint3, cornerPoint4, point_add;
    vector <Point> cornerPoint4_Candidates(3);
    size_t conP_i1 = 0, conP_i2 = 0, conP_i3 = 0, conP_i_add, flag = 0;
    vector <vector<size_t> > sorted_corners(RectContours.size());
    vector <Point2f> final_corners;
    vector <vector<Point2f> > results;
    vector <vector<Point2f> > planes[plane_size];
    vector<Point2f> result[plane_size];


    /***************************
     ** Main Loop: Detect corners **
     ****************************/
    for (size_t j = 0; j < RectContours.size(); j++)
    {
        // Find the first corner
        distanceMax = 0;
        for (i = 0; i < RectContours[j].size(); i++)
        {
            distance = getDistance(RectContours[j][i], RectContours[j][0]);
            if (distance > distanceMax)
            {
                distanceMax = distance;
                cornerPoint1 = RectContours[j][i];
                conP_i1 = i;
            }
        }

        // Find the second corner
        distanceMax = 0;
        for (i = 0; i < RectContours[j].size(); i++)
        {
            distance = getDistance(RectContours[j][i], cornerPoint1);
            if (distance > distanceMax)
            {
                distanceMax = distance;
                cornerPoint2 = RectContours[j][i];
                conP_i2 = i;
            }
        }

        // Find the third corner
        distanceMax = 0;
        for (i = 0; i < RectContours[j].size(); i++)
        {
            distance = getDistance(RectContours[j][i], cornerPoint1) + getDistance(RectContours[j][i], cornerPoint2);
            if (distance>distanceMax)
            {
                distanceMax = distance;
                cornerPoint3 = RectContours[j][i];
                conP_i3 = i;
            }
        }

        // List the three corners
        flag = list_connor(conP_i1, conP_i2, conP_i3);
        switch (flag)
        {
            case 0:break;
            case 123:break;
            case 132:point_add = cornerPoint2; cornerPoint2 = cornerPoint3; cornerPoint3 = point_add; break;
            case 213:point_add = cornerPoint1; cornerPoint1 = cornerPoint2; cornerPoint2 = point_add; break;
            case 231:point_add = cornerPoint1; cornerPoint1 = cornerPoint2; cornerPoint2 = point_add;
                point_add = cornerPoint2; cornerPoint2 = cornerPoint3; cornerPoint3 = point_add; break;
            case 321:point_add = cornerPoint3; cornerPoint3 = cornerPoint1; cornerPoint1 = point_add; break;
            case 312:point_add = cornerPoint3; cornerPoint3 = cornerPoint1; cornerPoint1 = point_add;
                point_add = cornerPoint2; cornerPoint2 = cornerPoint3; cornerPoint3 = point_add; break;
        }
        switch (flag)
        {
            case 0:break;
            case 123:break;
            case 132:conP_i_add = conP_i2; conP_i2 = conP_i3; conP_i3 = conP_i_add; break;
            case 213:conP_i_add = conP_i1; conP_i1 = conP_i2; conP_i2 = conP_i_add; break;
            case 231:conP_i_add = conP_i1; conP_i1 = conP_i2; conP_i2 = conP_i_add;
                conP_i_add = conP_i2; conP_i2 = conP_i3; conP_i3 = conP_i_add; break;
            case 321:conP_i_add = conP_i3; conP_i3 = conP_i1; conP_i1 = conP_i_add; break;
            case 312:conP_i_add = conP_i3; conP_i3 = conP_i1; conP_i1 = conP_i_add;
                conP_i_add = conP_i2; conP_i2 = conP_i3; conP_i3 = conP_i_add; break;
        }

        // Find the candidates of the fourth corner
        distanceMax = 0;
        for (i = conP_i3; i < conP_i2; i++)
        {
            distance = getDistance(RectContours[j][i], cornerPoint3) + getDistance(RectContours[j][i], cornerPoint2);
            if (distance > distanceMax)
            {
                distanceMax = distance;
                cornerPoint4_Candidates[0] = RectContours[j][i];
            }
        }

        distanceMax = 0;
        for (i = conP_i2; i < conP_i1; i++)
        {
            distance = getDistance(RectContours[j][i], cornerPoint1) + getDistance(RectContours[j][i], cornerPoint2);
            if (distance > distanceMax)
            {
                distanceMax = distance;
                cornerPoint4_Candidates[1] = RectContours[j][i];
            }
        }

        distanceMax = 0;
        for (i = conP_i1; i < RectContours[j].size() + conP_i3; i++)
        {
            if (i < RectContours[j].size())
            {
                distance = getDistance(RectContours[j][i], cornerPoint1) + getDistance(RectContours[j][i], cornerPoint3);
                if (distance>distanceMax)
                {
                    distanceMax = distance;
                    cornerPoint4_Candidates[2] = RectContours[j][i];
                }
            }
            else
            {
                distance = getDistance(RectContours[j][i - RectContours[j].size()], cornerPoint1) + getDistance(RectContours[j][i - RectContours[j].size()], cornerPoint3);
                if (distance > distanceMax)
                {
                    distanceMax = distance;
                    cornerPoint4_Candidates[2] = RectContours[j][i - RectContours[j].size()];
                }
            }
        }

        // Determine the fourth corner
        if (getDist_P2L(cornerPoint4_Candidates[0], cornerPoint3, cornerPoint2)>5)
        {
            cornerPoint4 = cornerPoint4_Candidates[0];
        }
        else if (getDist_P2L(cornerPoint4_Candidates[1], cornerPoint2, cornerPoint1)>5)
        {
            cornerPoint4 = cornerPoint4_Candidates[1];
        }
        else if (getDist_P2L(cornerPoint4_Candidates[2], cornerPoint1, cornerPoint3)>5)
        {
            cornerPoint4 = cornerPoint4_Candidates[2];
        }

        // Draw the corner points
        circle(drawing, cornerPoint1, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
        circle(drawing, cornerPoint2, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
        circle(drawing, cornerPoint3, 3, Scalar(255, 255, 255), FILLED, LINE_AA);
        circle(drawing, cornerPoint4, 3, Scalar(255, 255, 255), FILLED, LINE_AA);

        Point verticess[4];
        Point tempPoint;

        verticess[0] = cornerPoint1;
        verticess[1] = cornerPoint2;
        verticess[2] = cornerPoint3;
        verticess[3] = cornerPoint4;


        // List the 4 points counterclockwise with the first point in the upper left
        for (size_t a = 0; a < 3; a++)
        {
            for (size_t b = a + 1; b < 4; b++)
            {
                if (verticess[a].x > verticess[b].x)
                {
                    tempPoint = verticess[a];
                    verticess[a] = verticess[b];
                    verticess[b] = tempPoint;
                }
            }
        }

        if (verticess[0].y > verticess[1].y)
        {
            tempPoint = verticess[0];
            verticess[0] = verticess[1];
            verticess[1] = tempPoint;
        }
        if (verticess[2].y > verticess[3].y)
        {
            tempPoint = verticess[2];
            verticess[2] = verticess[3];
            verticess[3] = tempPoint;
        }

        // Get the unsorted results
        final_corners.clear();
        final_corners.push_back(verticess[0]);
        final_corners.push_back(verticess[1]);
        final_corners.push_back(verticess[3]);
        final_corners.push_back(verticess[2]);

        size_t unsorted_x = (final_corners[0].x + final_corners[1].x + final_corners[2].x + final_corners[3].x) / 4;
        size_t unsorted_y = (final_corners[0].y + final_corners[1].y + final_corners[2].y + final_corners[3].y) / 4;
        sorted_corners[j].push_back(unsorted_x);
        sorted_corners[j].push_back(unsorted_y);

        results.push_back(final_corners);
    }


    // Get sorted index
    auto sorted_index = sort_indexes<vector<size_t>>(sorted_corners);

    for (i = 0; i < results.size(); i++){
        char label[5];
        sprintf(label,"%zu", i+1);
        Point center;
        center.x = results[sorted_index[i]][1].x + 3;
        center.y = results[sorted_index[i]][1].y - 5;
        putText(drawing,label, center,FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,0,255), 0.5, 7);
    }

    imshow("corner points", drawing);
    cout << "Please tap any key in the photo windows" << endl;
    waitKey(0);

    // Debug process: remove incorrect squares
    cout << "Please enter the sequence of the squares you want to remove and press enter, finish by typing 0" << endl;
    cout << "If the squares are correct, typing 0 to continue.." << endl;

    size_t remove_num;
    vector<size_t> remove_nums;
    vector<size_t> label_remove_nums;

    while (1) {
        cin >> remove_num;
        if (remove_num == 0) {break;}
        remove_nums.push_back(remove_num);
        label_remove_nums.push_back(sorted_index[remove_num - 1]);
    }

    if (remove_nums.size() != 0){
        sort(label_remove_nums.begin(), label_remove_nums.end());

        for (int p = label_remove_nums.size() - 1 ; p > - 1; p --){
            results.erase(results.begin() + label_remove_nums[p]);
            sorted_corners.erase(sorted_corners.begin() + label_remove_nums[p]);
        }
        sorted_index = sort_indexes<vector<size_t>>(sorted_corners);
    }

    if (plane_size*4 != results.size()) ROS_WARN("Contour size does not match plane size!");

    // List the corners by u v axis
    for (i = 0; i < plane_size; i++){
        planes[i].push_back(results[sorted_index[4*i]]);planes[i].push_back(results[sorted_index[4*i+1]]);
        planes[i].push_back(results[sorted_index[4*i+2]]);planes[i].push_back(results[sorted_index[4*i+3]]);
        result[i].push_back(planes[i][0][0]);result[i].push_back(planes[i][0][1]);
        result[i].push_back(planes[i][1][0]);result[i].push_back(planes[i][1][1]);
        result[i].push_back(planes[i][0][3]);result[i].push_back(planes[i][0][2]);
        result[i].push_back(planes[i][1][3]);result[i].push_back(planes[i][1][2]);
        result[i].push_back(planes[i][2][0]);result[i].push_back(planes[i][2][1]);
        result[i].push_back(planes[i][3][0]);result[i].push_back(planes[i][3][1]);
        result[i].push_back(planes[i][2][3]);result[i].push_back(planes[i][2][2]);
        result[i].push_back(planes[i][3][3]);result[i].push_back(planes[i][3][2]);

    }


    // Write data
    ofstream outfile(output_name.c_str());
    if (!outfile) {
        cout << "Can not open the output file" << endl;
        exit(0);
    }

    // Get precise & sorted corner points
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, result_img_size, 1, result_img_size, 0), result_img_size, CV_16SC2, result_map1, result_map2);
    cv::remap(result_img, result_img, result_map1, result_map2, cv::INTER_LINEAR);  // correct the distortion
    cv::Size winSize = cv::Size(5, 5);
    cv::Size zerozone = cv::Size(-1, -1);
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);

    cv::namedWindow("output");
    cv::cvtColor(result_img, gray_img, cv::COLOR_BGR2GRAY);

    for (i = 0; i < plane_size; i++) {
        cv::cornerSubPix(gray_img, result[i], winSize, zerozone, criteria);
        cout << "Plane" << i + 1 << " precise sorted corner points: " << endl;
        for (size_t q = 0; q < result[i].size(); ++q){
            cv::circle(result_img, result[i][q], 3,
                       cv::Scalar(random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255),
                                  random_number_generator.uniform(0, 255)), 1, 8, 0);
            printf("(%.3f %.3f)", result[i][q].x, result[i][q].y);
            cout << " " << endl;
            outfile << float2str(result[i][q].x) << "    " << float2str(result[i][q].y) << endl;
        }
    }

    cv::namedWindow("output");
    imshow("output", result_img);

    cout << endl << "Result saved in " << output_name << endl;
    cout << "Please tap any key to exit.." << endl;
    cv::waitKey(0);

    return 0;
}
