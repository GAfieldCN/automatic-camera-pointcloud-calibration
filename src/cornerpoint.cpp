#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "geometry_msgs/Point.h"
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>

#include "common.h"

using namespace std;

ros::Publisher camera_points_pub;
geometry_msgs::Point point;
string photo_path;
IplImage* src=NULL;

void on_mouse( int event, int x, int y, int flags, void* ustc);


int main(int argc, char **argv)
{

    ros::init(argc, argv, "cornerpoint");
    ros::NodeHandle nh;
    ros::param::get("input_photo_path", photo_path);
    camera_points_pub = nh.advertise<geometry_msgs::Point>("/camera_selected_points", 10);
    src = cvLoadImage(photo_path.c_str(),1);

    cvNamedWindow("src",1);
    cvSetMouseCallback( "src", on_mouse, 0 );

    cvShowImage("src",src);
    cvWaitKey(0);
    cvDestroyAllWindows();
    cvReleaseImage(&src);

    return 0;
}


void on_mouse( int event, int x, int y, int flags, void* ustc)
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.3, 0.3, 0, 0.5, CV_AA);

    if( event == CV_EVENT_LBUTTONDOWN )
    {
        CvPoint pt = cvPoint(x,y);
        char temp[16];
        sprintf(temp,"(%d,%d)",pt.x,pt.y);
        point.x = pt.x;
        point.y = pt.y;
        point.z = 0;

        camera_points_pub.publish(point);
        //cvPutText(src,temp, pt, &font, cvScalar(0, 255, 255, 0));
        cvCircle( src, pt, 2,cvScalar(0,0,255,0) ,CV_FILLED, CV_AA, 0 );
        cvShowImage( "src", src );
    }
}