#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <typeinfo>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>



#define PI 3.14159265


using namespace cv;
using namespace std;

image_transport::Publisher pub;

// Debug settings variables.
bool show_images = 1;	// Show images or not.
bool debug_mode;		// Variable to regulate debugging inside program.
//bool camera_show = 0;	// Show camera feed or not.

// Settings for object tracking and drawing.
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;

bool black = true;
cv::Mat kernel(3, 3, CV_8UC1, 1);


void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, "mono8");

	cv::Mat image = cv_ptr->image;

	cv::Rect roi = cv::Rect(0, image.rows/2, image.cols, image.rows/2);
	cv::Mat image_roi = image(roi);

	cv::resize(image_roi, image, cv::Size(), 0.25, 0.25);

    int threshType = black ? (cv::THRESH_BINARY_INV | cv::THRESH_OTSU) : (cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    cv::threshold(image, image, 0, 255, threshType);


	cv::erode(image, image, kernel );	
	
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	cv::Mat temp(image.size(), CV_8UC1);
	cv::Mat skel(image.size(), CV_8UC1, cv::Scalar(0));

	bool done;
	do
	{
		cv::morphologyEx(image, temp, cv::MORPH_OPEN, kernel);
		cv::bitwise_not(temp, temp);
		cv::bitwise_and(image, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		cv::erode(image, image, kernel);

		double max;
		cv::minMaxLoc(image, 0, &max);
		done = (max == 0);
	} while (!done);

	cv::filter2D( skel, skel, -1 , kernel, Point( -1, -1 ), 0, BORDER_DEFAULT );

	
	cv_ptr->image = skel;
	pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;

	if ( CV_MAJOR_VERSION == 2){
		ROS_ERROR("CV_MAJOR_VERSION == 2");
		return 0;
	}

	ROS_INFO("Started line sensor");

	image_transport::ImageTransport it(nh);
	pub = it.advertise("usb_cam/image_filtered", 1);
	image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
	debug_mode=1;
	ros::spin();
	return 0;
}