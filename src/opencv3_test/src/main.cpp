
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <typeinfo>

image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	/*cv::Mat jpegData(1,msg->data.size(),CV_8UC1);
    jpegData.data     = const_cast<uchar*>(&msg->data[0]);
    cv::InputArray data(jpegData);
    cv::Mat bgrMat     = cv::imdecode(data,cv::IMREAD_COLOR);*/
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

	
	//cv::Rect cmlsROI(0,400,cv_ptr->image.cols,1);

	cv::Mat mat = cv_ptr->image;
	
	//const cv::Mat* src_g=cv_ptr->image;


	if (mat.rows > 60 && mat.cols > 60){
		cv::circle(mat, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	}

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

	image_transport::ImageTransport it(nh);
	pub = it.advertise("usb_cam/image_filtered", 1);
	image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
	ros::spin();
	return 0;
}