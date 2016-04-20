#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <chrono>


image_transport::Publisher pub;


int hueMin = 120;
int satMin = 50;
cv::Mat segment;

ros::Publisher pubRegbot;

// Prototypes
float segment_regbot(cv::Mat image);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    float regbot = segment_regbot(cv_ptr->image);
    ROS_ERROR("Regbot: %f", regbot);
    /*std_msgs::Float32 msg_out;
    msg_out.data = regbot;
    pubRegbot.publish(msg_out);*/


    

    /*cv_bridge::CvImagePtr cv_ptrS;
    cv_ptrS->image = sampled;
    pubS.publish(cv_ptrS->toImageMsg());*/
}

float segment_regbot(cv::Mat image){
    //high_resolution_clock::time_point t1 = high_resolution_clock::now();

    cv::Mat channel;

    cv::resize(image, image, cv::Size(image.cols/4, image.rows/4),5,0, CV_INTER_NN);
    cv::cvtColor(image, image, CV_BGR2HLS);

    channel = cv::Mat(image.size(), CV_8U);
    cv::extractChannel(image, channel, 0);
    cv::threshold(channel, segment, hueMin, 255, cv::THRESH_BINARY);

    cv::extractChannel(image, channel, 2);
    cv::threshold(channel, channel, satMin, 255, cv::THRESH_BINARY);
    cv::bitwise_and(channel, segment, segment);

    int cnt = 0;
    for( int y = 0; y < segment.rows; y++ ) {
        for( int x = 0; x < segment.cols; x++ ) {
            if ( segment.at<uchar>(y,x) != 0 )
                cnt++  ;
        }
    }

    cv::cvtColor(channel, channel, CV_GRAY2BGR);

    cv_bridge::CvImagePtr ptr;

    ptr->image = channel;
    pub.publish(ptr->toImageMsg());

    return ((float)cnt)/(segment.rows*segment.cols);
    
    /*high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    cout << "timing: " << duration << " uS" << endl;*/

    
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
    pub = it.advertise("usb_cam/regbot", 1);
    
    pubRegbot = nh.advertise<std_msgs::Float32>("regbot_sensor/regbot", 1);
    
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    ros::spin();
    return 0;
}


