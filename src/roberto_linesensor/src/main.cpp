
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


int k = 0;
image_transport::Publisher pub;
std::vector<std::vector<cv::Point> > contours;

std::vector<cv::Point> lines_prev;
std::vector<double> a_prev;

int endOfLine = 0;
int imgError = 0;
bool err = false;

int angle = 0;
int anglePrev = 0;


    
int frames = 0;
int currFrameCnt = 0;

bool black = true;
bool turnLeft = false;

void errorContour(int im){
    if (k == 0){
        endOfLine = endOfLine + 1;

        err = true;
        	
        //ROS_ERROR("** Contour error **");
        
        if (im - imgError >= 4){
            endOfLine = 0;
        }
        imgError = im;
    }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg){


	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");


	cv::Mat img = cv_ptr->image;

	cv::Rect roi = cv::Rect(10, img.rows-60, img.cols-20, img.rows/12);

	cv::Mat img_roi = img(roi);

    // convert to gray
    cv::Mat img_gray;
    cv::cvtColor(img_roi, img_gray, cv::COLOR_BGR2GRAY);
    
    // Blur
    cv::Mat img_blur;
    cv::GaussianBlur(img_gray, img_blur, cv::Size(9, 9), 2);
        

    // Threshold
    cv::Mat img_thresh;

    int threshType;
    threshType = black ? (cv::THRESH_BINARY_INV | cv::THRESH_OTSU) : (cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    cv::threshold(img_blur, img_thresh, 0, 255, threshType);
        
    
    // Opening (reduces noise)
    cv::Mat kernel(5, 5, CV_8UC1, 1);
    cv::Mat img_open;

    cv::morphologyEx(img_thresh,img_open,2,kernel);

    // find contours
    
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(img_open,contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    
    std::vector<cv::Rect> r;
    std::vector<cv::Point> lines;
    std::vector<double> a;
    
    cv::Point midRect;

    int index = 0;

    k = 0;
    
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
    {
        double area = cv::contourArea( contours[i],false);  //  Find the area of contour
        // use only contours with area above 3500
        if(area > 3500)
        {
        	a.push_back(area);
            // get bounding rec of contour
            r.push_back(cv::boundingRect(contours[i]));
            r[k].x = r[k].x + 10;
            r[k].y = r[k].y + img.rows-60;
            
            cv::rectangle(img,r[k],cv::Scalar(0,255,0));
             
            // get mid-point of rect
            midRect = cv::Point(r[k].x+r[k].width/2, 
                    r[k].y+r[k].height/2);

            cv::circle(img,midRect,3,cv::Scalar(0,0,255),-1,8,0);

            lines.push_back( midRect);
            
            k = k+1;
            
            cv::drawContours(img,contours, i, cv::Scalar(0,255,255), 1, 8, hierarchy, 0, cv::Point(10,img.rows-60));

        
            if(lines_prev.size() > 0){
            	double score = 1e20;
            	for(int j = 0; j < lines_prev.size(); j++){
            		double score_ = abs(a.back() - a_prev[j]) * 1 + abs(midRect.x - lines_prev[j].x) * 1;
            		if(score_ < score){
        				index = j;
        				score = score_;
   					}
            	}
            	std::string text = std::to_string(score);
            	cv::putText(img, text, lines.back(), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255,0,255));
            	//cv::circle(img,lines_prev[index],3,cv::Scalar(255,0,255),-1,8,0);
            }


            if(a_prev.size() > index && a.back() - a_prev[index] > 2000){
            	ROS_INFO("CROSSING LINE");
            }
        }
    }


    lines_prev = lines;
    a_prev = a;

    // if k = 0 no contour was found -> error
    errorContour(currFrameCnt);

    /*
    if (err)
    {
        char errStr2[50];
        sprintf(errStr2,"../img/imgERR%d.jpg",im);
        imwrite(errStr2,img);
        printf("saved");
        
    }
    */  
    
    // error routine
    if (endOfLine == 4){
        ROS_INFO("### REACHED END OF LINE ### ( or made an error )");
    }else{
        
    
    

	    // get the edge points
	    int leftmostEdge = img.cols;
	    
	    for (int i = 0; i < lines.size(); i++){
	        int edge = lines[i].x;
	                                
	        if (edge < leftmostEdge){
	            leftmostEdge = edge;
	        }
	    }
	    
	    int rightmostEdge = 0;
	    
	    for (int i = 0; i < lines.size(); i++){
	        int edge = lines[i].x;
	        
	        if (edge > rightmostEdge){
	            rightmostEdge = edge;
	        }
	    }
	    
	            
	    // calculate angle
	    double deltaX = 0;
	    if (turnLeft)
	    {
	        deltaX = leftmostEdge - img.cols/2;
	    }
	    else
	    {
	        deltaX = rightmostEdge - img.cols/2;
	    }
	            
	    double deltaY = midRect.y - img.rows;


	    int refAngle = atan(deltaY/deltaX) * 180/M_PI;

	    // convert from img coordinates to regbot coordinates
	    refAngle = -(refAngle-90);

	    if (refAngle > 90)
	        refAngle = refAngle - 180;

	    ROS_INFO("%d",refAngle);
	}

	
    currFrameCnt++;


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