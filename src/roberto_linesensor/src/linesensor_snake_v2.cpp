#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;

int thetaSteps = 360;
int rSteps = 1;
int rStepSize = 0;
int radius = 70;

cv::Mat sampled(rSteps, thetaSteps, CV_8UC3);

bool black = false;
int threshType = black ? (cv::THRESH_BINARY_INV | cv::THRESH_OTSU) : (cv::THRESH_BINARY | cv::THRESH_OTSU);   
cv::Mat kernel(5, 5, CV_8UC1, 1);


void imageCallback(const sensor_msgs::ImageConstPtr& msg){

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");


    image = cv_ptr->image;

    sample(image);


    pub.publish(cv_ptr->toImageMsg());
}


vector<cv::Point> findLine(cv::Mat sampled, bool wrap){    

    cv::threshold(sampled, sampled, 0, 255, threshType);
    cv::morphologyEx(sampled, sampled,2,kernel);

    vector<cv::Point> lines; 

    int segmentStart = 0;

    uint8_t old = 255;// sampled.at<uint8_t>(0,thetaSteps-1);
    uint8_t value = 0;
    for(int i = 0 ; i< sampled.cols ; i++){

        uint8_t value = sampled.at<uint8_t>(0,i);
        if (value != old){
            //new segment
            if(value == 255){
                int segmentEnd = i-1;
                //Wrap around
                if (segmentEnd < segmentStart){
                    segmentEnd += sampled.cols;
                }

                lines.push_back(cv::Point(segmentStart , segmentEnd));
                
                //sampled.at<uint8_t>(0,(int)(segmentCenterAngle)) = 128;

            }else{
                segmentStart = i;
            }
        }
        old = value;
    }
    if (wrap && lines.size() > 0 && value == 0 && lines[0].x == 0){
        lines[0] = cv::Point(segmentStart, lines[0].y + sampled.cols);
        //cout << "Wrapping Line" << endl;
    }

    return lines;
}

void sampleCircle(cv::Mat image, cv::Point center){  
    //int oldPx = -1;
    //int oldPy = -1;
    sampled = cv::Mat(rSteps, thetaSteps, CV_8UC3);
    float thetaStepsize = 2*M_PI/thetaSteps;
    
    for(int i = 0 ; i < rSteps ; i++)
    {   
        for(int j = 0 ; j<thetaSteps ; j++){
            float t = j*thetaStepsize;
            float r = radius+i*rStepSize;
            int pX = center.x + (r)*cos(t);
            int pY = center.y + (r)*sin(t);
            
            sampled.at<cv::Vec3b>(i,j) = image.at<cv::Vec3b>(pY,pX);
            //image.at<cv::Vec3b>(pY,pX) = 0;

            /*if (!(pX==oldPx && pY == oldPy)){                
            }else{
                cout << "Pixel sampled twice" << endl;
            }
            oldPx = pX;
            oldPy = pY;
            */
        }
    }
    
    cv::cvtColor(sampled, sampled, CV_BGR2GRAY);



    cv::circle(image, center, radius+rSteps*rStepSize, CV_RGB(255,255,0));
    
}

void sampleLine(cv::Mat image){
    cv::Rect lineROI(0,400,image.cols,1);
    sampled = image(lineROI);
    cv::cvtColor(sampled, sampled, CV_BGR2GRAY);
}

void followLine(cv::Mat image, cv::Point center, cv::Point lastPoint, int depthLeft){

    sampleCircle(image, center);
    
    auto lines = findLine(sampled, true);
    //cout << "size: " << lines.size() << endl; 
    for(cv::Point line : lines){
        float segmentCenterAngle = ((line.x+line.y)/2)*(2*M_PI)/thetaSteps; // Scaling for thetaSteps?
        //int segmentSize = segmentEnd-segmentStart;
        int r = radius + (rSteps*rStepSize/2);
        int x = center.x + r*cos(segmentCenterAngle);
        int y = center.y + r*sin(segmentCenterAngle);
        cv::circle( image, cv::Point(x,y), 8, CV_RGB( 255, 0, 0 ), 1);

        float dist = (x-lastPoint.x)*(x-lastPoint.x) + (y-lastPoint.y)*(y-lastPoint.y);
        //cout << "y: " << y << endl;
        //cout << "dist: " << dist << endl;
        if(depthLeft > 0 && y < 400 && dist > r*r*0.5){
            followLine(image, cv::Point(x,y), center, depthLeft-1);
        }
    }
}

void sample(cv::Mat image){
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    
    sampleLine(image);
    auto lines = findLine(sampled, false);
    for(cv::Point line : lines){
        cv::line( image, cv::Point(line.x, 400), cv::Point(line.y, 400), CV_RGB( 255, 0, 0 ), 1);
    }
    //cout << "" << lines << endl;

    //Select Line
    int lineselect = 0;

    //cv::line( image, cv::Point(lines[lineselect].x, 400), cv::Point(lines[lineselect].y, 400), CV_RGB( 255, 0, 0 ), 1);
    auto center = cv::Point((lines[lineselect].x + lines[lineselect].y)/2, 400);
    followLine(image, center, center, 2);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    //cout << "timing: " << duration << " uS" << endl;
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
    //first_scanpoint = Point(FRAME_WIDTH/2, scan_height_reg);
    //debug_mode=1;
    ros::spin();
    return 0;
}


