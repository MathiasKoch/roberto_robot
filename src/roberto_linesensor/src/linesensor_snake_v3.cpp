#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "roberto_msgs/Line.h"
#include "roberto_msgs/MotorState.h"
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
int height, width;
float pixel2Angle = 0.13;


bool black = false;
bool goRight = 1;


image_transport::Publisher pub;
ros::Publisher motor_pub;

float speed = 0.3;

int lostCounter = 0;
float lostSpinSpeed = 0.2;
int lineLastSeen = 0;
bool running = true;


// Prototypes
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

void activateCallback(const std_msgs::Float32& msg){
    speed = msg.data;
    if(speed > 0)
        running = true;
    else
        running = false;
    cout << "Speed: " << speed << endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    if(!running){
        return;
    }
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat image = cv_ptr->image;
    height = (int)(image.rows * 0.8);
    width = image.cols;


    // Segment line
    cv::Rect roi(0, image.rows/2,image.cols,image.rows/2);
    image = image(roi);
    cv::cvtColor(image, image, CV_BGR2GRAY);
    
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(image, mean, stddev );
    if (black){
        cv::adaptiveThreshold(image, image,255,CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV ,101, stddev[0]);
    }else{
        cv::adaptiveThreshold(image, image,255,CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY ,101, -stddev[0]);
    }
    cv::erode(image, image, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(8,8)));
    cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(8,8)));
    cv::resize(image, image, cv::Size(image.cols/4, image.rows/4),5,0, CV_INTER_NN);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    int largest_area = -1;
    int largest_contour_index = 0;
    cv::Rect bounding_rect;
    vector<vector<Point> >hull(1);
    if(contours.size()){
        for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
        {
            double a = contourArea( contours[i] , false);  //  Find the area of contour
            if(a > largest_area){
                largest_area=a;
                largest_contour_index=i;                //Store the index of largest contour
            }
        }      
        bounding_rect = boundingRect(contours[largest_contour_index]);
        cv::convexHull( cv::Mat(contours[largest_contour_index]), hull[0], false );
        //cout << endl << largest_area << endl;
    }



    

    // change to bgr for plotting
    cv::cvtColor(image, image, CV_GRAY2BGR);

   

    float driveTowards = 0;
    float speedMultiplier = 1;
    //cout << "@ " << upperPoint << endl;
    //50 is magic size constant of minimum line size
    if(largest_area > 50 && contours.size()){
        //Draw on image for debugging
        drawContours( image, contours, largest_contour_index, cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy); // Draw the largest contour using previously stored index.
        rectangle(image, bounding_rect,  Scalar(0,255,0),1, 8,0);
        drawContours( image, hull, 0, cv::Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );

        //compensation terms
        int lowerPoint = 0;
        float upperPoint = 0;

        // drive towards the outer boundary to the followed side if bounding rect larger than constant otherwise drive towards center
        if (bounding_rect.width > 12){
            upperPoint = bounding_rect.x;
            upperPoint += goRight ? bounding_rect.width : 0;
            
            int cnt = 0;
            for(cv::Point h : hull[0]){
                if(h.y <= bounding_rect.y + 1){
                    if((goRight && h.x < upperPoint) || (!goRight && h.x > upperPoint)){
                        upperPoint = h.x;
                    }
                }
                if(h.y >= bounding_rect.y + bounding_rect.height - 1){
                    cnt++;
                    lowerPoint = lowerPoint + h.x;
                }
            }
            // Average h.x
            if(cnt>0){
                lowerPoint /= cnt;
            }
        }else{
            upperPoint = bounding_rect.x + bounding_rect.width/2.0;
            lowerPoint = upperPoint;
        }
        // Draw blue dot for upperPoint and Yellow dot for LowerPoint
        image.at<cv::Vec3b>(0, upperPoint) = cv::Vec3b(255,0,0);
        image.at<cv::Vec3b>(0, lowerPoint) = cv::Vec3b(0,255,255);

        //rescale from downsized image
        float alpha = 0.8;
        
        float deltaX = upperPoint - lowerPoint;
        driveTowards = alpha*upperPoint + (1-alpha)*lowerPoint;
        if((deltaX > 0 && lowerPoint < image.cols/2) || (deltaX < 0 && lowerPoint > image.cols/2))
        {
            // Deadband in center of image
            if((upperPoint > image.cols/2 && lowerPoint < image.cols/2) || (upperPoint < image.cols/2 && lowerPoint > image.cols/2))
            {
                driveTowards = image.cols/2;
            }else{
                driveTowards = upperPoint;
            }
        }

        // Draw turqouise dot for driveTowards
        image.at<cv::Vec3b>(0, driveTowards) = cv::Vec3b(255,255,0);

        /*Vec4f lines;
        cv::fitLine(cv::Mat(contours[largest_contour_index]),lines,CV_DIST_L2,0,0.01,0.01);
        int lefty = (-lines[2]*lines[1]/lines[0])+lines[3];
        int righty = ((image.cols-lines[2])*lines[1]/lines[0])+lines[3];
        cv::line(image,Point(image.cols-1,righty),Point(0,lefty),Scalar(255,0,0),1);*/

        //cv::Mat dt;
        //cv::distanceTransform(cv::Mat(contours[largest_contour_index]),dt,CV_DIST_L1,CV_DIST_MASK_PRECISE);



        
        //driveTowards in original image pixels
        driveTowards = (driveTowards - image.cols/2)*4;

        //rescale between 0.6 and 1
        speedMultiplier = std::min(std::max(((image.cols*4)-abs(driveTowards))/(image.cols*4.0f), 0.5f), 1.0f);

        //cout << "lefty " << lefty << ",righty  " << righty<< endl;

        //Keep track of line
        lostCounter = 0;
        lineLastSeen = driveTowards;
    }else{
        // No line detected
        ROS_INFO("No line found");
        lostCounter++;
    } 
    
    //cout << "Angle " << -driveTowards*pixel2Angle << endl;
    //cout << "speedMultiplier " << speedMultiplier << endl;

    if(speed != 0){
        if(lostCounter < 3){
            roberto_msgs::MotorState motor_msg;
            motor_msg.speed = speed*speedMultiplier;
            motor_msg.heading_angle = -driveTowards*pixel2Angle;
            motor_msg.mode = motor_msg.DRIVE_MODE_PIVOT;
            motor_pub.publish(motor_msg);
        }else{
            // Search for line
            roberto_msgs::MotorState motor_msg;
            motor_msg.speed = ((lineLastSeen >= 0) ? -1 : 1) * 0.27;
            motor_msg.heading_angle = 0;
            motor_msg.mode = motor_msg.DRIVE_MODE_SPIN;
            motor_pub.publish(motor_msg);
        }
    }

    cv_ptr->image = image;

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    //  cout << "timing: " << duration << " uS" << endl;

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
    motor_pub = nh.advertise<roberto_msgs::MotorState>("cmd_vel",1);
    //pubS = it.advertise("usb_cam/sampled", 1);
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    ros::Subscriber sub_sm = nh.subscribe("linesensor_active", 1, activateCallback);
    //first_scanpoint = Point(FRAME_WIDTH/2, scan_height_reg);
    //debug_mode=1;
    ros::spin();
    return 0;
}


