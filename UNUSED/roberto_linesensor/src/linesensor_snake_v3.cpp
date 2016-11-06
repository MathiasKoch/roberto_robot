#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
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

class Line
{
    public:
        int id = 0;
        int start = 0;
        int end = 0;
        int center = 0;
        int width = 0;
};

int thetaSteps = 360;
int rSteps = 1;
int rStepSize = 0;
int radius = 70;
int height, width;
float pixel2Angle = 0.13;

std::vector<Line*> lines;
std::vector<Line*> oldLines;
int maxLineId = 0;

cv::Mat sampled(rSteps, thetaSteps, CV_8UC3);

bool black = true;
image_transport::Publisher pub;
ros::Publisher line_pub;
ros::Publisher motor_pub;

int linesensor_angle;
float speed = 0.35;

// Prototypes
void findLine(cv::Mat sampled, bool wrap);
void sample(cv::Mat image);
void followLine(cv::Mat image, cv::Point center, cv::Point lastPoint, int depthLeft);
void sampleLine(cv::Mat image);
void sampleCircle(cv::Mat image, cv::Point center);
void findLine(cv::Mat sampled, bool wrap);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);


void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat image = cv_ptr->image;
    height = (int)(image.rows * 0.8);
    width = image.cols;
    //sample(image);

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

        cout << endl << contours[largest_contour_index] << endl;
    }
 


    cv::cvtColor(image, image, CV_GRAY2BGR);
    
    if(contours.size()){
        drawContours( image, contours, largest_contour_index, cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy); // Draw the largest contour using previously stored index.
        rectangle(image, bounding_rect,  Scalar(0,255,0),1, 8,0);
    }

    cv_ptr->image = image;
    


    /*roberto_msgs::Line linemsg;
    linemsg.length = lines.size();
    
    for(auto line : lines){
        linemsg.id.push_back(line->id);
        linemsg.start.push_back(line->start - width/2);
        linemsg.end.push_back(line->end - width/2);
        linemsg.center.push_back(line->center - width/2);
        linemsg.width.push_back(line->width);
    }
    line_pub.publish(linemsg);*/

    pub.publish(cv_ptr->toImageMsg());
}


void findLine(cv::Mat sampled, bool wrap){
    lines.clear();
    
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(sampled, mean, stddev );
    //cout << stddev << endl;

    // do not search for lines if the standard dev is small (same intensity across the image)
    if(stddev.val[0] > 15){
        
        if (black){
            cv::adaptiveThreshold(sampled,sampled,255,CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV ,101, stddev[0]);
        }else{
            cv::adaptiveThreshold(sampled,sampled,255,CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY ,101, -stddev[0]);
        }
        cv::erode(sampled, sampled, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,3)));
        cv::dilate(sampled, sampled, cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,3)));
        cv::reduce(sampled, sampled, 0, CV_REDUCE_AVG);
        cv::threshold(sampled,sampled,128,255,CV_THRESH_BINARY);

        int segmentStart = 0;

        uint8_t old = 0;// sampled.at<uint8_t>(0,thetaSteps-1);
        uint8_t value = 0;
        for(int i = 0 ; i< sampled.cols ; i++){

            uint8_t value = sampled.at<uint8_t>(0,i);
            if (value != old){
                //new segment
                if(value == 0){                    
                    Line *line = new Line();
                    line->start = segmentStart;
                    line->end = i-1;
                    lines.push_back(line);
                }else{
                    segmentStart = i;
                } 
            }
            old = value;
        }
        // Fencepost
        if (old == 255){
            if (wrap && lines.size() > 0 && lines[0]->start == 0){
                lines[0]->start = segmentStart;
                lines[0]->end   = lines[0]->end + sampled.cols;
                //cout << "Wrapping Line" << endl;
            }else{
                Line *line =new Line();
                line->start = segmentStart;
                line->end = sampled.cols;
                lines.push_back(line);

            }
        }
        for(Line* line : lines){
            line->width = line->end-line->start;
            line->center = (line->start+line->end)/2;
            line->id = 0;

            //Match line
            for(auto oldLine : oldLines){
                //cout << "check " << (line->center - oldLine->center) << ", " <<  (line->width - oldLine->width) << endl;
                if((abs(line->center - oldLine->center) < 40) && (abs(line->width - oldLine->width) < 10) ){
                    line->id = oldLine->id;
                }
            }
            if(line->id == 0){
                line->id = maxLineId+1;
                //cout << "assign id" << line->id << endl;
                maxLineId++;
            }
            //cout << "id = " << line->id << endl;
        }
    }
}

void sampleCircle(cv::Mat image, cv::Point center){  
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
        }
    }
    
    cv::cvtColor(sampled, sampled, CV_BGR2GRAY);
    cv::circle(image, center, radius+rSteps*rStepSize, CV_RGB(255,255,0));    
}

void sampleLine(cv::Mat image){
    cv::Rect lineROI(0, height,image.cols,1);
    sampled = image(lineROI);
    cv::cvtColor(sampled, sampled, CV_BGR2GRAY);
}

void followLine(cv::Mat image, cv::Point center, cv::Point lastPoint, int depthLeft){
    /*sampleCircle(image, center);
    auto lines = findLine(sampled, true);
    ////cout << "size: " << lines.size() << endl;

    for(cv::Point line : lines){
        float segmentCenterAngle = ((line.x+line.y)/2)*(2*M_PI)/thetaSteps; // Scaling for thetaSteps?
        //int segmentSize = segmentEnd-segmentStart;
        int r = radius + (rSteps*rStepSize/2);
        int x = center.x + r*cos(segmentCenterAngle);
        int y = center.y + r*sin(segmentCenterAngle);
        cv::circle( image, cv::Point(x,y), 8, CV_RGB( 255, 0, 0 ), 1);

        float dist = (x-lastPoint.x)*(x-lastPoint.x) + (y-lastPoint.y)*(y-lastPoint.y);
        ////cout << "y: " << y << endl;
        ////cout << "dist: " << dist << endl;
        if(depthLeft > 0 && y < height && dist > r*r*0.5){
            followLine(image, cv::Point(x,y), center, depthLeft-1);
        }
    }*/
}

void sample(cv::Mat image){
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    
    sampleLine(image);
    findLine(sampled, false);

    for(auto line : lines){
        cv::line( image, cv::Point(line->start, height), cv::Point(line->end, height), CV_RGB( 255, 0, 0 ), 1);
        std::string text = std::to_string(line->id);
        cv::putText(image, text, cv::Point(line->center, height), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255,0,255));
    }

    // Select line a line if there is one
    float selectLine = 0;
    float selectLineVal = 10000;
    if(lines.size() > 0){
        int cnt = 0;
        for(cnt = 0; cnt < lines.size(); cnt++){
            //Criteria for selection close to and same size as previous line
            int val = abs(lines[cnt]->center - linesensor_angle)*pixel2Angle
                    + abs(lines[cnt]->width - pixel2Angle);  
            if(val < selectLineVal){
                selectLineVal = val;
                selectLine = cnt;
            }  
            cnt++;
        }
        linesensor_angle = lines[selectLine]->center;

        //ROS_ERROR("%d", lines[selectLine]->width);

        //sampleCircle(image, cv::Point(lines[selectLine]->center, height));
        //followLine()


        if(speed != 0){
            roberto_msgs::MotorState motor_msg;
            motor_msg.speed = 0.0;
            motor_msg.heading_angle = -(linesensor_angle - width/2)*0.13;
            motor_msg.mode = motor_msg.DRIVE_MODE_PIVOT;
            motor_pub.publish(motor_msg);
        }
    }











    for(auto line : oldLines){
        delete line;
    }
    oldLines = lines;





    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    ////cout << "timing: " << duration << " uS" << endl;
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
    line_pub = nh.advertise<roberto_msgs::Line>("line",1);
    motor_pub = nh.advertise<roberto_msgs::MotorState>("cmd_vel",1);
    //pubS = it.advertise("usb_cam/sampled", 1);
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    //first_scanpoint = Point(FRAME_WIDTH/2, scan_height_reg);
    //debug_mode=1;
    ros::spin();
    return 0;
}


