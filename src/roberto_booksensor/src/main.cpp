
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include "std_msgs/String.h"
#include <dirent.h>

using namespace cv;
using namespace std;

const double ransac_thresh = 2.5f;
const double nn_match_ratio = 0.8f;

vector<Point2f> Points(vector<KeyPoint> keypoints)
{
    vector<Point2f> res;
    for(unsigned i = 0; i < keypoints.size(); i++) {
        res.push_back(keypoints[i].pt);
    }
    return res;
}

Ptr<ORB> detector;
Ptr<DescriptorMatcher> matcher;

vector<Mat> books;

vector<vector<KeyPoint>> kpbooks;
vector<KeyPoint> kp;

vector<Mat> descbooks;
Mat desc;

// Default parameters of ORB
int nfeatures=2500;
float scaleFactor=1.2f;
int nlevels=8;
int edgeThreshold=15; // Changed default (31);
int firstLevel=0;
int WTA_K=2;
int scoreType=ORB::HARRIS_SCORE;
int patchSize=31;
int fastThreshold=20;


bool running = true;

image_transport::Publisher pub;

int frameCnt = 0;


void imageCallback(const sensor_msgs::ImageConstPtr& msg){

    if(running){
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat img = cv_ptr->image;


        detector->detectAndCompute(img, noArray(), kp, desc);

        vector< vector<DMatch> > matches;
        vector<KeyPoint> matched1, matched2;

        int bookCnt = 0;
        int matched = -1;
        for(bookCnt = 0; bookCnt < books.size(); bookCnt++){


            matches.clear();
            matched1.clear();
            matched2.clear();
            matcher->knnMatch(descbooks[bookCnt], desc, matches, 2);


            for(unsigned i = 0; i < matches.size(); i++) {
                if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
                    vector<KeyPoint> kpbook = kpbooks[bookCnt];
                    matched1.push_back(kpbook[matches[i][0].queryIdx]);
                    matched2.push_back(kp[matches[i][0].trainIdx]);
                }
            }
//            ROS_INFO("Match found! Image matches book %d", (int)matched1.size());

            if((int)matched1.size() > 50){
                matched = bookCnt;
                break;
            }
        }


        if(matched != -1){
            ROS_INFO("Match found! Image matches book %d", matched);
            Mat inlier_mask, homography;
            vector<KeyPoint> inliers1, inliers2;
            vector<DMatch> inlier_matches;
            if(matched1.size() >= 4) {
                homography = findHomography(Points(matched1), Points(matched2),
                                            RANSAC, ransac_thresh, inlier_mask);
            }

            Mat res;
            if(matched1.size() < 4 || homography.empty()) {
                hconcat(books[matched], img, res);
            }else{

                for(unsigned i = 0; i < matched1.size(); i++) {
                    if(inlier_mask.at<uchar>(i)) {
                        int new_i = static_cast<int>(inliers1.size());
                        inliers1.push_back(matched1[i]);
                        inliers2.push_back(matched2[i]);
                        inlier_matches.push_back(DMatch(new_i, new_i, 0));
                    }
                }

                //vector<Point2f> new_bb;
                //perspectiveTransform(object_bb, new_bb, homography);
                //Mat frame_with_bb = img2.clone();
                

                drawMatches(books[matched], inliers1, img, inliers2,
                            inlier_matches, res,
                            Scalar(255, 0, 0), Scalar(255, 0, 0));
            }


            cv_ptr->image = res;
            /*std::cout << "Found " << kp1.size() << " Keypoints on img1" << std::endl;
            std::cout << "Found " << kp2.size() << " Keypoints on img2" << std::endl;
            std::cout << "Found " << matched1.size() << " Matches" << std::endl;*/
            pub.publish(cv_ptr->toImageMsg());
        }else{
            ROS_INFO("No Matches found!");
        }
    }
    frameCnt++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "book_node");
    ros::NodeHandle nh;

    if ( CV_MAJOR_VERSION == 2){
        ROS_ERROR("CV_MAJOR_VERSION == 2");
        return 0;
    }


    detector = ORB::create(
    nfeatures,
    scaleFactor,
    nlevels,
    edgeThreshold,
    firstLevel,
    WTA_K,
    scoreType,
    patchSize,
    fastThreshold );

    matcher = DescriptorMatcher::create("BruteForce-Hamming");


    DIR *dpdf;
    struct dirent *epdf;

    char path[] = "/home/odroid/images/";

    dpdf = opendir(path);
    if (dpdf != NULL){
        while (epdf = readdir(dpdf)){
            // std::cout << epdf->d_name << std::endl;
            if(strstr(epdf->d_name, ".jpg") != NULL || strstr(epdf->d_name, ".png") != NULL){
                char str[80];
                sprintf(str, "%s%s", path, epdf->d_name);
                ROS_INFO("Found book: %s", str);
                books.push_back(imread(str));
            }
        }
    }
    closedir(dpdf);    

    int i = 0;
    for(i = 0; i < books.size(); i++){
        vector<KeyPoint> kp_;
        Mat desc_;
        detector->detectAndCompute(books[i], noArray(), kp_, desc_);
        kpbooks.push_back(kp_);
        descbooks.push_back(desc_);

    }
   

    image_transport::ImageTransport it(nh);
    pub = it.advertise("usb_cam/image_book_matching", 1);
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    ros::spin();

    return 0;
}
