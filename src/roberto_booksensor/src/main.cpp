
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

vector<KeyPoint> kpbooks[4];
vector<KeyPoint> kp;

Mat descbooks[4];
Mat desc;

// Default parameters of ORB
int nfeatures=1000;
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

            if((int)matched1.size() > 170){
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


    books.push_back(imread("/home/mathias/book1.jpg"));
    books.push_back(imread("/home/mathias/book2.jpg"));
    books.push_back(imread("/home/mathias/book3.jpg"));
    books.push_back(imread("/home/mathias/book4.jpg"));

    detector->detectAndCompute(books[0], noArray(), kpbooks[0], descbooks[0]);
    detector->detectAndCompute(books[1], noArray(), kpbooks[1], descbooks[1]);
    detector->detectAndCompute(books[2], noArray(), kpbooks[2], descbooks[2]);
    detector->detectAndCompute(books[3], noArray(), kpbooks[3], descbooks[3]);

	image_transport::ImageTransport it(nh);
	pub = it.advertise("usb_cam/image_book_matching", 1);
	image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
	ros::spin();
	return 0;
}