/*
 * Line follower algorithm
 *
 * Version: 1.0
 * Date: 2014-10-24
 *
 *
 * Copyright 2014 Arne Baeyens
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The object tracking part is mainly made by Kyle Hounslow. Check his excellent tutorial about object tracking: youtube.com/watch?v=bSeFrPrqZ2A
 * or the complete source code: dl.dropboxusercontent.com/u/28096936/tuts/objectTrackingTut.cpp.
 * Without the work of Pierre Raufast (thinkrpi.wordpress.com) and Emil Valvov (robidouille.wordpress.com) this project would have been much more difficult.
 * A big thanks to them.
 *
 * 2arne.baeyens@gmail.com
 *
 *
 * For compiling, please refer to the README on github or the forum thread.
*/


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

/* FUNCTIONS */
// For scanning and searching the line.
void scancircle( Mat& I, Point Mp, int radius, int look_angle, int width );
void scanline( Mat& I, Point Mp, int line_radius );
Point findLine( char scan_mode );

bool test_inimage( Mat& I, short x_cor, short y_cor );

//void loadTime( float *t_load_pointer );
int lineangle( void );	// Returns the angle of the line (0 - 360 degrees), using global variables.

int first_angle = 0;


// Data about scans.
uchar scandata[640];		// Array to contain scan values.
short scan_w;				// Number of (active) values in scandata array.
Point scanpoint;			// Last scanpoint.
Point line_point;			// Last x coordinates of found line.
vector<Point> line_points(7);	// vector om de laatste gevonden punten bij te houden.
Point first_scanpoint;		// Eerst gevonden punt.


short scan_radius;
short sc_strt, sc_end;

short new_scan_radius1 = 140;

// Variables for trackbars.
int scan_height_reg = 220;
int scan_radius1_reg = 55;
int scan_radius2_reg = 40;
//int look_angle_reg = 180;
int look_width_reg = 160;

// Some variables for time measurement.
/*float time_e;
double time_running = (double)getTickCount() / getTickFrequency();
double time_old = 0;*/

// Matrices.
Mat image;			// image from camera.
Mat gray_image;		// gray camera image.
Mat scan_image( 256, 640, CV_8UC3, Scalar( 255, 255, 255));		// to graph greyscale values on.
//Mat D_scan_image( 256, 640, CV_8UC3, Scalar( 255, 255, 255));		// to graph the differences in greyscale values.


void imageCallback(const sensor_msgs::ImageConstPtr& msg){

	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");


	image = cv_ptr->image;

	cvtColor( image, gray_image, CV_RGB2GRAY );	// Convert it to a gray image.

	/* ZERO SCAN */
	scanline( gray_image, Point(first_scanpoint.x, scan_height_reg), new_scan_radius1 );
	first_scanpoint = findLine(0);			// Keep point in memory for next iteration.
	new_scan_radius1 = scan_radius1_reg;


	/*if (first_angle < -45)
		first_angle = -45;
	else if (first_angle > 45)
		first_angle = 45;*/


	int angle = first_angle;					// Keep angle in memory for next iteration.
	look_width_reg = 160;
	int curvature = 0;
	for(int a = 0; a < 4; a++){
		scancircle( gray_image, line_points[0], scan_radius2_reg, angle, look_width_reg);
		findLine(1);
		angle = lineangle();
		look_width_reg = 180;
		curvature += angle * line_point.y;
	}
	

	ROS_INFO("FIRST ANGLE: %d\t%d", angle, first_scanpoint.x - image.cols / 2);


	pub.publish(cv_ptr->toImageMsg());
}

void scancircle( Mat& I, Point Mp, int radius, int look_angle, int width ) {
	int i, j;
	int n = 0;
	Point dot_pos, dot2_pos;

	scanpoint = Mp;

	scan_radius = radius;
	sc_strt = Mp.x - radius;
	sc_end = Mp.x + radius;

	/*Point end_point_left = Point( scanpoint.x - sin(PI * (look_angle + 180 - width / 2) / 180) * scan_radius, scanpoint.y - cos(PI * (look_angle + 180 - width / 2 + 180) / 180) * scan_radius );
	Point end_point_right = Point( scanpoint.x - sin(PI * (look_angle + 180 + width / 2) / 180) * scan_radius, scanpoint.y + cos(PI * (look_angle + 180 + width / 2) / 180) * scan_radius );

	if (show_images == 1) {
		circle( image, end_point_left, 5, Scalar( 255, 0, 0 ), -1, 8, 0 );
		circle( image, end_point_right, 5, Scalar( 0, 255, 0 ), -1, 8, 0 );
	}*/

	for ( i = 0; i < radius * 2; i++ ) {
		dot_pos.x = round(Mp.x - radius * sqrt(1 - pow(((float)i / radius) - 1, 2)));
		dot2_pos.x = round(Mp.x - radius * sqrt(1 - pow((((float)i + 1) / radius) - 1, 2)));

		dot_pos.y = Mp.y - i + radius;
		if ( test_inimage( I, dot_pos.x, dot_pos.y))
			scandata[n] = I.at<uchar>(dot_pos);
		else
			scandata[n] = scandata[n - 1];
		n++;
		/*I.at<uchar>(dot_pos) = 0;
		waitKey(10);
		imshow( "experiment", I );*/

		for ( j = 1; j < dot2_pos.x - dot_pos.x; j++) {
			if (test_inimage( I, dot_pos.x, dot_pos.y + j))
				scandata[n] = I.at<uchar>(dot_pos.y, dot_pos.x + j);
			else
				scandata[n] = scandata[n - 1];
			n++;
			/*I.at<uchar>(dot_pos.y, dot_pos.x+j)=120 ;
			waitKey(10);
			imshow( "experiment", I );*/
		}
		for ( j = 1; j < dot_pos.x - dot2_pos.x; j++) {
			if ( test_inimage( I, dot_pos.x, dot_pos.y - j))
				scandata[n] = I.at<uchar>(dot_pos.y, dot_pos.x - j);
			else
				scandata[n] = scandata[n - 1];
			n++;
			/*I.at<uchar>(dot_pos.y, dot_pos.x-j)=120 ;
			waitKey(10);
			imshow( "experiment", I );*/
		}
	}

	for ( i = radius * 2 - 1; i >= 0; i-- ) {
		dot_pos.x = round(Mp.x + radius * sqrt(1 - pow(((float)i / radius) - 1, 2)));
		dot2_pos.x = round(Mp.x + radius * sqrt(1 - pow((((float)i + 1) / radius) - 1, 2)));

		dot_pos.y = Mp.y - i + radius;

		for ( j = dot2_pos.x - dot_pos.x - 1; j > 0; j--) {
			if (test_inimage( I, dot_pos.x, dot_pos.y + j))
				scandata[n] = I.at<uchar>(dot_pos.y, dot_pos.x + j);
			else
				scandata[n] = scandata[n - 1];
			n++;
			/*I.at<uchar>(dot_pos.y, dot_pos.x+j)=120 ;
			waitKey(100);
			imshow( "experiment", I );*/
		}
		for ( j = dot_pos.x - dot2_pos.x - 1; j > 0; j--) {
			if (test_inimage( I, dot_pos.x, dot_pos.y - j))
				scandata[n] = I.at<uchar>(dot_pos.y, dot_pos.x - j);
			else
				scandata[n] = scandata[n - 1];
			n++;
			/*I.at<uchar>(dot_pos.y, dot_pos.x-j)=120 ;
			waitKey(100);
			imshow( "experiment", I );*/
		}

		if (test_inimage( I, dot_pos.x, dot_pos.y))
			scandata[n] = I.at<uchar>(dot_pos);
		else
			scandata[n] = scandata[n - 1];

		n++;
		/*I.at<uchar>(dot_pos.y, dot_pos.x) = 0;
		waitKey(100);
		imshow( "experiment", I );*/
	}

	scan_w = n - 1;
	//cout << scan_w << endl;

	int look_strt = round((float)scan_w * (look_angle + 180 - width / 2) / 360);
	int look_end = round((float)scan_w * (look_angle + 180 + width / 2) / 360);

	for (i = 0; i < look_strt; i++)
		scandata[i] = scandata[look_strt + 1];
	for (i = look_end; i < scan_w; i++)
		scandata[i] = scandata[look_end - 1];

	/*if (debug_mode == 1) {
		for (i = 0; i < scan_w; i++)
			line( scan_image, Point(i, scan_image.rows), Point(i, scandata[i]), Scalar( 0, 0, 0 ), 1, 8 );
		//circle( I, Mp, 5, Scalar( 255 ), -1, 8, 0  );
		//imshow( "experiment", I );
	}*/

	//I = Mat( 240, 320, CV_8U, Scalar( 255 ));
}

void scanline( Mat& I, Point Mp, int line_radius ) {
	// accept only char type matrices
	CV_Assert(I.depth() != sizeof(uchar));

	int channels = I.channels();

	int nRows = I.rows;
	int nCols = I.cols * channels;

	int i;

	scanpoint = Mp;

	scan_radius = line_radius;
	scan_w = line_radius * 2;
	sc_strt = Mp.x - line_radius;
	sc_end = Mp.x + line_radius;

	uchar* p;
	p = I.ptr<uchar>(Mp.y);

	for ( i = 0; i < scan_w; i++) {
		//cout << i << " " ;
		if (i + sc_strt < 0)
			scandata[i] = p[0];
		else if (i + sc_strt >= I.cols)
			scandata[i] = p[I.cols - 2];
		else
			scandata[i] = p[i + sc_strt];

		//if (debug_mode == 1)
		//	line( scan_image, Point(i, scan_image.rows), Point(i, scandata[i]), Scalar( 0, 0, 0 ), 1, 8 );
	}

	if ( show_images == 1 ) {
		line( image, Point(0, Mp.y), Point(image.cols, Mp.y), Scalar( 0, 0, 255 ), 1, 8 );

		if (Mp.x - line_radius >= 0)
			circle( image, Point(Mp.x - line_radius, Mp.y), 5, Scalar( 255, 0, 0 ), -1, 8, 0 );

		if (Mp.x + line_radius < I.cols)
			circle( image, Point(Mp.x + line_radius, Mp.y), 5, Scalar( 255, 0, 0 ), -1, 8, 0 );
	}
}

Point findLine( char scan_mode ) {
	Point left_side;
	Point right_side;

	int i, j;

	int der_scan[639];
	der_scan[0] = 0;
	der_scan[639] = 0;

	// Find derivative of 'scandata' buffer
	for ( i = 1; i < scan_w - 1; i++) {
		der_scan[i] = int(scandata[i - 1]) - int(scandata[i + 1]);
		//if (debug_mode == 1) 
		//	line( D_scan_image, Point(i, D_scan_image.rows / 2), Point(i, D_scan_image.rows / 2 - der_scan[i]), Scalar( 255, 0, 0 ), 1, 8 );
	}


	// Find Max and Min of derivative image
	for ( i = 1; i < scan_w - 1; i++ ) {
		if ( der_scan[i] > left_side.y ) {
			left_side.y = der_scan[i];
			left_side.x = i;
		}
		if ( der_scan[i] < right_side.y ) {
			right_side.y = der_scan[i];
			right_side.x = i;
		}
	}


	float line_pos = (right_side.x + left_side.x) / 2;

	// If scanning a line
	if ( scan_mode == 0 ){
		line_point = Point( line_pos + sc_strt, scanpoint.y );
	// Else if scanning a circle
	}else if ( scan_mode == 1 ) {
		line_pos = line_pos / scan_w;
		line_point = Point( scanpoint.x + cos(PI * ((float)line_pos * 2 + 0.5)) * scan_radius, scanpoint.y + sin(PI * ((float)line_pos * 2 + 0.5)) * scan_radius );
		if (show_images == 1)
			circle( image, scanpoint, scan_radius, Scalar( 255, 0, 0 ), 1, 8, 0 );
	}


	// Saturate line center point within image
	if (line_point.x >= image.cols)
		line_point.x = image.cols - 1;
	else if (line_point.x < 0)
		line_point.x = 0;

	if (show_images == 1)
		circle( image, line_point, 5, Scalar( 0, 0, 255 ), -1, 8, 0 );

	/*circle( D_scan_image, Point( left_side.x+sc_strt, D_scan_image.rows/2-left_side.y ), 5, Scalar( 255, 0, 255 ), 2, 8, 0 );
	circle( D_scan_image, Point( right_side.x+sc_strt, D_scan_image.rows/2-right_side.y ), 5, Scalar( 0, 255, 255 ), 2, 8, 0 );

	if (debug_mode == 1) {
		imshow( "scan image", scan_image );
		imshow( "derivative scan", D_scan_image );
		scan_image = Mat( 256, 640, CV_8UC3, Scalar( 255, 255, 255));
		D_scan_image = Mat( 256, 640, CV_8UC3, Scalar( 255, 255, 255));
	}*/

	for (i = 2; i >= 0; i--)
		line_points[i + 1] = line_points[i];	// Waarden doorschuiven.
	line_points[0] = line_point;				// Laatst gevonden punt opslaan in de lijst.

	return line_point;
}

bool test_inimage( Mat& I, short x_cor, short y_cor ) {
	if ( x_cor >= 0 && y_cor >= 0 && x_cor < I.cols && y_cor < I.rows )
		return 1;
	return 0;
}

int lineangle( void ) {
	return round(atan2( (line_points[0].x - scanpoint.x), -(line_points[0].y - scanpoint.y) ) * 180 / PI);
}

/*void loadTime( float *t_load_pointer ) {
	time_running = (float)getTickCount() / getTickFrequency();
	*t_load_pointer = time_running - time_old;
	time_old = time_running;
}*/


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
	first_scanpoint = Point(FRAME_WIDTH/2, scan_height_reg);
	debug_mode=1;
	ros::spin();
	return 0;
}