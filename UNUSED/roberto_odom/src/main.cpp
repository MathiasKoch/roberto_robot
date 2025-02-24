
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32MultiArray.h"


ros::Publisher pub_;
ros::Publisher odom_pub;
ros::Subscriber sub_;

ros::Time current_time, last_time;

double x = 0.0;
double y = 0.0;
double th = 0.0;



void odomCallback(const std_msgs::Float32MultiArray::ConstPtr& od){
    static tf::TransformBroadcaster odom_broadcaster;

    current_time = ros::Time::now();

    float vx = od->data[0];
    float vy = od->data[1];
    float vth = od->data[2];

    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);


    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";  

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    pub_.publish(odom);


    nav_msgs::Odometry odom2;
    odom2.header.stamp = current_time;
    odom2.header.frame_id = "odom";

    //set the position
    odom2.pose.pose.position.x = x;
    odom2.pose.pose.position.y = y;
    odom2.pose.pose.position.z = 0.0;
    odom2.pose.pose.orientation = odom_quat;

    //set the velocity
    odom2.child_frame_id = "base_link";
    odom2.twist.twist.linear.x = vx;
    odom2.twist.twist.linear.y = vy;
    odom2.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom2);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_repacker");
	ros::NodeHandle nh;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


    pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10, true);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom_integral", 10, true);
    sub_ = nh.subscribe<std_msgs::Float32MultiArray>("odom_vel", 10, odomCallback);
	ros::spin();
	return 0;
}