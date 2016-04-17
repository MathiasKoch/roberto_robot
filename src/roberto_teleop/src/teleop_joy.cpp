/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <roberto_msgs/MotorState.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class RobertoTeleop
{
public:
  RobertoTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  bool init2, init5;

  roberto_msgs::MotorState last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_cmd_published_;
  ros::Timer timer_;

};

RobertoTeleop::RobertoTeleop():
  ph_("~"),
  linear_(5),
  angular_(0),
  deadman_axis_(4),
  l_scale_(0.3),
  a_scale_(0.9)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  deadman_pressed_ = false;
  zero_cmd_published_ = false;
  init2 = false;
  init5 = false;

  vel_pub_ = ph_.advertise<roberto_msgs::MotorState>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RobertoTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&RobertoTeleop::publish, this));
}

void RobertoTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  /*geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  last_published_ = vel;*/

  if(!init5){
    if(joy->axes[5] != 0){
      init5 = true;
    }
  }

  if(!init2){
    if(joy->axes[2] != 0){
      init2 = true;
    }
  }

  double scale = l_scale_;
  roberto_msgs::MotorState vel;
  if(joy->buttons[0]){
    scale = scale*0.35;
    vel.heading_angle = 0;
    vel.mode = vel.DRIVE_MODE_SPIN;
  }else if(joy->buttons[1]){
    scale = scale*0.5;
    vel.heading_angle = 0;
    vel.mode = vel.DRIVE_MODE_SIDEWAYS;
  }else{
    vel.heading_angle = a_scale_*joy->axes[angular_];
    vel.mode = vel.DRIVE_MODE_PIVOT;
  }
  vel.speed = (scale*(joy->axes[2]-1)) - (scale*(joy->axes[5]-1));
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
}

void RobertoTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_cmd_published_=false;
  }
  else if(!deadman_pressed_ && !zero_cmd_published_)
  {
    roberto_msgs::MotorState vel;
    vel.speed = 0;
    vel.heading_angle = 0;
    vel_pub_.publish(vel);
    zero_cmd_published_=true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roberto_teleop");
  RobertoTeleop Roberto_teleop;

  ros::spin();
}