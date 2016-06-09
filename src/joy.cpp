/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 * @file   joy.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @date   2016
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

//==============================================================================
class PlatformTeleop
{
public:
  PlatformTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_;
	int deadman_axis_slow_, deadman_axis_fast_, deadman_axis_precise_;
	int precise_left_, precise_right_, precise_up_, precise_down_;
  double l_scale_slow_, a_scale_slow_;
  double l_scale_fast_, a_scale_fast_;
  double l_scale_precise_, a_scale_precise_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_; // Determines if one of deadmans pressed
  bool zero_twist_published_;
  ros::Timer timer_;

};

//==============================================================================
PlatformTeleop::PlatformTeleop():
  ph_("~"),
  linear_(1),
  angular_(0),
  deadman_axis_slow_(10),
  deadman_axis_fast_(8),
  deadman_axis_precise_(11),
  l_scale_slow_(0.2),
  a_scale_slow_(0.4),
  l_scale_fast_(0.4),
  a_scale_fast_(1.0),
  l_scale_precise_(0.1),
  a_scale_precise_(0.2)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman_slow", deadman_axis_slow_, deadman_axis_slow_);
  ph_.param("axis_deadman_fast", deadman_axis_fast_, deadman_axis_fast_);
  ph_.param("axis_deadman_precise", deadman_axis_precise_, deadman_axis_precise_);
  ph_.param("scale_angular_slow", a_scale_slow_, a_scale_slow_);
  ph_.param("scale_linear_slow", l_scale_slow_, l_scale_slow_);
  ph_.param("scale_angular_fast", a_scale_fast_, a_scale_fast_);
  ph_.param("scale_linear_fast", l_scale_fast_, l_scale_fast_);
  ph_.param("scale_angular_precise", a_scale_precise_, a_scale_precise_);
  ph_.param("scale_linear_precise", l_scale_precise_, l_scale_precise_);
  ph_.param("precise_down", precise_down_, precise_down_);
  ph_.param("precise_up", precise_up_, precise_up_);
  ph_.param("precise_right", precise_right_, precise_right_);
  ph_.param("precise_left", precise_left_, precise_left_);

  deadman_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &PlatformTeleop::joyCallback, this);
  timer_ = nh_.createTimer(ros::Duration(0.01), boost::bind(&PlatformTeleop::publish, this));
}

//==============================================================================
void PlatformTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  deadman_pressed_ = (joy->buttons[deadman_axis_slow_] || joy->buttons[deadman_axis_fast_] || joy->buttons[deadman_axis_precise_]);
  
	if (joy->buttons[deadman_axis_slow_])
	{
		vel.angular.z = a_scale_slow_*joy->axes[angular_];
		vel.linear.x = l_scale_slow_*joy->axes[linear_];
	}
	else if (joy->buttons[deadman_axis_fast_])
	{
		vel.angular.z = a_scale_fast_*joy->axes[angular_];
		vel.linear.x = l_scale_fast_*joy->axes[linear_];
	}
	else if (joy->buttons[deadman_axis_precise_])
	{
		if (joy->buttons[precise_left_])
			vel.angular.z = -a_scale_precise_*joy->buttons[precise_left_];			
		else if (joy->buttons[precise_right_])
			vel.angular.z = a_scale_precise_*joy->buttons[precise_right_];;			

		if (joy->buttons[precise_up_])
			vel.linear.x = l_scale_precise_*joy->buttons[precise_up_];			
		else if (joy->buttons[precise_down_])
			vel.linear.x = -l_scale_precise_*joy->buttons[precise_down_];			
	}
  last_published_ = vel;
}

//==============================================================================
void PlatformTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

//==============================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "platform_teleop");
  PlatformTeleop platform_teleop;

  ros::spin();
}
