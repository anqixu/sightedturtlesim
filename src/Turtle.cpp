/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * Copyright (c) 2012-2015, Anqi Xu
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

#include "sightedturtlesim/Turtle.hpp"


Turtle::Turtle(const ros::NodeHandle& nh, const sightedturtlesim::PoseXYZ& initPose,
    const sightedturtlesim::TurtleParams& params) :
    nh_(nh), pos_(initPose), params_(params), alive(true) {
  velocity_xyz_sub_ = nh_.subscribe("command_velocity_xyz", 1, &Turtle::velocityXYZCallback, this);
  pose_xyz_pub_ = nh_.advertise<sightedturtlesim::PoseXYZ>("pose_xyz", 1);
  params_pub_ = nh_.advertise<sightedturtlesim::TurtleParams>("params", 1, true); // latch == true
  teleport_absolute_xyz_srv_ = nh_.advertiseService("teleport_absolute_xyz",
      &Turtle::teleportAbsoluteXYZCallback, this);
  set_params_srv_ = nh_.advertiseService("set_params",
      &Turtle::setParamsCallback, this);

  params_pub_.publish(params_);
};


Turtle::~Turtle() {
  alive = false;
};


void Turtle::velocityXYZCallback(
    const sightedturtlesim::VelocityXYZ::ConstPtr& msg) {
  poseMutex.lock();
  last_command_time_ = ros::WallTime::now();
  pos_.linear_velocity = msg->linear_xy;
  pos_.angular_velocity = msg->angular_xy;
  pos_.linear_velocity_z = msg->linear_z;
  poseMutex.unlock();
};


bool Turtle::teleportAbsoluteXYZCallback(
    sightedturtlesim::TeleportAbsoluteXYZ::Request& req,
    sightedturtlesim::TeleportAbsoluteXYZ::Response&) {
  setPose(req.x, req.y, req.z, req.theta, true); // resetSpeed == true
  return true;
};


bool Turtle::setParamsCallback(
    sightedturtlesim::SetTurtleParams::Request& req,
    sightedturtlesim::SetTurtleParams::Response&) {
  poseMutex.lock();
  params_ = req.new_params;
  poseMutex.unlock();
  params_pub_.publish(params_);
  return true;
};


void Turtle::update(double dt, double canvasWidth, double canvasHeight) {
  poseMutex.lock();

  if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(COMMAND_TIMEOUT_SECS)) {
    pos_.linear_velocity = 0.0;
    pos_.angular_velocity = 0.0;
    pos_.linear_velocity_z = 0.0;
  }

  pos_.theta = fmod(pos_.theta + pos_.angular_velocity * dt, 2*M_PI);
  pos_.x += params_.spatial_scale * sin(pos_.theta + M_PI/2.0) * pos_.linear_velocity * dt;
  pos_.y += params_.spatial_scale * cos(pos_.theta + M_PI/2.0) * pos_.linear_velocity * dt;
  double z_new = pos_.z + params_.spatial_scale * pos_.linear_velocity_z * dt;
  if (z_new > 0) { pos_.z = z_new; } // Only update z if new value is above zero

  // Cycle-clamp position
  pos_.x = pos_.x - floor(pos_.x/canvasWidth)*canvasWidth;
  if (pos_.x < 0) { pos_.x += canvasWidth; }
  pos_.y = pos_.y - floor(pos_.y/canvasHeight)*canvasHeight;
  if (pos_.y < 0) { pos_.y += canvasHeight; }

  sightedturtlesim::PoseXYZ p = pos_; p.header.stamp = ros::Time::now();

  //ROS_DEBUG_STREAM("[" << nh_.getNamespace() << "]: pos_x: " << pos_.x <<
  //    " pos_y: " << pos_.y << " theta: " << pos_.theta << " pos_z: " << pos_.z);

  poseMutex.unlock();

  pose_xyz_pub_.publish(p);
};
