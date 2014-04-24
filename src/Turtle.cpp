/*
 * Copyright (c) 2009, Willow Garage, Inc.
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


Turtle::Turtle(const ros::NodeHandle& nh, const Vector2& robotPos,
    double orientRad, double z, double s) :
    nh_(nh), pos_(robotPos), orient_(orientRad), z_(z),
    lin_vel_(0), ang_vel_(0), z_vel_(0), scale(s), alive(true) {
  velocity_xyz_sub_ = nh_.subscribe("command_velocity_xyz", 1, &Turtle::velocityXYZCallback, this);
  pose_xyz_pub_ = nh_.advertise<sightedturtlesim::PoseXYZ>("pose_xyz", 1);
  teleport_absolute_xyz_srv_ = nh_.advertiseService("teleport_absolute_xyz",
      &Turtle::teleportAbsoluteXYZCallback, this);
};


Turtle::~Turtle() {
  alive = false;
};


void Turtle::velocityXYZCallback(
    const sightedturtlesim::VelocityXYZConstPtr& msg) {
  poseMutex.lock();
  last_command_time_ = ros::WallTime::now();
  lin_vel_ = msg->linear_xy;
  ang_vel_ = msg->angular_xy;
  z_vel_ = msg->linear_z;
  poseMutex.unlock();
};


bool Turtle::teleportAbsoluteXYZCallback(
    sightedturtlesim::TeleportAbsoluteXYZ::Request& req,
    sightedturtlesim::TeleportAbsoluteXYZ::Response&) {
  teleport_requests_.push_back(TeleportRequest(req.x, req.y, req.theta, 0, false));
  poseMutex.lock();
  z_ = req.z;
  poseMutex.unlock();
  return true;
};


void Turtle::update(double dt, double canvasWidth, double canvasHeight) {
  poseMutex.lock();

  // Upon any teleportation request, robot's velocity is set to zero
  if (!teleport_requests_.empty()) {
    lin_vel_ = 0.0;
    ang_vel_ = 0.0;
    z_vel_ = 0.0;
  }

  // process all queued teleportation requests, in order
  std::vector<TeleportRequest>::iterator it = teleport_requests_.begin();
  std::vector<TeleportRequest>::iterator end = teleport_requests_.end();
  for (; it != end; it++) {
    const TeleportRequest& req = *it;

    if (req.relative) {
      orient_ += req.theta;
      pos_.x += scale * sin(orient_ + M_PI/2.0) * req.linear;
      pos_.y += scale * cos(orient_ + M_PI/2.0) * req.linear;
    } else {
      pos_.x = req.pos.x;
      pos_.y = req.pos.y;
      orient_ = req.theta;
    }
  }
  teleport_requests_.clear();

  if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(COMMAND_TIMEOUT_SECS)) {
    lin_vel_ = 0.0;
    ang_vel_ = 0.0;
    z_vel_ = 0.0;
  }

  orient_ = fmod(orient_ + ang_vel_ * dt, 2*M_PI);
  pos_.x += scale * sin(orient_ + M_PI/2.0) * lin_vel_ * dt;
  pos_.y += scale * cos(orient_ + M_PI/2.0) * lin_vel_ * dt;
  double z_new = z_ + scale * z_vel_ * dt;
  if (z_new > 0) { z_ = z_new; } // Only update z if new value is above zero

  // Cycle-clamp position
  pos_.x = pos_.x - floor(pos_.x/canvasWidth)*canvasWidth;
  if (pos_.x < 0) { pos_.x += canvasWidth; }
  pos_.y = pos_.y - floor(pos_.y/canvasHeight)*canvasHeight;
  if (pos_.y < 0) { pos_.y += canvasHeight; }

  sightedturtlesim::PoseXYZ p;
  p.x = pos_.x;
  p.y = pos_.y;
  p.theta = orient_;
  p.linear_velocity = lin_vel_;
  p.angular_velocity = ang_vel_;
  p.z = z_;
  p.linear_velocity_z = z_vel_;

  //ROS_DEBUG_STREAM("[" << nh_.getNamespace() << "]: pos_x: " << pos_.x <<
  //    " pos_y: " << pos_.y << " theta: " << orient_ << " pos_z: " << z_);

  poseMutex.unlock();

  pose_xyz_pub_.publish(p);
};
