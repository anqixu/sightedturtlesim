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
  drive_xyz_sub_ = nh_.subscribe("command_drive_xyz", 1, &Turtle::driveXYZCallback, this);
  pose_xyz_pub_ = nh_.advertise<sightedturtlesim::PoseXYZ>("pose_xyz", 1);
  params_pub_ = nh_.advertise<sightedturtlesim::TurtleParams>("params", 1, true); // latch == true
  teleport_absolute_xyz_srv_ = nh_.advertiseService("teleport_absolute_xyz",
      &Turtle::teleportAbsoluteXYZCallback, this);
  set_params_srv_ = nh_.advertiseService("set_params",
      &Turtle::setParamsCallback, this);

  if (params_.spatial_scale <= 0) params_.spatial_scale = 1.0;
  params_pub_.publish(params_);
};


Turtle::~Turtle() {
  alive = false;
};


void Turtle::velocityXYZCallback(
    const sightedturtlesim::VelocityXYZ::ConstPtr& msg) {
  if (params_.ctrl_mode == sightedturtlesim::TurtleParams::VELOCITY_CTRL_MODE) {
    poseMutex.lock();
    last_command_time_ = ros::WallTime::now();
    pos_.linear_velocity = msg->linear_xy;
    pos_.angular_velocity = msg->angular_xy;
    pos_.linear_velocity_z = msg->linear_z;
    pos_.linear_acceleration = 0; // the following are for redundancy
    pos_.throttle = 0;            // (see above)
    pos_.brake = 0;               // (see above)
    poseMutex.unlock();
  } else {
    ROS_WARN_STREAM("Ignoring velocity cmds in DRIVE_CTRL_MODE");
  }
};


void Turtle::driveXYZCallback(
    const sightedturtlesim::DriveXYZ::ConstPtr& msg) {
  if (params_.ctrl_mode == sightedturtlesim::TurtleParams::DRIVE_CTRL_MODE) {
    poseMutex.lock();
    last_command_time_ = ros::WallTime::now();
    // NOTE: dismiss previous settings without update(),
    // except pos_.linear_velocity & pos_.linear_acceleration
    pos_.angular_velocity = msg->angular_xy;
    pos_.linear_velocity_z = msg->linear_z;
    pos_.throttle = std::min(std::max(msg->throttle, 0.0), 1.0);
    pos_.brake = std::min(std::max(msg->brake, 0.0), 1.0);
    poseMutex.unlock();
  } else {
    ROS_WARN_STREAM("Ignoring drive cmds in VELOCITY_CTRL_MODE");
  }
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

  if (req.new_params.ctrl_mode != params_.ctrl_mode) {
    pos_.linear_velocity = 0;
    pos_.linear_velocity_z = 0;
    pos_.linear_acceleration = 0;
    pos_.throttle = 0;
    pos_.brake = 0;
    pos_.angular_velocity = 0;
  }

  params_ = req.new_params;

  if (params_.spatial_scale <= 0) params_.spatial_scale = 1.0;
  if (params_.accel_throttle <= 0) params_.accel_throttle = 0.0;
  if (params_.decel_brake <= 0) params_.decel_brake = 0.0;
  if (params_.coeff_drag <= 0) params_.coeff_drag = 0.0;
  if (params_.coeff_resistance <= 0) params_.coeff_resistance = 0.0;
  if (params_.mass <= 0) params_.mass = 1.0;

  poseMutex.unlock();

  params_pub_.publish(params_);
  return true;
};


// Model for throttle/brake -> state/velocity/acceleration:
// http://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
void Turtle::update(double dt, double canvasWidth, double canvasHeight) {
  poseMutex.lock();

  if (params_.ctrl_mode == sightedturtlesim::TurtleParams::DRIVE_CTRL_MODE) {
    // If command timed out, reset throttle and brake, and let robot smoothly come to halt
    if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(COMMAND_TIMEOUT_SECS)) {
      pos_.throttle = 0.0;
      pos_.brake = 0.0;
    }

    // Update acceleration via forces
    pos_.linear_acceleration = (params_.accel_throttle * pos_.throttle)
        - (params_.decel_brake * pos_.brake)
        - (params_.coeff_drag * pos_.linear_velocity * pos_.linear_velocity)/params_.mass
        - (params_.coeff_resistance * pos_.linear_velocity)/params_.mass;

    // Integrate acceleration into velocity into state
    pos_.linear_velocity = std::max(pos_.linear_velocity + pos_.linear_acceleration * dt, 0.0);
    pos_.theta = fmod(pos_.theta + pos_.angular_velocity * dt, 2*M_PI);
    pos_.x += params_.spatial_scale * sin(pos_.theta + M_PI/2.0) * pos_.linear_velocity * dt;
    pos_.y += params_.spatial_scale * cos(pos_.theta + M_PI/2.0) * pos_.linear_velocity * dt;

    double z_new = pos_.z + params_.spatial_scale * pos_.linear_velocity_z * dt;
    if (z_new > 0) { pos_.z = z_new; } // Only update z if new value is above zero

  } else { // params_.ctrl_mode == sightedturtlesim::TurtleParams::VELOCITY_CTRL_MODE
    // If command timed out, reset velocity immediately
    if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(COMMAND_TIMEOUT_SECS)) {
      pos_.linear_velocity = 0.0;
      pos_.angular_velocity = 0.0;
      pos_.linear_velocity_z = 0.0;
    }

    // Integrate velocity into state
    pos_.theta = fmod(pos_.theta + pos_.angular_velocity * dt, 2*M_PI);
    pos_.x += params_.spatial_scale * sin(pos_.theta + M_PI/2.0) * pos_.linear_velocity * dt;
    pos_.y += params_.spatial_scale * cos(pos_.theta + M_PI/2.0) * pos_.linear_velocity * dt;

    double z_new = pos_.z + params_.spatial_scale * pos_.linear_velocity_z * dt;
    if (z_new > 0) { pos_.z = z_new; } // Only update z if new value is above zero
  }

  // Circular-wrap position
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
