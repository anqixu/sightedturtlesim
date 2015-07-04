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


#ifndef TURTLE_HPP_
#define TURTLE_HPP_

#include <ros/ros.h>
#include <mutex>

#include <turtlesim/Pose.h>
#include <sightedturtlesim/PoseXYZ.h>
#include <sightedturtlesim/VelocityXYZ.h>
#include <sightedturtlesim/TeleportAbsoluteXYZ.h>


class Turtle {
public:
  Turtle(const ros::NodeHandle& nh, const sightedturtlesim::PoseXYZ& initPose, double s = 1.0);
  virtual ~Turtle();
  virtual void update(double dt, double canvasWidth, double canvasHeight);

  void setPose(const sightedturtlesim::PoseXYZ& newPose) {
    poseMutex.lock();
    pos_ = newPose;
    poseMutex.unlock();
  };
  void setPose(double x, double y, double z, double theta, bool resetVel = false) {
    poseMutex.lock();
    pos_.x = x;
    pos_.y = y;
    pos_.z = z;
    pos_.theta = theta;
    if (resetVel) {
      pos_.linear_velocity = 0;
      pos_.linear_velocity_z = 0;
      pos_.angular_velocity = 0;
    }
    poseMutex.unlock();
  };

  double x() { return pos_.x; };
  double y() { return pos_.y; };
  double z() { return pos_.z; };

  double angleRad() { return pos_.theta; };
  double angleDeg() { return pos_.theta/M_PI*180.0; };

  virtual int type() { return 0; };

  double getScale() { return scale; };

protected:
  void velocityXYZCallback(const sightedturtlesim::VelocityXYZ::ConstPtr& msg);
  bool teleportAbsoluteXYZCallback(
      sightedturtlesim::TeleportAbsoluteXYZ::Request&,
      sightedturtlesim::TeleportAbsoluteXYZ::Response&);

  ros::NodeHandle nh_;

  sightedturtlesim::PoseXYZ pos_;
  double scale;

  bool alive;

  ros::Subscriber velocity_xyz_sub_;
  ros::Publisher pose_xyz_pub_;
  ros::ServiceServer teleport_absolute_xyz_srv_;

  ros::WallTime last_command_time_;

  std::mutex poseMutex;

  constexpr static double COMMAND_TIMEOUT_SECS = 1.0;
};


#endif /* TURTLE_HPP_ */
