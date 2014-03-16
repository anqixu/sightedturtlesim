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


#ifndef TURTLE_HPP_
#define TURTLE_HPP_

#include <ros/ros.h>
#include <mutex>

#include <turtlesim/Pose.h>
#include <sightedturtlesim/PoseXYZ.h>
#include <sightedturtlesim/VelocityXYZ.h>
#include <sightedturtlesim/TeleportAbsoluteXYZ.h>


struct Vector2 {
  Vector2() : x(0.0), y(0.0) {};
  Vector2(double _x, double _y) : x(_x), y(_y) {};
  bool operator==(const Vector2& rhs) { return x == rhs.x && y == rhs.y; };
  bool operator!=(const Vector2& rhs) { return x != rhs.x || y != rhs.y; };
  double x, y;
};


struct TeleportRequest {
  TeleportRequest(double x, double y, double _theta, double _linear, \
      bool _relative) : pos(x, y), theta(_theta), linear(_linear), \
      relative(_relative) {};
  Vector2 pos;
  double theta;
  double linear;
  bool relative;
};


class Turtle {
public:
  Turtle(const ros::NodeHandle& nh, const Vector2& pos, double orientRad, \
      double z = 250.0, double s = 1.0);
  virtual ~Turtle();
  virtual void update(double dt, double canvasWidth, double canvasHeight);

  void setPose(double x, double y, double z, double angleRad) {
    poseMutex.lock();
    pos_.x = x;
    pos_.y = y;
    z_ = z;
    orient_ = angleRad;
    teleport_requests_.clear();
    poseMutex.unlock();
  };

  double x() { return pos_.x; };
  double y() { return pos_.y; };
  double z() { return z_; };

  double angleRad() { return orient_; };
  double angleDeg() { return orient_/M_PI*180.0; };

  virtual int type() { return 0; };

protected:
  void velocityXYZCallback(const sightedturtlesim::VelocityXYZConstPtr& msg);
  bool teleportAbsoluteXYZCallback( \
      sightedturtlesim::TeleportAbsoluteXYZ::Request&, \
      sightedturtlesim::TeleportAbsoluteXYZ::Response&);

  ros::NodeHandle nh_;

  Vector2 pos_;
  double orient_;
  double z_;
  double lin_vel_;
  double ang_vel_;
  double z_vel_;
  double scale;

  bool alive;

  ros::Subscriber velocity_xyz_sub_;
  ros::Publisher pose_xyz_pub_;
  ros::ServiceServer teleport_absolute_xyz_srv_;

  ros::WallTime last_command_time_;

  std::mutex poseMutex;

  constexpr static double COMMAND_TIMEOUT_SECS = 1.0;

  std::vector<TeleportRequest> teleport_requests_;
};


#endif /* TURTLE_HPP_ */
