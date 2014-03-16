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

#include "sightedturtlesim/TurtleFrame.hpp"
#include "sightedturtlesim/VisionTurtle.hpp"
#include "sightedturtlesim/AbstractImageServer.hpp"
#include <cstdlib>
#include <ctime>


TurtleFrame::TurtleFrame(ros::NodeHandle& nh, \
    AbstractImageServer* server) : nh_(nh), \
    freeRobotID(1), imageServer(server), turtlesMutex() {
  //clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
  //spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
  //kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);

  ROS_INFO_STREAM("Starting turtlesim with node name " << ros::this_node::getName()) ;

  //spawnVisionTurtle();
};


TurtleFrame::~TurtleFrame() {
  reset();
};


bool TurtleFrame::spawnCallback(turtlesim::Spawn::Request& req, \
    turtlesim::Spawn::Response& res) {
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
  if (name.empty()) {
    ROS_ERROR_STREAM("A turtled named [" << req.name << "] already exists");
    return false;
  }
  res.name = name;
  return true;
};


bool TurtleFrame::killCallback(turtlesim::Kill::Request& req, \
    turtlesim::Kill::Response&) {
  bool result = false;

  turtlesMutex.lock();

  M_Turtle::iterator it = turtles_.find(req.name);
  if (it == turtles_.end()) {
    ROS_ERROR_STREAM("Tried to kill turtle [" << req.name << "], which does not exist");
  } else {
    delete it->second;
    turtles_.erase(it);
    result = true;
  }

  turtlesMutex.unlock();

  return result;
};


bool TurtleFrame::deleteTurtle(const std::string& name) {
  bool deleted = false;

  turtlesMutex.lock();

  M_Turtle::iterator itTurtles = turtles_.begin();
  M_Turtle::iterator itTurtlesEnd = turtles_.end();
  for (; itTurtles != itTurtlesEnd; itTurtles++) {
    if (itTurtles->first == name) {
      delete itTurtles->second;
      turtles_.erase(itTurtles);
      deleted = true;
      break;
    }
  }

  turtlesMutex.unlock();

  return deleted;
};


bool TurtleFrame::hasTurtle(const std::string& name) {
  turtlesMutex.lock();
  bool result = turtles_.find(name) != turtles_.end();
  turtlesMutex.unlock();
  return result;
};


std::string TurtleFrame::spawnTurtle(const std::string& name, \
    double x, double y, double angle) {
  Turtle* t;
  std::string real_name = name;
  if (real_name.empty()) {
    do {
      std::stringstream ss;
      ss << "turtle" << freeRobotID++;
      real_name = ss.str();
    } while (hasTurtle(real_name));
    t = new VisionTurtle(ros::NodeHandle(real_name), Vector2(x, y), angle, \
        imageServer, freeRobotID - 1);
  } else {
    if (hasTurtle(real_name)) {
      return "";
    }
    t = new Turtle(ros::NodeHandle(real_name), Vector2(x, y), angle);
  }

  turtlesMutex.lock();
  turtles_[real_name] = t;
  turtlesMutex.unlock();

  ROS_INFO_STREAM("Spawning turtle [" << real_name << "] at x=[" << x << \
      "], y=[" << y << "], theta=[" << angle << "]");

  return real_name;
};


std::string TurtleFrame::spawnVisionTurtle(double x, double y, double z, \
    double angle, unsigned int camW, unsigned int camH, double fps, double scale) {
  std::string real_name;
  do {
    real_name = "turtle" + boost::lexical_cast<std::string>(freeRobotID++);
  } while (hasTurtle(real_name));

  Turtle* t = new VisionTurtle(ros::NodeHandle(real_name), Vector2(x, y), \
      angle, imageServer, freeRobotID - 1, camW, camH, fps, z, scale);

  turtlesMutex.lock();
  turtles_[real_name] = t;
  turtlesMutex.unlock();

  ROS_INFO_STREAM("Spawning turtle [" << real_name << "] at x=[" << x << \
      "], y=[" << y << "], theta=[" << angle << "]");

  return real_name;
};


void TurtleFrame::clear() {
};


void TurtleFrame::reset() {
  turtlesMutex.lock();
  M_Turtle::iterator itTurtles = turtles_.begin();
  M_Turtle::iterator itTurtlesEnd = turtles_.end();
  for (; itTurtles != itTurtlesEnd; itTurtles++) {
    delete itTurtles->second;
  }
  turtles_.clear();
  turtlesMutex.unlock();
  freeRobotID = 1;
  clear();
};


void TurtleFrame::updateTurtles() {
  if (last_turtle_update_.isZero()) {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }
  ros::WallTime now = ros::WallTime::now();
  ros::WallDuration td = now - last_turtle_update_;
  last_turtle_update_ = now;

  turtlesMutex.lock();
  M_Turtle::iterator itTurtles = turtles_.begin();
  M_Turtle::iterator itTurtlesEnd = turtles_.end();
  for (; itTurtles != itTurtlesEnd; itTurtles++) {
    itTurtles->second->update(td.toSec(), \
        imageServer->width() / imageServer->pixelsPerMeter(), \
        imageServer->height() / imageServer->pixelsPerMeter());
  }
  turtlesMutex.unlock();
};


bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, \
    std_srvs::Empty::Response&) {
  ROS_INFO("Clearing turtlesim.");
  clear();
  return true;
};


bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, \
    std_srvs::Empty::Response&) {
  ROS_INFO("Resetting turtlesim.");
  reset();
  return true;
};
