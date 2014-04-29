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


#ifndef TURTLEFRAME_HPP_
#define TURTLEFRAME_HPP_


#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <map>
#include "sightedturtlesim/Turtle.hpp"
#include "sightedturtlesim/VisionTurtle.hpp"
#include <boost/thread/mutex.hpp>


class AbstractImageServer;


typedef std::map<std::string, Turtle*> M_Turtle;
class TurtleFrame {
public:
  TurtleFrame(ros::NodeHandle& nh, AbstractImageServer* server);
  ~TurtleFrame();

  void updateImageServer(AbstractImageServer* newServer) { imageServer = newServer; };

  std::string spawnTurtle(const std::string& name, double x, double y, double angle);
  std::string spawnVisionTurtle( \
      double x = 0.0, double y = 0.0, double z = 100.0, double angle = 0.0, \
      unsigned int camW = VisionTurtle::DEFAULT_IMAGE_WIDTH, \
      unsigned int camH = VisionTurtle::DEFAULT_IMAGE_HEIGHT, \
      double fps = VisionTurtle::DEFAULT_FPS, \
      double scale = 1.0);

  bool deleteTurtle(const std::string& name);

  M_Turtle& getTurtles() { return turtles_; };
  unsigned int size() { return turtles_.size(); };

  void updateTurtles();
  void reset();

protected:
  void clear();
  bool hasTurtle(const std::string& name);

  bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool spawnCallback(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);
  bool killCallback(turtlesim::Kill::Request&, turtlesim::Kill::Response&);

  ros::NodeHandle nh_;

  ros::WallTime last_turtle_update_;

  ros::ServiceServer clear_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer spawn_srv_;
  ros::ServiceServer kill_srv_;

  M_Turtle turtles_;
  unsigned int freeRobotID;

  AbstractImageServer* imageServer;

  boost::mutex turtlesMutex;
};


#endif /* TURTLEFRAME_HPP_ */
