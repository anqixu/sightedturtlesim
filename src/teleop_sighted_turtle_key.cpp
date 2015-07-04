#include <ros/ros.h>
#include <sightedturtlesim/VelocityXYZ.h>
#include <csignal>
#include <cstdio>
#include <termios.h>
#include <thread>

class TeleopSightedTurtle {
public:
  TeleopSightedTurtle() : alive(true) {
    nh_.param("scale_linear", scale.linear_xy, 200.0);
    nh_.param("scale_linear_z", scale.linear_z, 50.0);
    nh_.param("scale_angular", scale.angular_xy, 3.1415/2.0);

    cmd_pub_ = nh_.advertise<sightedturtlesim::VelocityXYZ>("turtle1/command_velocity_xyz", 1);
  };
  void keyLoop();
  void spin();

  bool alive;

private:
  ros::NodeHandle nh_;
  sightedturtlesim::VelocityXYZ cmd;
  sightedturtlesim::VelocityXYZ scale;
  ros::Publisher cmd_pub_;
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_sighted_turtle");

  TeleopSightedTurtle teleop;

  signal(SIGINT,quit);

  std::thread spinThread(&TeleopSightedTurtle::spin, &teleop);
  teleop.keyLoop();
  spinThread.join();
  
  return(0);
};


void TeleopSightedTurtle::keyLoop() {
  char c;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard / numpad");
  puts("---------------------------");
  puts("8/2: increase/decrease linear velocity");
  puts("4/6: change angular velocity");
  puts("7/1: increase/decrease z linear velocity");
  puts("5: reset velocity");
  puts("x: exit");
  puts("---------------------------");


  while(alive) {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    ROS_INFO("value: 0x%02X\n", c);
  
    switch(c) {
    case '8':
      ROS_DEBUG("INCREASE SPEED");
      cmd.linear_xy = std::min(cmd.linear_xy + 0.1, 1.0);
      break;
    case '2':
      ROS_DEBUG("DECREASE SPEED");
      cmd.linear_xy = std::max(cmd.linear_xy - 0.1, 0.0);
      break;
    case '7':
      ROS_DEBUG("INCREASE Z");
      cmd.linear_z = std::min(cmd.linear_z + 0.1, 1.0);
      break;
    case '1':
      ROS_DEBUG("DECREASE Z");
      cmd.linear_z = std::max(cmd.linear_z - 0.1, -1.0);
      break;
    case '4':
      ROS_DEBUG("LEFT");
      cmd.angular_xy = std::min(cmd.angular_xy + 0.1, 1.0);
      break;
    case '6':
      ROS_DEBUG("RIGHT");
      cmd.angular_xy = std::max(cmd.angular_xy - 0.1, -1.0);
      break;
    case '5':
      ROS_DEBUG("RESET");
      cmd.linear_xy = 0;
      cmd.angular_xy = 0;
      cmd.linear_z = 0;
      break;
    case 'x':
    case 'X':
      alive = false;
      break;
    case ' ':
      ROS_INFO("teleop_sighted_turtle_key\nlinear_xy: %.1f\nangular_xy: %.1f\nlinear_z: %.1f", cmd.linear_xy, cmd.angular_xy, cmd.linear_z);
    }
  }

  return;
};

void TeleopSightedTurtle::spin() {
  ros::Rate hz(10);
  while (ros::ok() && alive) {
    sightedturtlesim::VelocityXYZ cmdScaled;
    cmdScaled.linear_xy = cmd.linear_xy * scale.linear_xy;
    cmdScaled.linear_z = cmd.linear_z * scale.linear_z;
    cmdScaled.angular_xy = cmd.angular_xy * scale.angular_xy;
    cmd_pub_.publish(cmdScaled);
    hz.sleep();
  }
};
