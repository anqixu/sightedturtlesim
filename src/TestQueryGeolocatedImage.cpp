#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sightedturtlesim/ImageWithPoseXYZ.h>
#include <sightedturtlesim/QueryGeolocatedImage.h>
#include <iostream>


using namespace std;
using namespace cv;
using namespace cv_bridge;


class Robbie {
public:
  void callback(const sightedturtlesim::ImageWithPoseXYZ::ConstPtr& msg) {
    sightedturtlesim::QueryGeolocatedImage s;
    s.request.pose = msg->pose;
    if (!query.call(s)) {
      ROS_ERROR_STREAM("Service call to /turtle1/query_geolocated_image failed");
      return;
    }

    bool match = true;
    size_t size;
    if (msg->img.height != s.response.img.height) {
      ROS_ERROR("ERROR: height mismatch: %d / %d", msg->img.height, s.response.img.height);
      return;
    } else if (msg->img.width != s.response.img.width) {
      ROS_ERROR("ERROR: width mismatch: %d / %d", msg->img.width, s.response.img.width);
      return;
    } else if (msg->img.encoding != s.response.img.encoding) {
      ROS_ERROR("ERROR: encoding mismatch: %s / %s", msg->img.encoding.c_str(), s.response.img.encoding.c_str());
      return;
    } else if (msg->img.is_bigendian != s.response.img.is_bigendian) {
      ROS_ERROR("ERROR: is_bigendian mismatch: %d / %d", msg->img.is_bigendian, s.response.img.is_bigendian);
      return;
    } else if (msg->img.step != s.response.img.step) {
      ROS_ERROR("ERROR: step mismatch: %d / %d", msg->img.step, s.response.img.step);
      return;
    } else {
      size = msg->img.step * msg->img.height;
      std::vector<unsigned char>::const_iterator itA = msg->img.data.begin();
      std::vector<unsigned char>::const_iterator itB = s.response.img.data.begin();
      for (size_t i = 0; i < size; i++, itA++, itB++) {
        if (*itA != *itB) {
          match = false;
          ROS_ERROR("value mismatch at index %ld (%d)", i, msg->img.header.seq);

          cout << "x: " << msg->pose.x << endl;
          cout << "y: " << msg->pose.y << endl;
          cout << "z: " << msg->pose.z << endl;
          cout << "theta: " << msg->pose.theta << endl;
          cout << "linvel: " << msg->pose.linear_velocity << endl;
          cout << "linvelz: " << msg->pose.linear_velocity_z << endl;
          cout << "angvel: " << msg->pose.angular_velocity << endl;
          cout << "rxImg (" << msg->img.header.seq << "): " << msg->img.height << " x " << msg->img.width << " [";
          for (int i = 0; i < 5; i++) {
            cout << (unsigned short) msg->img.data[i] << " ";
          }
          cout << "]" << endl;
          cout << "queryImg (" << s.response.img.header.seq << "): " << s.response.img.height << " x " << s.response.img.width << " [";
          for (int i = 0; i < 5; i++) {
            cout << (unsigned short) s.response.img.data[i] << " ";
          }
          cout << "]" << endl;

          break;
        }
      }
    }

    if (match) {
      ROS_INFO("SUCCESS: img match (%d)", msg->img.header.seq);
    }

    boost::shared_ptr<void const> t1, t2;
    CvImageConstPtr sourceImg = toCvShare(msg->img, t1, "bgr8");
    CvImageConstPtr replayImg = toCvShare(s.response.img, t2, "bgr8");
    Mat sourceGray, replayGray, diffGray;
    cvtColor(sourceImg->image, sourceGray, CV_RGB2GRAY);
    cvtColor(replayImg->image, replayGray, CV_RGB2GRAY);
    absdiff(sourceGray, replayGray, diffGray);



    imshow("Source", sourceImg->image);
    imshow("Replay", replayImg->image);
    imshow("Diff", diffGray);
    waitKey(3);
  };

  Robbie() : nh() {
    ROS_INFO_STREAM("Waiting for service: /turtle1/query_geolocated_image");
    ros::service::waitForService("/turtle1/query_geolocated_image");
    query = nh.serviceClient<sightedturtlesim::QueryGeolocatedImage>("/turtle1/query_geolocated_image");
    sub = nh.subscribe("/turtle1/image_xyz", 1, &Robbie::callback, this);
    ROS_INFO_STREAM("test query node ready");
    namedWindow("Source");
    namedWindow("Replay");
    namedWindow("Diff");
  };

private:
  ros::NodeHandle nh;
  ros::ServiceClient query;
  ros::Subscriber sub;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "listener", ros::init_options::AnonymousName);
  Robbie r;
  ros::spin();
  std::cout << "Shutting down" << std::endl;
  destroyAllWindows();

  return 0;
};
