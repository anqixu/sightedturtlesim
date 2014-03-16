#include "sightedturtlesim/VisionTurtle.hpp"
#include "sightedturtlesim/AbstractImageServer.hpp"
#include <std_msgs/Header.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>


using namespace std;


bool eq(const sightedturtlesim::PoseXYZ& a, const sightedturtlesim::PoseXYZ& b) {
  return (a.x == b.x && a.y == b.y && a.z == b.z && a.theta == b.theta && a.linear_velocity == b.linear_velocity && a.angular_velocity == b.angular_velocity && a.linear_velocity_z == b.linear_velocity_z);
};


VisionTurtle::VisionTurtle(const ros::NodeHandle& nh,
    const Vector2& pos, double orientRad, AbstractImageServer* server,
    unsigned int id, unsigned int imWidth, unsigned int imHeight,
    double fps, double z, double s) : Turtle(nh, pos, orientRad, z, s),
        imageTransport(nh_), imageRate(fps),
        imageServer(server), imageThread(),
        ID(id), imageSeqCount(0),
        hfovDeg(DEFAULT_HFOV_DEG), aspectRatio(DEFAULT_ASPECT_RATIO) {
  // Connect to ROS hooks
  imagePub = imageTransport.advertise("image_raw", 1);
  geolocatedImagePub = nh_.advertise<sightedturtlesim::ImageWithPoseXYZ>("image_xyz", 1);
  queryGeolocatedImageSrv = nh_.advertiseService("query_geolocated_image",
      &VisionTurtle::queryGeolocatedImageCallback, this);

  // Setup image buffer
  image.header.seq = ++imageSeqCount;
  image.header.stamp = ros::Time::now();
  image.header.frame_id = "";
  image.encoding = "bgr8";
  image.image = cv::Mat::zeros(imHeight, imWidth, CV_8UC3);
  image_req.header.seq = 0;
  image_req.header.stamp = ros::Time::now();
  image_req.header.frame_id = "";
  image_req.encoding = "bgr8";
  image_req.image = cv::Mat::zeros(imHeight, imWidth, CV_8UC3);

  // Start image thread
  imageThread = boost::thread(boost::bind(&VisionTurtle::imagePoller, this));
};


VisionTurtle::~VisionTurtle() {
  alive = false;
  imageThread.interrupt();
  imageThread.join();
};


void VisionTurtle::imagePoller() {
  double cornersXYBuffer[8];

  try {
    while (ros::ok() && alive) {
      if (imagePub.getNumSubscribers() > 0 || geolocatedImagePub.getNumSubscribers() > 0) {
        poseMutex.lock();

        // Check if current pose is same as previously published pose (which must have a populated image)
        bool same = (
            (imageWithPoseMsg.pose.x == pos_.x) &&
            (imageWithPoseMsg.pose.y == pos_.y) &&
            (imageWithPoseMsg.pose.theta == orient_) &&
            (imageWithPoseMsg.pose.linear_velocity == lin_vel_) &&
            (imageWithPoseMsg.pose.angular_velocity == ang_vel_) &&
            (imageWithPoseMsg.pose.z == z_) &&
            (imageWithPoseMsg.pose.linear_velocity_z == z_vel_) &&
            (imageWithPoseMsg.img.step * imageWithPoseMsg.img.width > 0));

        // Fetch new image if cache not found
        if (!same) {
          imageWithPoseMsg.pose.x = pos_.x;
          imageWithPoseMsg.pose.y = pos_.y;
          imageWithPoseMsg.pose.theta = orient_;
          imageWithPoseMsg.pose.linear_velocity = lin_vel_;
          imageWithPoseMsg.pose.angular_velocity = ang_vel_;
          imageWithPoseMsg.pose.z = z_;
          imageWithPoseMsg.pose.linear_velocity_z = z_vel_;

          imageServer->getImage(imageWithPoseMsg.pose.x * imageServer->pixelsPerMeter(),
              imageWithPoseMsg.pose.y * imageServer->pixelsPerMeter(),
              -imageWithPoseMsg.pose.theta/M_PI*180.0 + 90.0,
              imageWithPoseMsg.pose.z * imageServer->pixelsPerMeter(),
              hfovDeg, aspectRatio, image.image);
          AbstractImageServer::toCornersXY(
              imageWithPoseMsg.pose.x * imageServer->pixelsPerMeter(),
              imageWithPoseMsg.pose.y * imageServer->pixelsPerMeter(),
              -imageWithPoseMsg.pose.theta/M_PI*180.0 + 90.0,
              imageWithPoseMsg.pose.z * imageServer->pixelsPerMeter(),
              hfovDeg, aspectRatio,
              cornersXYBuffer);
          for (int i = 0; i < 8; i++) imageWithPoseMsg.imgCornersXY[i] = cornersXYBuffer[i] / imageServer->pixelsPerMeter();
        }

        // Update CV_Image's header, whether its image was updated or not
        imageWithPoseMsg.pose.img_seq = image.header.seq = imageSeqCount++;
        image.header.stamp = ros::Time::now();

        // Convert to ROS image buffer
        // NOTE: always storing to imageWithPoseMsg, to accommodate caching for queryGeolocatedImageCallback
        image.toImageMsg(imageWithPoseMsg.img);

        poseMutex.unlock();

        // Publish ROS image buffers to subscribers
        if (imagePub.getNumSubscribers() > 0) {
          imagePub.publish(imageWithPoseMsg.img);
        }
        if (geolocatedImagePub.getNumSubscribers() > 0) {
          geolocatedImagePub.publish(imageWithPoseMsg);
        }
      }
      boost::this_thread::interruption_point();
      imageRate.sleep(); // NOTE: depending on the specified frame rate, this could take a long time
      boost::this_thread::interruption_point();
    }
  } catch (boost::thread_interrupted& err) {
  }
};


bool VisionTurtle::queryGeolocatedImageCallback(
    sightedturtlesim::QueryGeolocatedImage::Request& req,
    sightedturtlesim::QueryGeolocatedImage::Response& res) {
  double cornersXYBuffer[8];

  poseMutex.lock();

  bool same = eq(imageWithPoseMsg.pose, req.pose);

  if (same) {
    res.img = imageWithPoseMsg.img; // NOTE: confirmed to perform deep copy
  } else {
    imageServer->getImage(req.pose.x * imageServer->pixelsPerMeter(),
        req.pose.y * imageServer->pixelsPerMeter(),
        -req.pose.theta/M_PI*180.0 + 90.0, req.pose.z * imageServer->pixelsPerMeter(),
        hfovDeg, aspectRatio, image_req.image);
    image_req.toImageMsg(res.img);

    AbstractImageServer::toCornersXY(
        req.pose.x * imageServer->pixelsPerMeter(),
        req.pose.y * imageServer->pixelsPerMeter(),
        -req.pose.theta/M_PI*180.0 + 90.0, req.pose.z * imageServer->pixelsPerMeter(),
        hfovDeg, aspectRatio,
        cornersXYBuffer);
    for (int i = 0; i < 8; i++) res.imgCornersXY[i] = cornersXYBuffer[i] / imageServer->pixelsPerMeter();
  }

  poseMutex.unlock();

  return true;
};
