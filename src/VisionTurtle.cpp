#include "sightedturtlesim/VisionTurtle.hpp"
#include "sightedturtlesim/AbstractImageServer.hpp"
#include <std_msgs/Header.h>
#include <sightedturtlesim/VisionTurtleSettings.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>


using namespace std;


// NOTE: since this fn is used for fetching images, only care about state, and not velocity/acceleration/controls
inline bool isSamePose(const sightedturtlesim::PoseXYZ& a, const sightedturtlesim::PoseXYZ& b) {
  return (a.x == b.x && a.y == b.y && a.z == b.z && a.theta == b.theta);
};


VisionTurtle::VisionTurtle(const ros::NodeHandle& nh,
    const sightedturtlesim::PoseXYZ& initPose,
    const sightedturtlesim::TurtleParams& params,
    AbstractImageServer* server, unsigned int id,
    double hfovDeg, double aspectRatio,
    unsigned int imWidth, unsigned int imHeight,
    double fps) : Turtle(nh, initPose, params),
        imageTransport(nh_), imageRate(fps),
        imageServer(server), imageThread(),
        ID(id), imageSeqCount(0),
        hfovDeg(hfovDeg), aspectRatio(aspectRatio) {
  // Connect to ROS hooks
  imagePub = imageTransport.advertise("image_raw", 1); // WARNING: this can take some time to start, e.g. ~1sec
  visionSettingsPub = nh_.advertise<sightedturtlesim::VisionTurtleSettings>("vision_settings", 1, true); // latch=true
  geolocatedImagePub = nh_.advertise<sightedturtlesim::ImageWithPoseXYZ>("image_xyz", 1);
  queryGeolocatedImageSrv = nh_.advertiseService("query_geolocated_image",
      &VisionTurtle::queryGeolocatedImageCallback, this);

  // Publish vision settings
  sightedturtlesim::VisionTurtleSettings s;
  s.camera_hfov_deg = hfovDeg;
  s.camera_aspect_ratio = aspectRatio;
  s.image_width = imWidth;
  s.image_height = imHeight;
  s.image_fps = fps;
  visionSettingsPub.publish(s);

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
  stopThread();
};


void VisionTurtle::stopThread() {
  alive = false;
  imageThread.interrupt();
  imageThread.join();
};


void VisionTurtle::restartThread(AbstractImageServer* newServer) {
  if (alive) {
    stopThread();
  }
  if (newServer != NULL) {
    imageServer = newServer;
  } else {
    ROS_WARN_STREAM("VisionTurtle::restartThread called with newServer=NULL; not recommended; things may go wrong!");
  }
  alive = true;
  imageThread = boost::thread(boost::bind(&VisionTurtle::imagePoller, this));
};


void VisionTurtle::imagePoller() {
  double cornersXYBuffer[8];
  bool firstPoll = true; // always fetch new image upon start

  try {
    while (ros::ok() && alive) {
      if (imagePub.getNumSubscribers() > 0 || geolocatedImagePub.getNumSubscribers() > 0) {
        poseMutex.lock();

        // Check if current pose is same as previously published pose (which must have a populated image)
        bool samePose = isSamePose(imageWithPoseMsg.pose, pos_) &&
            (imageWithPoseMsg.img.step * imageWithPoseMsg.img.width > 0);
        if (firstPoll) {
          samePose = false;
          firstPoll = false;
        }

        // Fetch new image if cache not found
        if (!samePose) {
          imageWithPoseMsg.pose = pos_;

          bool isWrapped;
          imageServer->getImage(imageWithPoseMsg.pose.x * imageServer->pixelsPerMeter(),
              imageWithPoseMsg.pose.y * imageServer->pixelsPerMeter(),
              -imageWithPoseMsg.pose.theta/M_PI*180.0 + 90.0,
              imageWithPoseMsg.pose.z * imageServer->pixelsPerMeter(),
              hfovDeg, aspectRatio, image.image, isWrapped);
          imageWithPoseMsg.isWrapped = isWrapped;
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
        imageWithPoseMsg.pose.header.seq = image.header.seq = imageSeqCount;
        imageWithPoseMsg.pose.header.stamp = image.header.stamp = ros::Time::now();
        imageSeqCount += 1;

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
  if (!alive) {
    for (int i = 0; i < 8; i++) res.imgCornersXY[i] = 0;
    return true;
  }

  double cornersXYBuffer[8];

  poseMutex.lock();

  if (isSamePose(imageWithPoseMsg.pose, req.pose)) {
    res.img = imageWithPoseMsg.img; // NOTE: confirmed to perform deep copy
    res.isWrapped = imageWithPoseMsg.isWrapped;
  } else {
    bool isWrapped;
    imageServer->getImage(req.pose.x * imageServer->pixelsPerMeter(),
        req.pose.y * imageServer->pixelsPerMeter(),
        -req.pose.theta/M_PI*180.0 + 90.0, req.pose.z * imageServer->pixelsPerMeter(),
        hfovDeg, aspectRatio, image_req.image, isWrapped);
    res.isWrapped = isWrapped;
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
