#ifndef VISIONTURTLE_HPP_
#define VISIONTURTLE_HPP_


#include "sightedturtlesim/Turtle.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sightedturtlesim/QueryGeolocatedImage.h>
#include <sightedturtlesim/ImageWithPoseXYZ.h>
#include <boost/thread/thread.hpp>


class AbstractImageServer;


class VisionTurtle : public Turtle {
public:
  VisionTurtle(const ros::NodeHandle& nh, \
      const Vector2& pos, double orientRad, \
      AbstractImageServer* server, \
      unsigned int id = 0, \
      unsigned int imWidth = DEFAULT_IMAGE_WIDTH, \
      unsigned int imHeight = DEFAULT_IMAGE_HEIGHT, \
      double fps = DEFAULT_FPS, double z = 250.0, \
      double s = 1.0);
  virtual ~VisionTurtle();

  unsigned int getID() { return ID; };

  virtual int type() { return ID; };

  constexpr static unsigned int DEFAULT_IMAGE_WIDTH = 320;
  constexpr static unsigned int DEFAULT_IMAGE_HEIGHT = 240;
  constexpr static double DEFAULT_FPS = 15.0;
  constexpr static double DEFAULT_HFOV_DEG = 46.0;
  constexpr static double DEFAULT_ASPECT_RATIO = 4.0/3;

protected:
  void imagePoller();

  bool queryGeolocatedImageCallback( \
      sightedturtlesim::QueryGeolocatedImage::Request&, \
      sightedturtlesim::QueryGeolocatedImage::Response&);

  image_transport::ImageTransport imageTransport;
  image_transport::Publisher imagePub;
  ros::Publisher geolocatedImagePub;
  ros::ServiceServer queryGeolocatedImageSrv;

  cv_bridge::CvImage image, image_req;
  ros::Rate imageRate;
  AbstractImageServer* imageServer;
  boost::thread imageThread;

  unsigned int ID;
  unsigned int imageSeqCount;

  double hfovDeg;
  double aspectRatio;

  sightedturtlesim::ImageWithPoseXYZ imageWithPoseMsg;
};

#endif /* VISIONTURTLE_HPP_ */
