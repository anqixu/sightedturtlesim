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

  void stopThread();
  void restartThread(AbstractImageServer* newServer);

  unsigned int getID() { return ID; };

  virtual int type() { return ID; };

  unsigned int getImWidth() { return image.image.cols; };
  unsigned int getImHeight() { return image.image.rows; };
  double getFPS() { return 1.0/imageRate.expectedCycleTime().toSec(); };

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


typedef struct _VisionTurtleState {
  double x;
  double y;
  double orientRad;
  int id;
  unsigned int imWidth;
  unsigned int imHeight;
  double fps;
  double z;
  double s;

  _VisionTurtleState() : x(0), y(0), orientRad(0), id(0),
      imWidth(0), imHeight(0), fps(0), z(0), s(0) {};

  _VisionTurtleState(VisionTurtle* t) :
    x(t->x()), y(t->y()), orientRad(t->angleRad()), id(t->getID()),
    imWidth(t->getImWidth()), imHeight(t->getImHeight()),
    fps(t->getFPS()), z(t->z()), s(t->getScale()) {};
} VisionTurtleState;


#endif /* VISIONTURTLE_HPP_ */
