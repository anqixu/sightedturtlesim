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
  VisionTurtle(const ros::NodeHandle& nh,
      const sightedturtlesim::PoseXYZ& initPose,
      const sightedturtlesim::TurtleParams& params,
      AbstractImageServer* server,
      unsigned int id = 0,
      double hfovDeg = DEFAULT_HFOV_DEG,
      double aspectRatio = DEFAULT_ASPECT_RATIO,
      unsigned int imWidth = DEFAULT_IMAGE_WIDTH,
      unsigned int imHeight = DEFAULT_IMAGE_HEIGHT,
      double fps = DEFAULT_FPS);
  virtual ~VisionTurtle();

  void stopThread();
  void restartThread(AbstractImageServer* newServer);

  unsigned int getID() { return ID; };

  virtual int type() { return ID; };

  double getHFOVDeg() { return hfovDeg; };
  double getAspectRatio() { return aspectRatio; };
  unsigned int getImWidth() { return image.image.cols; };
  unsigned int getImHeight() { return image.image.rows; };
  double getFPS() { return 1.0/imageRate.expectedCycleTime().toSec(); };

  constexpr static unsigned int DEFAULT_IMAGE_WIDTH = 320;
  constexpr static unsigned int DEFAULT_IMAGE_HEIGHT = 240;
  constexpr static double DEFAULT_FPS = 15.0;
  constexpr static double DEFAULT_HFOV_DEG = 100.0;
  constexpr static double DEFAULT_ASPECT_RATIO = 4.0/3;

protected:
  void imagePoller();

  bool queryGeolocatedImageCallback(
      sightedturtlesim::QueryGeolocatedImage::Request&,
      sightedturtlesim::QueryGeolocatedImage::Response&);

  image_transport::ImageTransport imageTransport;
  image_transport::Publisher imagePub;
  ros::Publisher visionSettingsPub;
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
  double hfovDeg;
  double aspectRatio;
  int id;
  unsigned int imWidth;
  unsigned int imHeight;
  double fps;
  double z;
  double s;

  _VisionTurtleState() : x(0), y(0), orientRad(0),
    hfovDeg(100), aspectRatio(4./3),
    id(-1), imWidth(320), imHeight(240), fps(1), z(100), s(1) {};

  _VisionTurtleState(VisionTurtle* t) :
    x(t->x()), y(t->y()), orientRad(t->angleRad()),
    hfovDeg(t->getHFOVDeg()), aspectRatio(t->getAspectRatio()),
    id(t->getID()),
    imWidth(t->getImWidth()), imHeight(t->getImHeight()),
    fps(t->getFPS()), z(t->z()), s(t->getScale()) {};
} VisionTurtleState;


#endif /* VISIONTURTLE_HPP_ */
