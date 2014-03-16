#ifndef SINGLEIMAGESERVER_HPP_
#define SINGLEIMAGESERVER_HPP_


#include "sightedturtlesim/AbstractImageServer.hpp"
#include <string>


//#define DEBUG_SINGLE_IMAGE_SERVER
//#define GET_IMAGE_PRINT_CROP_RANGE


class SingleImageServer : public AbstractImageServer {
public:
  SingleImageServer(const std::string& imageFilename, double ppm = 1.0);
  virtual ~SingleImageServer();

  // cornersXY = [topLeftX, topLeftY, topRightX, topRightY,
  //              bottomRightX, bottomRightY, bottomLeftX, bottomLeftY]
  //
  // NOTE: if !buffer.empty(), then resulting image size = buffer.size()
  virtual void getImage(double* cornersXY, cv::Mat& buffer);

  // NOTE: if !buffer.empty(), then resulting image size = buffer.size()
  virtual void getImage(double x, double y, double upDeg, \
      cv::Mat& buffer, double camW = 0, double camH = 0);

  virtual const cv::Mat& canvas() { return _canvas; };

protected:
  cv::Mat _canvas;
};


#endif /* SINGLEIMAGESERVER_HPP_ */
