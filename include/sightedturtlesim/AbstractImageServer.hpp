#ifndef ABSTRACTIMAGESERVER_HPP_
#define ABSTRACTIMAGESERVER_HPP_


#include <opencv2/core/core.hpp>


class AbstractImageServer {
public:
  AbstractImageServer(double ppm = 1.0);
  virtual ~AbstractImageServer();

  // cornersXY = [topLeftX, topLeftY, topRightX, topRightY,
  //              bottomRightX, bottomRightY, bottomLeftX, bottomLeftY]
  //
  // NOTE: if !buffer.empty(), then resulting image size = buffer.size()
  virtual void getImage(double* cornersXY, cv::Mat& buffer, bool& isWrapped) = 0;

  // NOTE: if !buffer.empty(), then resulting image size = buffer.size()
  virtual void getImage(double x, double y, double upDeg,
      cv::Mat& buffer, bool& isWrapped, double camW = 0, double camH = 0) = 0;
  void getImage(double x, double y, double upDeg,
      double z, double hfovDeg, double aspectRatio, cv::Mat& buffer, bool& isWrapped) {
    double camW = z * atan(hfovDeg / 90.0 * M_PI);
    double camH = camW / aspectRatio;
    getImage(x, y, upDeg, buffer, isWrapped, camW, camH);
  };

  double getMidX() { return _width / 2.0 / _pixelsPerMeter; };
  double getMidY() { return _height / 2.0 / _pixelsPerMeter; };

  virtual const cv::Mat& canvas() = 0;

  long long width() { return _width; };
  long long height() { return _height; };
  double pixelsPerMeter() { return _pixelsPerMeter; };
  void setCanvasSize(long long w, long long h) { _width = w; _height = h; };

  static void toCornersXY(double x, double y, double headingDeg,
      double z, double hfovDeg, double aspectRatio, double* cornersXYBuffer);
  static void toCornersXY(double x, double y, double headingDeg,
      double camW, double camH, double* cornersXYBuffer);

  // If the desired camera dimensions (with respect to the canvas dimensions)
  // exceeds DOWNSIZE_SCALE_RATIO times the dimensions of the desired image,
  // then the bounding box should be downsized prior to rotation, to ensure
  // that the downsized camera dimensions will be exactly DOWNSIZE_SCALE_RATIO
  // times the size of the desired image
  constexpr static double DOWNSIZE_SCALE_RATIO = 1.5;

protected:
  long long _width;
  long long _height;

  double _pixelsPerMeter;
};


#endif /* ABSTRACTIMAGESERVER_HPP_ */
