#include "sightedturtlesim/AbstractImageServer.hpp"
#include <cmath>


AbstractImageServer::AbstractImageServer(double ppm) :
    _width(0), _height(0), _pixelsPerMeter(ppm) {

};


AbstractImageServer::~AbstractImageServer() {

};


void AbstractImageServer::toCornersXY(double x, double y, double headingDeg,
      double camW, double camH, double* cornersXYBuffer) {
  double thetaRad = -headingDeg/180.0*M_PI;
  cv::Mat camTransform(2, 2, CV_64FC1);
  double* camT = camTransform.ptr<double>();
  camT[0] = cos(thetaRad);  camT[1] = sin(thetaRad);
  camT[2] = -sin(thetaRad); camT[3] = cos(thetaRad);
  cv::Mat camCorners(2, 4, CV_64FC1);
  double* camC = camCorners.ptr<double>();
  camC[0] = -camW/2; camC[1] = +camW/2; camC[2] = +camW/2; camC[3] = -camW/2;
  camC[4] = -camH/2; camC[5] = -camH/2; camC[6] = +camH/2; camC[7] = +camH/2;
  cv::Mat camTransformedCorners = camTransform*camCorners;
  double* camTC = camTransformedCorners.ptr<double>();
  cornersXYBuffer[0] = camTC[0] + x;
  cornersXYBuffer[1] = camTC[4] + y;
  cornersXYBuffer[2] = camTC[1] + x;
  cornersXYBuffer[3] = camTC[5] + y;
  cornersXYBuffer[4] = camTC[2] + x;
  cornersXYBuffer[5] = camTC[6] + y;
  cornersXYBuffer[6] = camTC[3] + x;
  cornersXYBuffer[7] = camTC[7] + y;
};
