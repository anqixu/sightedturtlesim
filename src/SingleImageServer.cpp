#include "sightedturtlesim/SingleImageServer.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION >= 3
// NOTE: in OpenCV 3, imread() was moved from highgui from imgcodecs
#include <opencv2/imgcodecs.hpp>
#endif
#include <iostream>
#ifdef DEBUG_SINGLE_IMAGE_SERVER
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#endif


using namespace cv;


SingleImageServer::SingleImageServer(const std::string& imageFilename,
    double ppm) :
    AbstractImageServer(ppm), _imageFilename() {
  _canvas = cv::imread(imageFilename, 1);
  if (_canvas.empty()) {
    throw std::string("Unable to read image: " + imageFilename);
  }
  _imageFilename = imageFilename;
  setCanvasSize(_canvas.cols, _canvas.rows);
};


SingleImageServer::SingleImageServer(cv::Mat cachedImage,
    const std::string& cachedImageFilename, double ppm) :
    AbstractImageServer(ppm), _imageFilename(cachedImageFilename) {
  _canvas = cachedImage;
  setCanvasSize(_canvas.cols, _canvas.rows);
};


SingleImageServer::~SingleImageServer() {
};


void SingleImageServer::getImage(double* cornersXY, cv::Mat& buffer) {
  std::cerr << "ERROR: SingleImageServer::getImage(double* cornersXY, cv::Mat& buffer) not implemented" << std::endl;
  // TODO: 9 implement getImage through pixel sampling
};


void SingleImageServer::getImage(double x, double y, double upDeg,
    cv::Mat& buffer, double camW, double camH) {
  if (buffer.empty() || buffer.rows <= 0 || buffer.cols <= 0) {
    return;
  }

#ifdef DEBUG_SINGLE_IMAGE_SERVER
  boost::posix_time::ptime tic = boost::posix_time::microsec_clock::local_time();
#endif

  // Process inputs
  double thetaRad = -upDeg/180.0*M_PI;
  if (camW <= 0 || camH <= 0) {
    camW = buffer.cols;
    camH = buffer.rows;
  }
  camW = std::max(round(camW), 1.0);
  camH = std::max(round(camH), 1.0);

  // Compute the bounding box width and height of the (rotated) camera frame
  cv::Mat camTransform(2, 2, CV_64FC1);
  double* camT = camTransform.ptr<double>();
  camT[0] = cos(thetaRad);  camT[1] = sin(thetaRad);
  camT[2] = -sin(thetaRad); camT[3] = cos(thetaRad);
  cv::Mat camCorners(2, 4, CV_64FC1);
  double* camC = camCorners.ptr<double>();
  camC[0] = -camW/2; camC[1] = -camW/2; camC[2] =  camW/2; camC[3] = camW/2;
  camC[4] = -camH/2; camC[5] =  camH/2; camC[6] = -camH/2; camC[7] = camH/2;
  cv::Mat camTransformedCorners = camTransform*camCorners;
  double* camTC = camTransformedCorners.ptr<double>();
  camTC[0] += x; camTC[1] += x; camTC[2] += x; camTC[3] += x;
  camTC[4] += y; camTC[5] += y; camTC[6] += y; camTC[7] += y;
  double camTXMax = (max(max(camTC[0], camTC[1]), max(camTC[2], camTC[3])));
  double camTXMin = (min(min(camTC[0], camTC[1]), min(camTC[2], camTC[3])));
  double camTYMax = (max(max(camTC[4], camTC[5]), max(camTC[6], camTC[7])));
  double camTYMin = (min(min(camTC[4], camTC[5]), min(camTC[6], camTC[7])));

  // Decide to slightly over-sample the bounding box if rotation angle is not exact
  double upDegMod90 = upDeg - floor(upDeg / 90.0)*90.0;
  if (upDegMod90 < -45.0) { upDegMod90 += 90.0; }
  else if (upDegMod90 > 45.0) { upDegMod90 -= 45.0; }
  if (abs(upDegMod90) > 5.0) { // If upDeg is not within +/- 5' away from 0', 90', 180', or 270'
    camTXMax += 1.;
    camTXMin -= 1.;
    camTYMax += 1.;
    camTYMin -= 1.;
  }

  // Extract the sub-window corresponding to the bounding box
  cv::Mat bbImage;
  if (round(camTXMin) >= 0 && round(camTXMax) < (int) _width &&
      round(camTYMin) >= 0 && round(camTYMax) < (int) _height) {
#ifdef GET_IMAGE_PRINT_CROP_RANGE
    std::cout << "bbImage = _canvas(" << camTYMin << ":" << camTYMax <<
        ", " << camTXMin << ":" << camTXMax << ")" << std::endl;
#endif

    bbImage = _canvas(Range(round(camTYMin), round(camTYMax) + 1),
        Range(round(camTXMin), round(camTXMax) + 1));
  } else {
    bbImage.create(round(camTYMax - camTYMin + 1), round(camTXMax - camTXMin + 1), _canvas.type());
    double currCamTY, currCamTX, currCamTYMod, currCamTXMod, patchHeight, patchWidth;
    long long bbY, bbX;
    for (currCamTY = camTYMin, bbY = 0; currCamTY <= camTYMax;
        currCamTY += patchHeight, bbY += patchHeight) {
      currCamTYMod = (long long) (round(currCamTY)) % (long long) (_height);
      if (currCamTYMod < 0) { currCamTYMod += _height; }
      patchHeight = round(min(camTYMax - currCamTY + 1, _height - currCamTYMod));

      for (currCamTX = camTXMin, bbX = 0; currCamTX <= camTXMax;
          currCamTX += patchWidth, bbX += patchWidth) {
        currCamTXMod = (long long) (round(currCamTX)) % (long long) (_width);
        if (currCamTXMod < 0) { currCamTXMod += _width; }
        patchWidth = round(min(camTXMax - currCamTX + 1, _width - currCamTXMod));

#ifdef GET_IMAGE_PRINT_CROP_RANGE
    std::cout << "bbImage(" << bbY << ":" << bbY + patchHeight << ", " <<
        bbX << ":" << bbX + patchWidth << ") = _canvas(" <<
        currCamTYMod << ":" << currCamTYMod + patchHeight <<
        ", " << currCamTXMod << ":" << currCamTXMod + patchWidth << ")" << std::endl;
    std::cout << "- currCamTX: " << currCamTX << "; camTXMin: " << camTXMin << "; camTXMax: " <<
        camTXMax << "; patchWidth: " << patchWidth << "; currCamTXMod: " << currCamTXMod <<
        "; _width: " << _width << std::endl;
#endif

        cv::Mat bbPatch = bbImage(Range(bbY, bbY + patchHeight), Range(bbX, bbX + patchWidth));
        _canvas(Range(currCamTYMod, currCamTYMod + patchHeight),
            Range(currCamTXMod, currCamTXMod + patchWidth)).copyTo(bbPatch);
      }
    }
  }

  // Decide to downsize image if necessary
  if (camW > DOWNSIZE_SCALE_RATIO*buffer.cols &&
      camH > DOWNSIZE_SCALE_RATIO*buffer.rows) {
    cv::Mat bbScaledImage;
    double downsizeFactor = max(DOWNSIZE_SCALE_RATIO*buffer.cols/camW,
        DOWNSIZE_SCALE_RATIO*buffer.rows/camH);
    resize(bbImage, bbScaledImage, Size(), downsizeFactor, downsizeFactor, INTER_AREA);
    bbImage = bbScaledImage;
    camW = camW * downsizeFactor;
    camH = camH * downsizeFactor;
  }

  // Compute the width and height of the rotated bounding box
  // and adjust the centers of the transformation matrix
  cv::Mat bbTransform = cv::getRotationMatrix2D(
      Point2f(bbImage.cols/2, bbImage.rows/2), upDeg, 1.0);
  cv::Mat bbCorners = cv::Mat(3, 4, CV_64FC1);
  double* bbC = bbCorners.ptr<double>();
  bbC[0] = 0;  bbC[1] = 0;            bbC[2] = bbImage.cols; bbC[3] = bbImage.cols;
  bbC[4] = 0;  bbC[5] = bbImage.rows; bbC[6] = 0;            bbC[7] = bbImage.rows;
  bbC[8] = 1;  bbC[9] = 1;            bbC[10] = 1;           bbC[11] = 1;
  cv::Mat bbTransformedCorners = bbTransform * bbCorners;
  double* bbTC = bbTransformedCorners.ptr<double>();
  double bbTWidth = round(max(max(bbTC[0], bbTC[1]), max(bbTC[2], bbTC[3])) -
      min(min(bbTC[0], bbTC[1]), min(bbTC[2], bbTC[3])));
  double bbTHeight = round(max(max(bbTC[4], bbTC[5]), max(bbTC[6], bbTC[7])) -
      min(min(bbTC[4], bbTC[5]), min(bbTC[6], bbTC[7])));
  double* bbT = bbTransform.ptr<double>();
  bbT[2] = bbT[2] - bbImage.cols/2.0 + bbTWidth/2.0;
  bbT[5] = bbT[5] - bbImage.rows/2.0 + bbTHeight/2.0;

  // Rotate the bounding box and crop out the desired sub-image (via copy!)
  cv::Mat bbRotatedImage;
  cv::warpAffine(bbImage, bbRotatedImage, bbTransform,
      cv::Size(int(bbTWidth), int(bbTHeight)), INTER_NEAREST);
  int bbRTopLeftX = std::max(floor(bbRotatedImage.cols/2.0 - camW/2), 0.0);
  int bbRTopLeftY = std::max(floor(bbRotatedImage.rows/2.0 - camH/2), 0.0);
  int bbRBottomRightX = bbRTopLeftX + std::max(int(floor(camW)), 1);
  if (bbRBottomRightX > bbRotatedImage.cols) { bbRBottomRightX = bbRotatedImage.cols; }
  int bbRBottomRightY = bbRTopLeftY + std::max(int(floor(camH)), 1);
  if (bbRBottomRightY > bbRotatedImage.rows) { bbRBottomRightY = bbRotatedImage.rows; }

#ifdef GET_IMAGE_PRINT_CROP_RANGE
    std::cout << "camImage = bbRotatedImage(" <<
        bbRTopLeftY << ":" << bbRBottomRightY - 1 << ", " <<
        bbRTopLeftX << ":" << bbRBottomRightX - 1 << ")" << std::endl;
#endif

  cv::Mat camImage = bbRotatedImage(
      Range(bbRTopLeftY, bbRBottomRightY),
      Range(bbRTopLeftX, bbRBottomRightX));
  if (camImage.size() != buffer.size()) {
    resize(camImage, buffer, buffer.size(), 0, 0, INTER_LINEAR);
  } else {
    camImage.copyTo(buffer);
  }

#ifdef DEBUG_SINGLE_IMAGE_SERVER
  boost::posix_time::time_duration td = boost::posix_time::microsec_clock::local_time() - tic;

  cv::Mat canvasBuffer;
  _canvas.copyTo(canvasBuffer);
  line(canvasBuffer, Point2f(camTC[0], camTC[4]), Point2f(camTC[1], camTC[5]),
      Scalar(Vec3b(255, 0, 0)), 2, CV_AA);
  line(canvasBuffer, Point2f(camTC[1], camTC[5]), Point2f(camTC[3], camTC[7]),
      Scalar(Vec3b(255, 0, 0)), 2, CV_AA);
  line(canvasBuffer, Point2f(camTC[3], camTC[7]), Point2f(camTC[2], camTC[6]),
      Scalar(Vec3b(255, 0, 0)), 2, CV_AA);
  line(canvasBuffer, Point2f(camTXMin, camTYMin), Point2f(camTXMin, camTYMax),
      Scalar(Vec3b(0, 255, 0)), 2, CV_AA);
  line(canvasBuffer, Point2f(camTXMin, camTYMax), Point2f(camTXMax, camTYMax),
      Scalar(Vec3b(0, 255, 0)), 2, CV_AA);
  line(canvasBuffer, Point2f(camTXMax, camTYMax), Point2f(camTXMax, camTYMin),
      Scalar(Vec3b(0, 255, 0)), 2, CV_AA);
  imshow("canvas", canvasBuffer);
  imshow("bbImage", bbImage);
  imshow("bbRotatedImage", bbRotatedImage);

  std::ostringstream oss;
  oss << "Fetched " << buffer.cols << " x " << buffer.rows <<
     " image @ (" << x << ", " << y <<") & " << upDeg <<
     "' from " << _width << " x " << _height << " canvas in " <<
     td.total_microseconds() / 1000.0 << " ms";
  displayOverlay("canvas", oss.str(), 10000);
#endif

  return;
};
