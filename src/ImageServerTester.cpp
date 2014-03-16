#include "sightedturtlesim/SingleImageServer.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sstream>


using namespace cv;


int main(int argc, char** argv) {
  SingleImageServer server("/home/mimic/Desktop/galaxy.jpg");
  cv::Mat buffer;

#ifdef DEBUG_SINGLE_IMAGE_SERVER
  namedWindow("canvas", CV_WINDOW_NORMAL);
  namedWindow("bbImage", CV_WINDOW_NORMAL);
  namedWindow("bbRotatedImage", CV_WINDOW_NORMAL);
#endif
  namedWindow("image", CV_WINDOW_NORMAL);

  double x = server.width()/2;
  double y = server.height()/2;
  double deltaX = max(server.width()/20.0, 2.0);
  double deltaY = max(server.height()/20.0, 2.0);
  double upDeg = 0;
  double deltaDeg = 90.0/16;
  unsigned int imWidth = 320;
  unsigned int imHeight = 240;

  bool redraw = true;

  while (1) {
    if (redraw) {
      buffer.create(imHeight, imWidth, CV_8UC3);

      boost::posix_time::ptime tic = boost::posix_time::microsec_clock::local_time();

      server.getImage(x, y, upDeg, buffer);

      boost::posix_time::time_duration td = boost::posix_time::microsec_clock::local_time() - tic;

      imshow("image", buffer);

      std::ostringstream oss;
      oss << "Fetched " << buffer.cols << " x " << buffer.rows << \
         " image @ (" << x << ", " << y <<") & " << upDeg << \
         "' from " << server.width() << " x " << server.height() << \
         " canvas in " << td.total_microseconds() / 1000.0 << " ms";
      displayOverlay("image", oss.str(), 10000);

      redraw = false;
    }

    char key = waitKey(33);
    if (key == 'w') { // Move up
      y -= deltaY;
      redraw = true;
    } else if (key == 's') { // Move down
      y += deltaY;
      redraw = true;
    } else if (key == 'a') { // Move left
      x -= deltaX;
      redraw = true;
    } else if (key == 'd') { // Move right
      x += deltaX;
      redraw = true;
    } else if (key == 'u') { // Increase image scale
      imWidth *= 1.25;
      imHeight *= 1.25;
      redraw = true;
    } else if (key == 'j') { // Decrease image scale
      imWidth *= 0.80;
      imHeight *= 0.80;
      redraw = true;
    } else if (key == 'h') { // Rotate left
      upDeg -= deltaDeg;
      redraw = true;
    } else if (key == 'k') { // Rotate right
      upDeg += deltaDeg;
      redraw = true;
    } else if (key == 'x') {
      break;
    }
  }

  return 0;
};
