#include "sightedturtlesim/QImageWidget.hpp"
#include "sightedturtlesim/AbstractImageServer.hpp"
#include <QSizePolicy>
#include <algorithm>
#include <ros/package.h>
#include <QColor>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;


#ifdef QT_USE_OPENGL
QImageWidget::QImageWidget(QWidget* parent) :
    QGLWidget(parent), imageLock(), imageBuffer(), drawBuffer(),
    buffer(NULL), bufferSize(0), bufferScale(1.0), greyscaleIndex(256), textFont(),
    robots_(NULL), pixelsPerMeter(1.0),
    selectedRobotName(""),
    selectionInitMouseXPx(-1), selectionInitMouseYPx(-1),
    selectionInitRobotZ(0), selectionInitRobotAngleRad(0) {
#else
  QImageWidget::QImageWidget(QWidget* parent) :
    QWidget(parent), imageLock(), imageBuffer(), drawBuffer(),
    buffer(NULL), bufferSize(0), bufferScale(1.0), greyscaleIndex(256), textFont(),
    robots_(NULL), pixelsPerMeter(1.0),
    selectedRobotName(""),
    selectionInitMouseXPx(-1), selectionInitMouseYPx(-1),
    selectionInitRobotZ(0), selectionInitRobotAngleRad(0) {
#endif
  // Initialize greyscale color indices
  for (int i = 0; i <= 255; i++) { greyscaleIndex[i] = qRgb(i, i, i); }

  // Load robot icons
#define MAX_NUM_TURTLES 9
  std::string turtles[MAX_NUM_TURTLES] = {
    "box-turtle.png",
    "robot-turtle.png",
    "sea-turtle.png",
    "diamondback.png",
    "electric.png",
    "fuerte.png",
    "groovy.png",
    "indigo.png",
    "jade.png"
  }; // skip hydro.png since image is much larger
  Qt::GlobalColor colors[MAX_NUM_TURTLES] = {
      Qt::green,
      Qt::blue,
      Qt::red,
      Qt::cyan,
      Qt::magenta,
      Qt::yellow,
      Qt::darkGreen,
      Qt::darkBlue,
      Qt::darkRed,
  };

  std::string images_path = ros::package::getPath("turtlesim") + "/images/";
  for (size_t i = 0; i < MAX_NUM_TURTLES; i++) {
    QImage robotImage = QImage(QString::fromStdString(images_path + turtles[i]));
    if (!robotImage.isNull()) {
      robot_images_.push_back(std::make_pair(robotImage, colors[i]));
    }
  }

  setAttribute(Qt::WA_OpaquePaintEvent, false); // Clear area before paintEvent to properly handle resizes
  setAttribute(Qt::WA_PaintOnScreen, false); // Enable double buffering to prevent image flickering

  setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
};


QImageWidget::~QImageWidget() {
  clearBuffer();
};


void QImageWidget::clearBuffer() {
  imageLock.lock();
  if (buffer != NULL) {
    free(buffer);
    buffer = NULL;
  }
  bufferSize = 0;
  imageBuffer = QImage();
  drawBuffer = QImage();
  imageLock.unlock();
};


void QImageWidget::fromROSImage(const sensor_msgs::Image::ConstPtr& rosimg) {
  // Check if image format is convertible to QImage
  std::pair<QImage::Format, bool> imgFormat = encoding2Format(rosimg->encoding);
  if (imgFormat.first == QImage::Format_Invalid) {
    //return false;
    return;
  }

  imageLock.lock();

  // Re-allocate buffer memory if needed
  size_t newBufferSize = rosimg->height * rosimg->step;
  if ((buffer == NULL) || (newBufferSize > bufferSize)) {
    buffer = (uint8_t*) realloc(buffer, newBufferSize); // realloc also works if buffer == NULL
    bufferSize = newBufferSize;
  }

  // Copy ROS image data into local buffer
  std::copy(rosimg->data.begin(), rosimg->data.end(), buffer);

  // Create new QImage from buffer (i.e. create soft wrapper)
  imageBuffer = QImage((uchar*) buffer, rosimg->width, rosimg->height,
      rosimg->step, imgFormat.first);
  if (imgFormat.second) {
    imageBuffer = imageBuffer.rgbSwapped();
  }
#ifdef QT_USE_OPENGL
  drawBuffer = QGLWidget::convertToGLFormat(imageBuffer);
#endif
  imageLock.unlock();

  emit update();

  return;
};


void QImageWidget::fromCVImage(const cv::Mat& _cvimg) {
  // Check if image format is convertible to QImage
  std::pair<QImage::Format, bool> imgFormat = depth2Format(_cvimg.depth(), _cvimg.channels());
  if (imgFormat.first == QImage::Format_Invalid) {
    return;
  }

  // Downscale source image to keep memory footprint low
  cv::Mat cvimg;
  double scale = double(MAX_CANVAS_WIDTH_PX)/std::max(_cvimg.rows, _cvimg.cols);
  if (scale > 0.0 && scale < 1.0) {
    bufferScale = scale;
    cv::resize(_cvimg, cvimg, cv::Size(), bufferScale, bufferScale, cv::INTER_NEAREST);
  } else {
    bufferScale = 1.0;
    cvimg = _cvimg;
  }

  imageLock.lock();

  // Re-allocate buffer memory if needed
  size_t newBufferSize = cvimg.rows * cvimg.step;
  if ((buffer == NULL) || (newBufferSize > bufferSize)) {
    buffer = (uint8_t*) realloc(buffer, newBufferSize); // realloc also works if buffer == NULL
    bufferSize = newBufferSize;
  }

  // Copy CV image data into local buffer
  memcpy(buffer, cvimg.data, newBufferSize);

  // Create new QImage from buffer (i.e. create soft wrapper)
  imageBuffer = QImage((uchar*) buffer, cvimg.cols, cvimg.rows,
      cvimg.step, imgFormat.first);
  if (imgFormat.second) {
    imageBuffer = imageBuffer.rgbSwapped();
  }
#ifdef QT_USE_OPENGL
  drawBuffer = QGLWidget::convertToGLFormat(imageBuffer);
#endif
  imageLock.unlock();

  emit update();

  return;
};


QSize QImageWidget::getScaledImageSize() {
  QSize result = size();
  if (buffer != NULL && imageBuffer.width() > 0 && imageBuffer.height() > 0) {
    if (result.width() < result.height()) {
      result.setHeight(result.width()*imageBuffer.height()/imageBuffer.width());
    } else {
      result.setWidth(result.height()*imageBuffer.width()/imageBuffer.height());
    }
  }
  return result;
};


#ifdef QT_USE_OPENGL
void QImageWidget::initializeGL() {
  glClearColor(0.0, 0.0, 0.0, 1.0);
};


void QImageWidget::resizeGL(int width, int height) {
  glViewport(0, 0, (GLint) width, (GLint) height);
};


void QImageWidget::paintGL() {
  qglClearColor(Qt::lightGray);
  glClear(GL_COLOR_BUFFER_BIT);
  if (buffer != NULL && imageLock.timed_lock(boost::posix_time::milliseconds(50))) {
    QSize imageSize = getScaledImageSize();
    // NOTE: we COULD use glDrawPixels directly, but it will be slow
    // glDrawPixels(image.width(), image.height(), GL_RGBA, GL_UNSIGNED_BYTE, image.bits());
    // alternatively, we will perform 2D texture mapping
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, width(), height(), 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnable(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D( GL_TEXTURE_2D, 0, 4, drawBuffer.width(), drawBuffer.height(),
        0, GL_RGBA, GL_UNSIGNED_BYTE, drawBuffer.bits());
    glBegin(GL_QUADS);
    glTexCoord2f(0,0); glVertex2f(0, imageSize.height());
    glTexCoord2f(0,1); glVertex2f(0, 0);
    glTexCoord2f(1,1); glVertex2f(imageSize.width(), 0);
    glTexCoord2f(1,0); glVertex2f(imageSize.width(), imageSize.height());
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glFlush();
    imageLock.unlock();
  } // else print No Image, but that is actually quite complex in OpenGL
};
#else
void QImageWidget::paintEvent(QPaintEvent* event) {
  QPainter painter(this);

  // Attempt to obtain ownership of image (using timed lock of 0.05 sec)
  // before painting it on screen
  if (buffer != NULL) {
    if (imageLock.timed_lock(boost::posix_time::milliseconds(50))) {
      drawBuffer = imageBuffer.scaled(this->size(), Qt::KeepAspectRatio,
          Qt::FastTransformation);
      painter.drawImage(0, 0, drawBuffer);
      double imageScale = (double) drawBuffer.width() / imageBuffer.width() * bufferScale;
      if (robots_ != NULL && robots_->size() > 0 && robot_images_.size() > 0) {
        double xPx, yPx, zPx;
        M_Turtle::iterator itRobots = robots_->begin();
        M_Turtle::iterator itRobotsEnd = robots_->end();
        for (; itRobots != itRobotsEnd; itRobots++) {
          VisionTurtle* robot = (VisionTurtle*) itRobots->second;
          xPx = robot->x() * pixelsPerMeter * imageScale;
          yPx = robot->y() * pixelsPerMeter * imageScale;
          zPx = robot->z() * pixelsPerMeter * imageScale;
          std::pair<QImage, enum Qt::GlobalColor>& currImagePair =
              robot_images_[robot->type() % robot_images_.size()];
          QImage robotImage = currImagePair.first.transformed(
              QMatrix().rotate(-robot->angleDeg() + 90.0), Qt::FastTransformation);
          painter.drawImage(round(xPx - robotImage.width()/2.0),
              round(yPx - robotImage.height()/2.0), robotImage);
          AbstractImageServer::toCornersXY(xPx, yPx,
              -robot->angleDeg() + 90.0, zPx, robot->getHFOVDeg(),
              robot->getAspectRatio(), cornersBuffer);
          painter.setPen(QPen(currImagePair.second,
              std::max(1, (int) (std::min(this->width(), this->height()) / 250.0)),
              Qt::DashLine));
          painter.drawLine(round(cornersBuffer[0]), round(cornersBuffer[1]),
              round(cornersBuffer[2]), round(cornersBuffer[3]));
          painter.drawLine(round(cornersBuffer[2]), round(cornersBuffer[3]),
              round(cornersBuffer[4]), round(cornersBuffer[5]));
          painter.drawLine(round(cornersBuffer[4]), round(cornersBuffer[5]),
              round(cornersBuffer[6]), round(cornersBuffer[7]));
          painter.drawLine(round(cornersBuffer[6]), round(cornersBuffer[7]),
              round(cornersBuffer[0]), round(cornersBuffer[1]));
        }


        M_Turtle::iterator it;
        if (selectedRobotName.length() > 0 && ((it = robots_->find(selectedRobotName)) != robots_->end())) {
          Turtle* selectedRobot = it->second;
          xPx = selectedRobot->x() * pixelsPerMeter * imageScale;
          yPx = selectedRobot->y() * pixelsPerMeter * imageScale;
          QPen highlightPen;
          highlightPen.setColor(Qt::yellow);
          highlightPen.setWidth(SELECTION_DISTANCE_THRESH_PX/10);
          highlightPen.setStyle(Qt::DotLine);
          painter.setPen(highlightPen);
          painter.drawEllipse(QPoint(xPx, yPx), int(SELECTION_DISTANCE_THRESH_PX), int(SELECTION_DISTANCE_THRESH_PX));
        }
      }
      imageLock.unlock();
    }
  } else { // Draw default (empty) scene
    QRect region = QRect(0, 0, width() - 1, height() - 1);
    painter.setPen(Qt::black);
    painter.setBrush(Qt::lightGray);
    painter.drawRect(region);
    painter.setPen(Qt::black);
    painter.setFont(textFont);
    painter.drawText(region, Qt::AlignCenter, tr("No Image"));
  }
  painter.end();
};
#endif


std::pair<QImage::Format, bool> QImageWidget::encoding2Format(const std::string& encoding) {
  if (encoding == "8UC1") return std::make_pair(QImage::Format_Indexed8, false);
  if (encoding == "8UC3") return std::make_pair(QImage::Format_RGB888, true);
  if (encoding == "8UC4") return std::make_pair(QImage::Format_ARGB32, true);
  if (encoding == "8SC1") return std::make_pair(QImage::Format_Indexed8, false);
  if (encoding == "8SC3") return std::make_pair(QImage::Format_RGB888, true);
  if (encoding == "8SC4") return std::make_pair(QImage::Format_ARGB32, true);
  if (encoding == "rgb8") return std::make_pair(QImage::Format_RGB888, false);
  if (encoding == "bgr8") return std::make_pair(QImage::Format_RGB888, true);
  if (encoding == "rgba8") return std::make_pair(QImage::Format_ARGB32, false);
  if (encoding == "bgra8") return std::make_pair(QImage::Format_ARGB32, true);
  if (encoding == "mono8") return std::make_pair(QImage::Format_Indexed8, false);
  return std::make_pair(QImage::Format_Invalid, false);
};


std::pair<QImage::Format, bool> QImageWidget::depth2Format(const int depth,
    const int channels) {
  if (channels == 1) {
    if (depth == CV_8UC1) return std::make_pair(QImage::Format_Indexed8, false);
    if (depth == CV_8UC3) return std::make_pair(QImage::Format_RGB888, false);
    if (depth == CV_8UC4) return std::make_pair(QImage::Format_ARGB32, false);
    if (depth == CV_8SC1) return std::make_pair(QImage::Format_Indexed8, false);
    if (depth == CV_8SC3) return std::make_pair(QImage::Format_RGB888, false);
    if (depth == CV_8SC4) return std::make_pair(QImage::Format_ARGB32, false);
  } else if (channels == 3) {
    if (depth == CV_8UC1) return std::make_pair(QImage::Format_RGB888, true);
    if (depth == CV_8SC1) return std::make_pair(QImage::Format_RGB888, true);
  } else if (channels == 4) {
    if (depth == CV_8UC1) return std::make_pair(QImage::Format_ARGB32, true);
    if (depth == CV_8SC1) return std::make_pair(QImage::Format_ARGB32, true);
  }
  return std::make_pair(QImage::Format_Invalid, false);
};


void QImageWidget::mousePressEvent(QMouseEvent* event) {
  // Attempt to locate and select robot
  if (event->button() == Qt::LeftButton || event->button() == Qt::RightButton) {
    if (buffer != NULL && robots_ != NULL && robots_->size() > 0) {
      if (imageLock.timed_lock(boost::posix_time::milliseconds(1))) {
        if (robots_ != NULL && robots_->size() > 0) { // Re-confirm robot presence after lock
          double imageScale = (double) drawBuffer.width() / imageBuffer.width() * bufferScale;
          double xPx, yPx, distSqrd;
          std::string closestRobotName = "";
          double closestDistSqrd = std::numeric_limits<double>::infinity();

          M_Turtle::iterator itRobots = robots_->begin();
          M_Turtle::iterator itRobotsEnd = robots_->end();
          for (; itRobots != itRobotsEnd; itRobots++) {
            Turtle* robot = itRobots->second;
            xPx = robot->x() * pixelsPerMeter * imageScale;
            yPx = robot->y() * pixelsPerMeter * imageScale;
            distSqrd = pow(event->x() - xPx, 2) + pow(event->y() - yPx, 2);

            if (distSqrd < closestDistSqrd) {
              closestDistSqrd = distSqrd;
              closestRobotName = itRobots->first;
            }
          }

          if (closestRobotName.length() > 0 && closestDistSqrd < SELECTION_DISTANCE_THRESH_PX*SELECTION_DISTANCE_THRESH_PX) {
            selectedRobotName = closestRobotName;
            selectionInitMouseXPx = event->x();
            selectionInitMouseYPx = event->y();
            selectionInitRobotZ = robots_->at(selectedRobotName)->z();
            selectionInitRobotAngleRad = robots_->at(selectedRobotName)->angleRad();
          } else {
            selectedRobotName = "";
            selectionInitMouseXPx = -1;
            selectionInitMouseYPx = -1;
            selectionInitRobotZ = 0;
            selectionInitRobotAngleRad = 0;
          }
        }
        imageLock.unlock();
      }
    }
  }
};


void QImageWidget::mouseMoveEvent(QMouseEvent* event) {
  if (selectedRobotName.length() > 0 && (event->buttons() & (Qt::LeftButton | Qt::RightButton))) {
    if (imageLock.timed_lock(boost::posix_time::milliseconds(1))) {
      M_Turtle::iterator it;
      if (robots_ != NULL && ((it = robots_->find(selectedRobotName)) != robots_->end())) {
        Turtle* robot = it->second;
        if (event->buttons() & Qt::LeftButton) { // NOTE: implicit preference of left button over right button
          // Left button action: move robot's X/Y
          double imageScale = (double) drawBuffer.width() / imageBuffer.width() * bufferScale;

          double mouseXPx = event->x();
          double mouseYPx = event->y();
          if (mouseXPx < 0) { mouseXPx = 0; }
          else if (mouseXPx >= drawBuffer.width()) { mouseXPx = drawBuffer.width() - 1; }
          if (mouseYPx < 0) { mouseYPx = 0; }
          else if (mouseYPx >= drawBuffer.height()) { mouseYPx = drawBuffer.height() - 1; }

          double newRobotXPos = mouseXPx / pixelsPerMeter / imageScale;
          double newRobotYPos = mouseYPx / pixelsPerMeter / imageScale;

          robot->setPose(newRobotXPos, newRobotYPos, robot->z(), robot->angleRad());
        } else if (event->buttons() & Qt::RightButton) {
          // Right button action: rotate robot via x mouse pos, change robot's height via y mouse pos
          double mouseDX = event->x() - selectionInitMouseXPx;
          double mouseDY = event->y() - selectionInitMouseYPx;
          double heightPercent = min(max(-mouseDY * DRAG_PX_TO_HEIGHT_PCT_RATIO, -99.0), 100.0) + 100.0; // -99% to prevent setting robot height = 0
          double angleChange = mouseDX * DRAG_PX_TO_ANGLE_RATIO;

          robot->setPose(robot->x(), robot->y(),
              selectionInitRobotZ * heightPercent / 100.0,
              selectionInitRobotAngleRad + angleChange);
        }
      } else {
        selectedRobotName = "";
      }

      imageLock.unlock();
    }
  }
};


void QImageWidget::mouseReleaseEvent(QMouseEvent* event) {
  imageLock.lock();
  selectedRobotName = "";
  selectionInitMouseXPx = -1;
  selectionInitMouseYPx = -1;
  selectionInitRobotZ = 0;
  selectionInitRobotAngleRad = 0;
  imageLock.unlock();
};
