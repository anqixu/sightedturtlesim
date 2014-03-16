#ifndef QIMAGEWIDGET_HPP_
#define QIMAGEWIDGET_HPP_


//#define QT_USE_OPENGL


#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <QImage>
#include <QPainter>
#include <QFont>
#include <QTimer>
#include <QMouseEvent>
#include <boost/thread/mutex.hpp>
#include <cstdlib>
#include <string>
#include <utility>
#include "sightedturtlesim/TurtleFrame.hpp"


// NOTE: all functions should be thread-safe (handled internally)
#ifdef QT_USE_OPENGL
#include <QGLWidget>
class QImageWidget : public QGLWidget {
#else
#include <QWidget>
class QImageWidget : public QWidget {
#endif
Q_OBJECT;

public:
  QImageWidget(QWidget* parent = NULL);

  virtual ~QImageWidget();

  void setRobotPtr(M_Turtle* robots) {
    imageLock.lock();
    robots_ = robots;
    imageLock.unlock();
  };

  void setPixelsPerMeter(double newPPM) { if (newPPM > 0) { pixelsPerMeter = newPPM; } };

  QSize getScaledImageSize();

  void resize(double scale) { QWidget::resize(scale * imageBuffer.size()); };

  QSize sizeHint() const { return QSize(MIN_IMAGE_WIDTH_PX, MIN_IMAGE_HEIGHT_PX); };

  static std::pair<QImage::Format, bool> encoding2Format(const std::string& encoding);

  static std::pair<QImage::Format, bool> depth2Format(const int depth, const int channels);


public slots:
  void fromROSImage(const sensor_msgs::Image::ConstPtr& rosimg);

  void fromCVImage(const cv::Mat& cvimg);

  bool autoRefresh(bool request) {
    bool result = false;
    if (request && robots_ != NULL) {
      drawTimer.start(100);
      connect(&drawTimer, SIGNAL(timeout()), this, SLOT(update()));
      result = true;
    } else {
      drawTimer.stop();
      disconnect(&drawTimer, SIGNAL(timeout()), this, 0);
    }

    return result;
  };

  void clearBuffer();


protected:
#ifdef QT_USE_OPENGL
  void initializeGL();

  void resizeGL(int width, int height);

  virtual void paintGL();
#else
  virtual void paintEvent(QPaintEvent* event);
#endif

  virtual void mousePressEvent(QMouseEvent* event);

  virtual void mouseMoveEvent(QMouseEvent* event);

  virtual void mouseReleaseEvent(QMouseEvent* event);


  // WARNING: Always use lock before accessing image or buffer
  boost::timed_mutex imageLock;
  QImage imageBuffer, drawBuffer;
  uint8_t* buffer;
  size_t bufferSize; // BIGGER or equals to (image's stride * image's height)

  QVector<QRgb> greyscaleIndex;
  QFont textFont;

  std::vector< std::pair<QImage, enum Qt::GlobalColor> > robot_images_;
  M_Turtle* robots_;

  QTimer drawTimer;

  double cornersBuffer[8];
  double pixelsPerMeter;

  std::string selectedRobotName;
  double selectionInitMouseXPx;
  double selectionInitMouseYPx;
  double selectionInitRobotZ;
  double selectionInitRobotAngleRad;

  constexpr static double SELECTION_DISTANCE_THRESH_PX = 20;
  constexpr static double DRAG_PX_TO_HEIGHT_PCT_RATIO = 1.0/6;
  constexpr static double DRAG_PX_TO_ANGLE_RATIO = M_PI/200;

  constexpr static unsigned int MIN_IMAGE_WIDTH_PX = 160;
  constexpr static unsigned int MIN_IMAGE_HEIGHT_PX = 120;
};


#endif /* QIMAGEWIDGET_HPP_ */
