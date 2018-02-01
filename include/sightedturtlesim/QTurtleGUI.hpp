#ifndef QTURTLEGUI_HPP_
#define QTURTLEGUI_HPP_


#include <QMainWindow>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <sightedturtlesim/MapSettings.h>
#include <sightedturtlesim/LoadSingleImageMap.h>
#include <sightedturtlesim/Spawn.h>
#include <sightedturtlesim/Kill.h>
#include <opencv2/core/core.hpp>
#include <utility>


class QAction;
class QMenu;
class QScrollArea;
class QScrollBar;
class AbstractImageServer;
class TurtleFrame;
class QImageWidget;
class QSocketNotifier;


class QTurtleGUI : public QMainWindow {
  Q_OBJECT;

public:
  QTurtleGUI();
  ~QTurtleGUI();

  void spin();

  static void sigintHandler(int unused) {
    char a = 1;
    ::write(sigintFd[0], &a, sizeof(a));
  };

public slots:
  void open();
  void spawn();
  void kill();
  void zoomIn();
  void zoomOut();
  void normalSize();
  void fitToWindow();
  void toggleUpdates();
  void autoRefresh();
  void handleSigint();

  bool loadSingleImageMap(QString filename, double ppm);
  std::string spawnTurtle(double x, double y, double z, double angle,
      double hfovDeg, double aspectRatio,
      unsigned int imWidth, unsigned int imHeight, double imFPS, double scale);
  bool killTurtle(const std::string& name);

signals:
  void requestLoadSingleImageMap(QString filename, double ppm);
  void requestSpawnTurtle(double x, double y, double z, double angle,
    double hfovDeg, double aspectRatio,
    unsigned int imWidth, unsigned int imHeight, double imFPS, double scale);
  void requestKillTurtle(const std::string& name);

protected:
  void createActions();
  void createMenus();
  void updateActions();
  void scaleImage(double factor);
  void adjustScrollBar(QScrollBar *scrollBar, double factor);

  bool loadSingleImageMapCB(sightedturtlesim::LoadSingleImageMap::Request& req,
      sightedturtlesim::LoadSingleImageMap::Response& res);
  bool spawnCB(sightedturtlesim::Spawn::Request& req,
      sightedturtlesim::Spawn::Response& res);
  bool killCB(sightedturtlesim::Kill::Request& req,
      sightedturtlesim::Kill::Response& res);

  QMenu* fileMenu;
    QAction* openAct;
    QAction* spawnAct;
    QAction* killAct;
    QAction *closeAct;
  QMenu* viewMenu;
    QAction* zoomInAct;
    QAction* zoomOutAct;
    QAction* normalSizeAct;
    QAction* fitToWindowAct;
    QAction* toggleUpdatesAct;
    QAction* autoRefreshAct;

  QImageWidget* imageWidget;
  QScrollArea* scrollArea;
  double scaleFactor;

  ros::NodeHandle node;
  ros::NodeHandle localNode;
  ros::Publisher mapSettingsPub;
  ros::ServiceServer loadSingleImageMapSrv;
  ros::ServiceServer spawnSrv;
  ros::ServiceServer killSrv;

  AbstractImageServer* imageServer;
  sightedturtlesim::MapSettings mapSettings;
  TurtleFrame* robots;
  boost::mutex robotsMutex;

  boost::thread spinThread;
  double spinRateHz;

  static int sigintFd[2];
  QSocketNotifier* snSigint;

  std::list< std::pair< std::string, cv::Mat > > cachedImageMaps;
};


#endif /* QTURTLEGUI_HPP_ */
