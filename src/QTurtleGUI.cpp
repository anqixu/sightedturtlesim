#include "sightedturtlesim/QTurtleGUI.hpp"
#include "sightedturtlesim/QImageWidget.hpp"
#include "sightedturtlesim/QSpawnRobotDialog.hpp"
#include "sightedturtlesim/SingleImageServer.hpp"
#include "sightedturtlesim/TurtleFrame.hpp"
#include "sightedturtlesim/PoseXYZ.h"
#include <QtGui>
#include <QAction>
#include <QScrollBar>
#include <QScrollArea>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QInputDialog>
#include <QFileDialog>
#include <QStringList>
#include <QMessageBox>
#include <QSocketNotifier>
#include <ros/package.h>
#include <sys/socket.h>

#include <opencv2/highgui/highgui.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>


int QTurtleGUI::sigintFd[2];


QTurtleGUI::QTurtleGUI() : QMainWindow(),
    node(), local_node("~"), imageServer(NULL), robots(NULL), robotsMutex(), spinRateHz(100) {
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigintFd)) {
    qFatal("Could not create SIGINT socketpair for Qt");
  }
  snSigint = new QSocketNotifier(sigintFd[1], QSocketNotifier::Read, this);
  connect(snSigint, SIGNAL(activated(int)), this, SLOT(handleSigint()));

  loadSingleImageMapSrv = local_node.advertiseService("load_single_image_map",
      &QTurtleGUI::loadSingleImageMapCB, this);
  spawnSrv = local_node.advertiseService("spawn", &QTurtleGUI::spawnCB, this);
  killSrv = local_node.advertiseService("kill", &QTurtleGUI::killCB, this);

  scrollArea = new QScrollArea(this);
  scrollArea->setBackgroundRole(QPalette::Dark);
  imageWidget = new QImageWidget(scrollArea);
  scrollArea->setWidget(imageWidget);
  setCentralWidget(scrollArea);

  createActions();
  createMenus();

  qRegisterMetaType<std::string>("std::string");
  connect(this, SIGNAL(requestLoadSingleImageMap(QString, double)),
      this, SLOT(loadSingleImageMap(QString, double)), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(requestSpawnTurtle(double, double, double, double, unsigned int, unsigned int, double, double)),
      this, SLOT(spawnTurtle(double, double, double, double, unsigned int, unsigned int, double, double)), Qt::BlockingQueuedConnection);
  connect(this, SIGNAL(requestKillTurtle(const std::string&)),
      this, SLOT(killTurtle(const std::string&)), Qt::BlockingQueuedConnection);

  setWindowTitle(tr("Sighted Turtle Simulator"));
  resize(400, 300);

  // Start spin thread
  local_node.param<double>("spin_rate_hz", spinRateHz, 100.0);
  if (spinRateHz < 50.) { spinRateHz = 50; }
  spinThread = boost::thread(boost::bind(&QTurtleGUI::spin, this));

  // Load cached images
  std::string fnameList = "";
  local_node.param<std::string>("cached_image_fnames", fnameList, fnameList);
  if (!fnameList.empty()) {
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(";");
    tokenizer tok(fnameList, sep);
    std::string fname;
    try {
      for (tokenizer::iterator it = tok.begin(); it != tok.end(); it++) {
        fname = *it;
        boost::algorithm::trim(fname);
        ROS_INFO_STREAM("Caching simulator map " << fname << "...");
        cv::Mat canvas = cv::imread(fname, 1);
        if (canvas.empty()) {
          ROS_ERROR_STREAM("Unable to read image: " << fname);
          continue;
        }
        cachedImageMaps.push_back(std::make_pair(fname, canvas));
      }
    } catch (const boost::bad_lexical_cast& ex) {
      ROS_ERROR_STREAM("Could not parse cached_image_fnames: " << ex.what());
    }
    ROS_INFO_STREAM("Cached " << cachedImageMaps.size() << " images");
  }

  // Process parameters for fast initialization
  std::string INIT_LoadSingleImageMap_path;
  double INIT_LoadSingleImageMap_pixelPerMeter;
  if (local_node.getParam("INIT_LoadSingleImageMap_path", INIT_LoadSingleImageMap_path) &&
      local_node.getParam("INIT_LoadSingleImageMap_pixelPerMeter", INIT_LoadSingleImageMap_pixelPerMeter)) {
    loadSingleImageMap(QString::fromStdString(INIT_LoadSingleImageMap_path),
        INIT_LoadSingleImageMap_pixelPerMeter); // WARNING: do not emit signal since connection is blocking, so will cause deadlock

    double INIT_Spawn_x, INIT_Spawn_y, INIT_Spawn_z, INIT_Spawn_theta, INIT_Spawn_scale, INIT_Spawn_imageFPS;
    int INIT_Spawn_imageWidth, INIT_Spawn_imageHeight;
    if (local_node.getParam("INIT_Spawn_x", INIT_Spawn_x) &&
        local_node.getParam("INIT_Spawn_y", INIT_Spawn_y) &&
        local_node.getParam("INIT_Spawn_z", INIT_Spawn_z) &&
        local_node.getParam("INIT_Spawn_theta", INIT_Spawn_theta) &&
        local_node.getParam("INIT_Spawn_scale", INIT_Spawn_scale) &&
        local_node.getParam("INIT_Spawn_imageWidth", INIT_Spawn_imageWidth) &&
        local_node.getParam("INIT_Spawn_imageHeight", INIT_Spawn_imageHeight) &&
        local_node.getParam("INIT_Spawn_imageFPS", INIT_Spawn_imageFPS)) {
      spawnTurtle(INIT_Spawn_x, INIT_Spawn_y, INIT_Spawn_z,
          INIT_Spawn_theta, INIT_Spawn_imageWidth, INIT_Spawn_imageHeight,
          INIT_Spawn_imageFPS, INIT_Spawn_scale); // WARNING: do not emit signal since connection is blocking, so will cause deadlock
    }
  }
};


QTurtleGUI::~QTurtleGUI() {
  loadSingleImageMapSrv.shutdown();
  spawnSrv.shutdown();
  killSrv.shutdown();

  imageWidget->setRobotPtr(NULL);

  robotsMutex.lock();
  if (robots != NULL) {
    delete robots;
    robots = NULL;
  }
  robotsMutex.unlock();

  if (imageServer != NULL) {
    delete imageServer;
    imageServer = NULL;
  }

  spinThread.interrupt();
  spinThread.timed_join(boost::posix_time::milliseconds(100));
};


void QTurtleGUI::spin() {
  ros::Rate sleepRate(spinRateHz);
  try {
    while (ros::ok()) {
      robotsMutex.lock();
      if (robots != NULL) {
        robots->updateTurtles();
      }
      robotsMutex.unlock();
      ros::spinOnce();
      sleepRate.sleep();
      boost::this_thread::interruption_point();
    }
  } catch (const boost::thread_interrupted& err) {
  }
};


void QTurtleGUI::handleSigint() {
  close();
};


void QTurtleGUI::open() {
  QString fileName = QFileDialog::getOpenFileName(this,
      tr("Open File"), QDir::currentPath());
  if (!fileName.isEmpty()) {
    QFile file(fileName);
    if (!file.exists()) {
      statusBar()->showMessage(tr("Could not load image %1.").arg(fileName));
      return;
    }

    // Query pixelsPerMeter
    bool ok = false;
    double ppm = QInputDialog::getDouble(this, tr("Specify Image Scale"),
        tr("Please specify image scale, in pixels per meters: "),
        1.0, 0.0, 10000.0, 4, &ok);
    if (!ok || ppm <= 0.0) {
      return;
    }

    loadSingleImageMap(fileName, ppm);
  }
};


bool QTurtleGUI::loadSingleImageMapCB(
    sightedturtlesim::LoadSingleImageMap::Request& req,
    sightedturtlesim::LoadSingleImageMap::Response& res) {
  emit requestLoadSingleImageMap(QString::fromStdString(req.path), req.pixelPerMeter);
  //loadSingleImageMap(QString::fromStdString(req.path), req.pixelPerMeter); // WARNING: running Qt fns on spin thread can cause segfaults!
  return true;
};


bool QTurtleGUI::loadSingleImageMap(QString filename, double ppm) {
  std::string filenameString = filename.toStdString();
  ROS_INFO_STREAM("Received request to load new map: " << filenameString << " @ ppm=" << ppm);

  QFile file(filename);
  if (!file.exists()) {
    statusBar()->showMessage(tr("Could not load image %1.").arg(filename));
    ROS_ERROR_STREAM("Could not load image: " << filenameString);
    return false;
  }

  // Do not reset image server if new map / ppm is same as previous
  if (imageServer != NULL &&
      ((SingleImageServer*) imageServer)->getImageFilename() == filenameString &&
      imageServer->pixelsPerMeter() == ppm) {
    ROS_INFO_STREAM("Not re-loading same map: " << filenameString << " @ " <<
        ppm << " pixels per meter resolution.");
    return true;
  }

#ifdef RECREATE_TURTLES_IT_ADVERTISE_TAKES_LONG
  // Clear previous image server
  std::vector<VisionTurtleState> existingTurtles;
  if (imageServer != NULL) {
    imageWidget->setRobotPtr(NULL);
    robotsMutex.lock();
    for (std::pair<const std::string, Turtle*>& tp: robots->getTurtles()) {
      existingTurtles.push_back(VisionTurtleState((VisionTurtle*) tp.second));
    }
    delete robots;
    robots = NULL;
    robotsMutex.unlock();
    delete imageServer;
  }

  // Create new image server
  try {
    bool hasCachedImage = false;
    for (const std::pair<std::string, cv::Mat>& cache: cachedImageMaps) {
      if (cache.first == filenameString) {
        hasCachedImage = true;
        imageServer = new SingleImageServer(cache.second, filenameString, ppm);
        imageWidget->setPixelsPerMeter(ppm);
      }
    }
    if (!hasCachedImage) {
      imageServer = new SingleImageServer(filenameString, ppm);
      imageWidget->setPixelsPerMeter(ppm);
    }
  } catch (const std::string& err) {
    statusBar()->showMessage(QString::fromStdString(err));
    ROS_ERROR_STREAM("Failed to create new image server: " << err);
    return false;
  }

  // Hook up robots and images
  robotsMutex.lock();
  if (robots == NULL) robots = new TurtleFrame(node, imageServer);
  robotsMutex.unlock();
  imageWidget->fromCVImage(imageServer->canvas());
  imageWidget->setRobotPtr(&robots->getTurtles());
  for (VisionTurtleState& t: existingTurtles) {
    spawnTurtle(t.x, t.y, t.z, t.orientRad, t.imWidth, t.imHeight, t.fps, t.s);
  }
#else
  if (imageServer != NULL) {
    imageWidget->setRobotPtr(NULL);
    robotsMutex.lock();
    for (std::pair<const std::string, Turtle*>& tp: robots->getTurtles()) {
      ((VisionTurtle*) tp.second)->stopThread();
    }
    robots->updateImageServer(NULL);
    robotsMutex.unlock();
    delete imageServer;
  }

  // Create new image server
  try {
    bool hasCachedImage = false;
    for (const std::pair<std::string, cv::Mat>& cache: cachedImageMaps) {
      if (cache.first == filenameString) {
        hasCachedImage = true;
        imageServer = new SingleImageServer(cache.second, filenameString, ppm);
        imageWidget->setPixelsPerMeter(ppm);
      }
    }
    if (!hasCachedImage) {
      imageServer = new SingleImageServer(filenameString, ppm);
      imageWidget->setPixelsPerMeter(ppm);
    }
  } catch (const std::string& err) {
    statusBar()->showMessage(QString::fromStdString(err));
    ROS_ERROR_STREAM("Failed to create new image server: " << err);
    return false;
  }

  // Hook up robots and images
  robotsMutex.lock();
  if (robots == NULL) robots = new TurtleFrame(node, imageServer);
  robots->updateImageServer(imageServer);
  robotsMutex.unlock();
  imageWidget->fromCVImage(imageServer->canvas());
  imageWidget->setRobotPtr(&robots->getTurtles());
  for (std::pair<const std::string, Turtle*>& tp: robots->getTurtles()) {
    ((VisionTurtle*) tp.second)->restartThread(imageServer);
  }
#endif

  // Configure GUI parameters
  scaleFactor = 1.0;
  fitToWindowAct->setEnabled(true);
  autoRefreshAct->setEnabled(true);
  spawnAct->setEnabled(true);
  killAct->setEnabled(true);
  fitToWindowAct->setChecked(true);
  fitToWindow();
  autoRefreshAct->setChecked(true);
  autoRefresh();

  ROS_INFO_STREAM("New map loaded: " << filenameString << " @ ppm=" << ppm);

  return true;
};


void QTurtleGUI::spawn() {
  if (imageServer == NULL || robots == NULL) {
    statusBar()->showMessage(tr("Please load image server before spawning robots."));
    return;
  }

  QSpawnRobotDialog dialog(this, imageServer->getMidX(), imageServer->getMidY());
  if (dialog.exec() == QDialog::Accepted) {
    spawnTurtle(dialog.getX(), dialog.getY(), dialog.getZ(), dialog.getAngle(),
        dialog.getWidth(), dialog.getHeight(), dialog.getFPS(), dialog.getScale());
  }
};


bool QTurtleGUI::spawnCB(sightedturtlesim::Spawn::Request& req,
    sightedturtlesim::Spawn::Response& res) {
  emit requestSpawnTurtle(req.x, req.y, req.z, req.theta * 180.0 / M_PI, req.imageWidth, req.imageHeight, req.imageFPS, req.scale);
  //spawnTurtle(req.x, req.y, req.z, req.theta * 180.0 / M_PI, req.imageWidth, req.imageHeight, req.imageFPS, req.scale); // WARNING: running Qt fns on spin thread can cause segfaults!
  return true;
};


std::string QTurtleGUI::spawnTurtle(double x, double y, double z, double angle,
    unsigned int imWidth, unsigned int imHeight, double imFPS, double scale) {
  if (imageServer == NULL || robots == NULL) {
    statusBar()->showMessage(tr("Please load image server before spawning robots."));
    return "";
  }

  bool fpsCapped = false;
  if (imFPS < 0.01) { imFPS = 0.01; fpsCapped = true; }
  if (imFPS > spinRateHz) { imFPS = spinRateHz; fpsCapped = true; }

  robotsMutex.lock();

  std::string robotName = robots->spawnVisionTurtle(x, y, z, angle,
      imWidth, imHeight, imFPS, scale);
  killAct->setEnabled(true);
  if (fpsCapped) {
    statusBar()->showMessage(tr("Robot %1 spawned (FPS capped @ %2 Hz).").arg(QString::fromStdString(robotName)).arg(imFPS));
  } else {
    statusBar()->showMessage(tr("Robot %1 spawned.").arg(QString::fromStdString(robotName)));
  }

  robotsMutex.unlock();

  return robotName;
};


void QTurtleGUI::kill() {
  robotsMutex.lock();

  QStringList robotIDs;
  M_Turtle::iterator itRobots = robots->getTurtles().begin();
  M_Turtle::iterator itRobotsEnd = robots->getTurtles().end();
  for (; itRobots != itRobotsEnd; itRobots++) {
    robotIDs.push_back(QString::fromStdString(itRobots->first));
  }
  robotsMutex.unlock();

  if (robotIDs.size() <= 0) {
    statusBar()->showMessage(tr("No robot available."));
    return;
  }

  bool ok = false;
  QString selectedRobot = QInputDialog::getItem(this, tr("Kill Robot"),
      tr("Select robot to be killed:"), robotIDs, 0, false, &ok);
  if (ok) {
    killTurtle(selectedRobot.toStdString());
  }
};


bool QTurtleGUI::killCB(sightedturtlesim::Kill::Request& req,
    sightedturtlesim::Kill::Response& res) {
  emit requestKillTurtle(req.turtleName);
  return true;
};


bool QTurtleGUI::killTurtle(const std::string& name) {
  if (robots->deleteTurtle(name)) {
    statusBar()->showMessage(tr("Robot %1 killed.").arg(QString::fromStdString(name)));
  } else {
    statusBar()->showMessage(tr("Could not kill robot %1.").arg(QString::fromStdString(name)));
    return false;
  }

  // Reset robots if no more robots available
  if (robots->size() <= 0) {
    robots->reset();
  }
  return true;
};


void QTurtleGUI::zoomIn() {
  scaleImage(1.25);
};


void QTurtleGUI::zoomOut() {
  scaleImage(0.8);
};


void QTurtleGUI::normalSize() {
  scaleFactor = 1.0;
  scaleImage(1.0);
};


void QTurtleGUI::fitToWindow() {
  bool fitToWindow = fitToWindowAct->isChecked();
  scrollArea->setWidgetResizable(fitToWindow);
  if (!fitToWindow) {
    normalSize();
  }
  updateActions();
};


void QTurtleGUI::toggleUpdates() {
  imageWidget->setUpdatesEnabled(toggleUpdatesAct->isChecked());
  imageWidget->autoRefresh(toggleUpdatesAct->isChecked());
  if (toggleUpdatesAct->isChecked()) {
    autoRefreshAct->setEnabled(true);
    autoRefreshAct->setChecked(true);
    statusBar()->showMessage(tr("Image updates and auto-refresh ENABLED."));
  } else {
    autoRefreshAct->setEnabled(false);
    autoRefreshAct->setChecked(false);
    statusBar()->showMessage(tr("Image updates and auto-refresh DISABLED."));
  }
};


void QTurtleGUI::autoRefresh() {
  autoRefreshAct->setChecked(imageWidget->autoRefresh(autoRefreshAct->isChecked()));
  if (autoRefreshAct->isChecked()) {
    statusBar()->showMessage(tr("Auto-refresh ENABLED."));
  } else {
    statusBar()->showMessage(tr("Auto-refresh DISABLED."));
  }
};


void QTurtleGUI::createActions() {
#define SETUP_ACTION(var, text, checkable, enabled, shortcutKey) \
    var##Act = new QAction(tr(text), this); \
    var##Act->setCheckable(checkable); \
    var##Act->setEnabled(enabled); \
    var##Act->setShortcut(tr(shortcutKey)); \
    connect(var##Act, SIGNAL(triggered()), this, SLOT(var()))

  SETUP_ACTION(open, "&Open Canvas Image", false, true, "O");
  SETUP_ACTION(spawn, "&Spawn Robot", false, false, "S");
  SETUP_ACTION(kill, "&Kill Robot", false, false, "K");
  SETUP_ACTION(close, "Close", false, true, "X");
  SETUP_ACTION(zoomIn, "Zoom In (25%)", false, false, "=");
  SETUP_ACTION(zoomOut, "Zoom Out (25%)", false, false, "-");
  SETUP_ACTION(normalSize, "&Normal Size", false, false, "N");
  SETUP_ACTION(fitToWindow, "&Fit to Window", true, false, "F");
  SETUP_ACTION(toggleUpdates, "&Draw Updates", true, true, "D");
  SETUP_ACTION(autoRefresh, "Auto &Refresh", true, false, "R");

  toggleUpdatesAct->setChecked(true);

#undef SETUP_ACTION
};


void QTurtleGUI::createMenus() {
  fileMenu = new QMenu(tr("&File"), this);
  fileMenu->addAction(openAct);
  fileMenu->addSeparator();
  fileMenu->addAction(spawnAct);
  fileMenu->addAction(killAct);
  fileMenu->addSeparator();
  fileMenu->addAction(closeAct);

  viewMenu = new QMenu(tr("&View"), this);
  viewMenu->addAction(zoomInAct);
  viewMenu->addAction(zoomOutAct);
  viewMenu->addAction(normalSizeAct);
  viewMenu->addSeparator();
  viewMenu->addAction(fitToWindowAct);
  viewMenu->addSeparator();
  viewMenu->addAction(autoRefreshAct);
  viewMenu->addAction(toggleUpdatesAct);

  menuBar()->addMenu(fileMenu);
  menuBar()->addMenu(viewMenu);
};


void QTurtleGUI::updateActions() {
  zoomInAct->setEnabled(!fitToWindowAct->isChecked());
  zoomOutAct->setEnabled(!fitToWindowAct->isChecked());
  normalSizeAct->setEnabled(!fitToWindowAct->isChecked());
};


void QTurtleGUI::scaleImage(double factor) {
  scaleFactor *= factor;
  imageWidget->resize(scaleFactor);

  adjustScrollBar(scrollArea->horizontalScrollBar(), factor);
  adjustScrollBar(scrollArea->verticalScrollBar(), factor);
};


void QTurtleGUI::adjustScrollBar(QScrollBar *scrollBar, double factor) {
  scrollBar->setValue(int(factor * scrollBar->value()
      + ((factor - 1) * scrollBar->pageStep()/2)));
};
