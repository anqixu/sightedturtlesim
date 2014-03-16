#include "sightedturtlesim/QTurtleGUI.hpp"
#include <QtGui>
#include <QApplication>
#include <csignal>


int main(int argc, char** argv) {
  ros::init(argc, argv, "sightedturtlesim", ros::init_options::NoSigintHandler);
  QApplication app(argc, argv);
  QTurtleGUI* gui = new QTurtleGUI();
  struct sigaction sigint;
  sigint.sa_handler = QTurtleGUI::sigintHandler;
  sigemptyset(&sigint.sa_mask);
  sigint.sa_flags = 0;
  sigint.sa_flags |= SA_RESTART;
  sigaction(SIGINT, &sigint, 0);
  gui->show();
  return app.exec();
};
