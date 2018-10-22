#include "qt_joystick/main_window.h"
#include <QApplication>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "qt_joystick");

  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
}
