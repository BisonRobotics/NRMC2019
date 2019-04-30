#include <tracker/tracker_gui.h>
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tracker_gui_node");
  QApplication a(argc, argv);
  TrackerGUI w;
  w.show();

  return a.exec();
}
