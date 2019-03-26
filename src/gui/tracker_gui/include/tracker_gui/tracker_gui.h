#ifndef TRACKER_GUI_H
#define TRACKER_GUI_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tracker/SetUIntAction.h>
#include <image_transport/image_transport.h>

namespace Ui
{
class TrackerGUI;
}

class TrackerGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit TrackerGUI(QWidget *parent = nullptr);
  ~TrackerGUI();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public slots:
  void getBrightnessButtonClicked();
  void getExposureButtonClicked();
  void brightnessSpinBoxChanged(int);
  void exposureSpinBoxChanged(int);
  void brightnessSliderChanged(int);
  void exposureSliderChanged(int);
  void topicComboBoxChanged(QString);
  void compressionComboBoxChanged(QString);
  void enableButtonClicked();
  void handleServices();


private:
  Ui::TrackerGUI *ui;
  std::string topic_base, compression;
  bool enabled;
  int brightness, exposure;
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  actionlib::SimpleActionClient<tracker::SetUIntAction> *current_brightness_client, *current_exposure_client;
  actionlib::SimpleActionClient<tracker::SetUIntAction> brightness_client0, brightness_client1;
  actionlib::SimpleActionClient<tracker::SetUIntAction> exposure_client0, exposure_client1;
  QTimer *service_timer;
  QGraphicsScene scene;
};

#endif // TRACKER_GUI_H
