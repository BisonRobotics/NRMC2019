#include <tracker/tracker_gui.h>
#include <ui_tracker_gui.h>
#include <tracker/GetUInt.h>
#include <tracker/SetUIntAction.h>
#include <QTimer>
#include <QtCore/QtCore>
#include <opencv2/opencv.hpp>

TrackerGUI::TrackerGUI(QWidget *parent) :
    QMainWindow(parent), ui(new Ui::TrackerGUI),
    topic_base("/tracker/left"), compression("raw"), enabled(false),
    brightness_client0("/tracker/left/set_brightness", false),
    brightness_client1("/tracker/right/set_brightness", false),
    exposure_client0("/tracker/left/set_exposure", false),
    exposure_client1("/tracker/right/set_exposure", false),
    it(nh), scene(this)
{
  service_timer = new QTimer(this);

  ui->setupUi(this);
  connect(ui->getBrightnessButton,   SIGNAL(clicked(bool)),                this, SLOT(getBrightnessButtonClicked()));
  connect(ui->getExposureButton,     SIGNAL(clicked(bool)),                this, SLOT(getExposureButtonClicked()));
  connect(ui->brightnessSpinBox,     SIGNAL(valueChanged(int)),            this, SLOT(brightnessSpinBoxChanged(int)));
  connect(ui->exposureSpinBox,       SIGNAL(valueChanged(int)),            this, SLOT(exposureSpinBoxChanged(int)));
  connect(ui->brightnessSlider,      SIGNAL(valueChanged(int)),            this, SLOT(brightnessSliderChanged(int)));
  connect(ui->exposureSlider,        SIGNAL(valueChanged(int)),            this, SLOT(exposureSliderChanged(int)));
  connect(ui->topicComboBox,         SIGNAL(currentIndexChanged(QString)), this, SLOT(topicComboBoxChanged(QString)));
  connect(ui->compressionComboBox,   SIGNAL(currentIndexChanged(QString)), this, SLOT(compressionComboBoxChanged(QString)));
  connect(ui->enableCheckbox,        SIGNAL(stateChanged(int)),            this, SLOT(enableButtonClicked()));
  connect(service_timer,             SIGNAL(timeout()),                    this, SLOT(handleServices()));

  ui->brightnessSpinBox->setMaximum(255);
  ui->brightnessSlider->setMaximum(255);
  ui->exposureSlider->setMaximum(195);
  ui->exposureSpinBox->setMaximum(195);
  ui->imageView->setScene(&scene);
  ui->topicComboBox->addItem("/tracker/left");
  ui->topicComboBox->addItem("/tracker/right");
  ui->compressionComboBox->addItem("raw");
  ui->compressionComboBox->addItem("compressed");
  ui->compressionComboBox->addItem("theora");

  current_brightness_client = &brightness_client0;
  current_exposure_client = &exposure_client0;
  getBrightnessButtonClicked();
  getExposureButtonClicked();
  service_timer->start(10);
}

TrackerGUI::~TrackerGUI()
{
  delete ui;
}

void TrackerGUI::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (enabled)
  {
    QImage::Format format;
    if (compression != "theora")
    {
      format = QImage::Format::Format_Grayscale8;
    }
    else
    {
      format = QImage::Format::Format_RGB888;
    }
    QImage image = QImage(msg->data.data(), msg->width, msg->height, format);
    image = image.scaled(ui->imageView->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
    QPixmap pix_image = QPixmap::fromImage(image);
    scene.clear();
    scene.addPixmap(pix_image);
  }
}

void TrackerGUI::handleServices()
{
  using actionlib::SimpleClientGoalState;

  if (!ros::ok())
  {
    this->close();
  }

  int value;
  SimpleClientGoalState state(SimpleClientGoalState::SUCCEEDED);
  if (enabled)
  {
    // Brightness
    value = ui->brightnessSpinBox->value();
    state = current_brightness_client->getState();
    if (value != brightness &&
        state != SimpleClientGoalState::PENDING &&
        state != SimpleClientGoalState::ACTIVE)
    {
      brightness = value;
      tracker::SetUIntActionGoal request;
      request.goal.value = (uint)value;
      current_brightness_client->sendGoal(request.goal);
    }

    // Exposure
    value = ui->exposureSpinBox->value();
    state = current_exposure_client->getState();
    if (value != exposure &&
        state != SimpleClientGoalState::PENDING &&
        state != SimpleClientGoalState::ACTIVE)
    {
      exposure = value;
      tracker::SetUIntActionGoal request;
      request.goal.value = (uint)value;
      current_exposure_client->sendGoal(request.goal);
    }
  }
  ros::spinOnce();

}

void TrackerGUI::getBrightnessButtonClicked()
{
  tracker::GetUInt request;
  if (ros::service::call(topic_base + "/get_brightness", request))
  {
      ui->brightnessSlider->setValue(request.response.value);
      ui->brightnessSpinBox->setValue(request.response.value);
  }
  else
  {
      printf("[ERROR] Unable to get brightness\n");
  }
}

void TrackerGUI::getExposureButtonClicked()
{
  tracker::GetUInt request;
  if (ros::service::call(topic_base + "/get_exposure", request))
  {
      ui->exposureSlider->setValue(request.response.value);
      ui->exposureSpinBox->setValue(request.response.value);
  }
  else
  {
      printf("[ERROR] Unable to get exposure\n");
  }
}

void TrackerGUI::brightnessSpinBoxChanged(int value)
{
  ui->brightnessSlider->setValue(value);
}

void TrackerGUI::exposureSpinBoxChanged(int value)
{
  ui->exposureSlider->setValue(value);
}

void TrackerGUI::brightnessSliderChanged(int value)
{
  ui->brightnessSpinBox->setValue(value);
}

void TrackerGUI::exposureSliderChanged(int value)
{
  ui->exposureSpinBox->setValue(value);
}

void TrackerGUI::topicComboBoxChanged(QString value)
{
  topic_base = value.toStdString();
  if (topic_base == "/tracker/left")
  {
    current_brightness_client = &brightness_client0;
    current_exposure_client = &exposure_client0;
  }
  else if (topic_base == "/tracker/right")
  {
    current_brightness_client = &brightness_client1;
    current_exposure_client = &exposure_client1;
  }
  else
  {
    printf("[ERROR] invalid topic %s", topic_base.c_str());
  }
}

void TrackerGUI::compressionComboBoxChanged(QString value)
{
  compression = value.toStdString();
  if (enabled)
  {
    sub.shutdown();
    sub = it.subscribe(topic_base + "/image", 1, &TrackerGUI::imageCallback, this,
        image_transport::TransportHints(compression, ros::TransportHints().unreliable(), ros::NodeHandle(topic_base + "/image")));
  }
}

void TrackerGUI::enableButtonClicked()
{
  enabled = ui->enableCheckbox->isChecked();
  if (enabled)
  {
    getBrightnessButtonClicked();
    getExposureButtonClicked();
    sub = it.subscribe(topic_base + "/image", 1, &TrackerGUI::imageCallback, this,
        image_transport::TransportHints(compression, ros::TransportHints().unreliable(), ros::NodeHandle(topic_base + "/image")));
  }
  else
  {
    sub.shutdown();
  }
}