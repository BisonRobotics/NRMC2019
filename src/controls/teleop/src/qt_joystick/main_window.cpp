#include "qt_joystick/main_window.h"
#include "ui_main_window.h"

#include <iostream>
#include <sensor_msgs/Joy.h>

using sensor_msgs::Joy;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  this->setWindowFlags(this->windowFlags() | Qt::WindowStaysOnTopHint);
  this->setVisible(true);

  joy_timer = new QTimer(this);
  connect(ui->reset_button,  SIGNAL(clicked(bool)),     this, SLOT(resetButtonClicked()));
  connect(ui->enabled,       SIGNAL(clicked(bool)),     this, SLOT(enabledCheckboxClicked(bool)));
  connect(ui->left_slider,   SIGNAL(valueChanged(int)), this, SLOT(leftSliderMoved()));
  connect(ui->center_slider, SIGNAL(valueChanged(int)), this, SLOT(centerSliderMoved()));
  connect(ui->right_slider,  SIGNAL(valueChanged(int)), this, SLOT(rightSliderMoved()));
  connect(joy_timer,         SIGNAL(timeout()),         this, SLOT(publishJoy()));

  nh = new ros::NodeHandle();
  publisher = new ros::Publisher;
  (*publisher) = nh->advertise<Joy>("joy", 10, true);

  this->resetButtonClicked();
  joy_timer->start(20);
}

MainWindow::~MainWindow()
{
  delete ui;
  delete joy_timer;

  nh->shutdown();
  publisher->shutdown();
  delete nh;
  delete publisher;
}

void MainWindow::enabledCheckboxClicked(bool checked)
{
  enabled = checked;
}

void MainWindow::resetButtonClicked()
{
  ui->left_slider->setValue(0);
  ui->center_slider->setValue(0);
  ui->right_slider->setValue(0);
  current_left = 0;
  current_center = 0;
  current_right = 0;
  offset_left = 0;
  offset_right = 0;
}

void MainWindow::startSliderMoved(QSlider *first, QSlider *second)
{
  current_left = ui->left_slider->value();
  current_center = ui->center_slider->value();
  current_right = ui->right_slider->value();
  first->blockSignals(true);
  second->blockSignals(true);
}

void MainWindow::endSliderMoved(QSlider *first, QSlider *second)
{
  first->blockSignals(false);
  second->blockSignals(false);
}

void MainWindow::leftSliderMoved()
{
  startSliderMoved(ui->center_slider, ui->right_slider);
  offset_left = current_left - current_center;
  offset_right = -offset_left;
  current_right = offset_right + current_center;
  ui->right_slider->setValue(current_right);
  endSliderMoved(ui->center_slider, ui->right_slider);
}

void MainWindow::rightSliderMoved()
{
  startSliderMoved(ui->left_slider, ui->center_slider);
  offset_right = current_right - current_center;
  offset_left = -offset_right;
  current_left = offset_left + current_center;
  ui->left_slider->setValue(current_left);
  endSliderMoved(ui->left_slider, ui->center_slider);
}

void MainWindow::centerSliderMoved()
{
  startSliderMoved(ui->left_slider, ui->right_slider);

  ui->left_slider->setValue(offset_left + current_center);
  ui->right_slider->setValue(offset_right + current_center);

  endSliderMoved(ui->left_slider, ui->right_slider);
}

float clamp(float value)
{
  if (value > 1.0f)
  {
    return 1.0f;
  }
  else if (value < -1.0f)
  {
    return -1.0f;
  }
  else
  {
    return value;
  }
}

void MainWindow::publishJoy()
{
  Joy joy;
  joy.header.stamp = ros::Time::now();
  joy.header.seq = seq++;
  joy.buttons.resize(5);
  joy.buttons[4] = enabled;
  joy.axes.resize(5);
  joy.axes[1] = clamp(current_left / 100.0f);
  joy.axes[4] = clamp(current_right / 100.0f);
  publisher->publish(joy);
}

