#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QtWidgets/QSlider>

#include <ros/ros.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

  public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow() override;

  public slots:
    void resetButtonClicked();
    void enabledCheckboxClicked(bool checked);
    void leftSliderMoved();
    void rightSliderMoved();
    void centerSliderMoved();
    void publishJoy();

  private:
    void startSliderMoved(QSlider *first, QSlider *second);
    void endSliderMoved(QSlider *first, QSlider *second);

    int offset_left, offset_right;
    int current_left, current_center, current_right;
    bool enabled;

    Ui::MainWindow *ui;
    QTimer *joy_timer;
    ros::NodeHandle *nh;
    ros::Publisher *publisher;
    uint32_t seq;
};

#endif // MAINWINDOW_H
