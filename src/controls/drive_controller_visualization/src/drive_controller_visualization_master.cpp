#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

#include <drive_controller_visualization/dcvis_multiplot.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>

#include <vrep_msgs/IMU.h>

#define NUM_SERIES_PLOT1 6
#define Y_MIN_POS_PLOT1 -5.0
#define Y_MAX_POS_PLOT1 5.0
#define NUM_Y_TICKS_PLOT1 5
#define TIME_WINDOW_PLOT1 30.0
#define TIME_TICK_PERIOD_PLOT1 5.0

#define NUM_SERIES_PLOT2 6
#define Y_MIN_POS_PLOT2 -5.0
#define Y_MAX_POS_PLOT2 5.0
#define NUM_Y_TICKS_PLOT2 5
#define TIME_WINDOW_PLOT2 30.0
#define TIME_TICK_PERIOD_PLOT2 5.0

#define NUM_SERIES_PLOT3 2
#define Y_MIN_POS_PLOT3 -.5
#define Y_MAX_POS_PLOT3 .5
#define NUM_Y_TICKS_PLOT3 5
#define TIME_WINDOW_PLOT3 30.0
#define TIME_TICK_PERIOD_PLOT3 5.0

cv::String names[] = {"Xpos", "Ypos", "Theta", "Xest", "Yest", "Thetaest"};
cv::String names2[] = {"Xvel", "Yvel", "Omega", "Xvest", "Yvest", "Omegaest"};
cv::String names3[] = {"Xacc", "Yacc", "Xaest", "Yaest"};
cv::Scalar colors6[] = {cv::Scalar(60,180,120), cv::Scalar(240,180,60), cv::Scalar(120, 60, 180),
                       cv::Scalar(60, 60,120), cv::Scalar(120,180,60), cv::Scalar(120, 180,180)};
cv::Scalar colors4[] = {cv::Scalar(60,180,120), cv::Scalar(240,180,60),
                       cv::Scalar(60, 60,120), cv::Scalar(120,180,60)};
                       
dcvis_multiplot dcvmp(NUM_SERIES_PLOT1, "Positions", Y_MIN_POS_PLOT1, Y_MAX_POS_PLOT1, NUM_Y_TICKS_PLOT1,
                      TIME_WINDOW_PLOT1, TIME_TICK_PERIOD_PLOT1, names, colors6);
                      
dcvis_multiplot dcvmv(NUM_SERIES_PLOT2, "Velocities", Y_MIN_POS_PLOT2, Y_MAX_POS_PLOT2, NUM_Y_TICKS_PLOT2,
                      TIME_WINDOW_PLOT2, TIME_TICK_PERIOD_PLOT2, names2, colors6);
                      
dcvis_multiplot dcvma(NUM_SERIES_PLOT3, "Accelerations", Y_MIN_POS_PLOT3, Y_MAX_POS_PLOT3, NUM_Y_TICKS_PLOT3,
                      TIME_WINDOW_PLOT3, TIME_TICK_PERIOD_PLOT3, names3, colors4);
                      
ros::Time start_time;

tf::TransformListener* listener = NULL;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //get plot time
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    //add point to plot
    dcvmp.add_point(msg->pose.position.x, t, 0);
    dcvmp.add_point(msg->pose.position.y, t, 1);
    double norm = std::sqrt(msg->pose.orientation.x * msg->pose.orientation.x +
                            msg->pose.orientation.y * msg->pose.orientation.y +
                            msg->pose.orientation.z * msg->pose.orientation.z);
    double theta = 2.0 * std::atan2(norm, msg->pose.orientation.w);
    int sign = (msg->pose.orientation.z / std::sin(theta/2.0)) > 0 ? 1 : -1;
    dcvmp.add_point(sign*theta, t, 2);
    //get what is believed to be true
    tf::StampedTransform local_transform;
    double plot_x;
    double plot_y;
    try
    {
        listener->lookupTransform("/base_link", "/map",
                                 ros::Time(0), local_transform);
                                 //note that using msg->header.stamp makes tf angry
                                 //this is b/c tf is a little (~.02s) behind.
        plot_x = local_transform.getOrigin().x();
        plot_y = local_transform.getOrigin().y();
        norm = std::sqrt(local_transform.getRotation().x() * local_transform.getRotation().x() +
                         local_transform.getRotation().y() * local_transform.getRotation().y() +
                         local_transform.getRotation().z() * local_transform.getRotation().z());
        theta = 2.0 * std::atan2(norm, msg->pose.orientation.w);
        sign = (msg->pose.orientation.z / std::sin(theta/2.0)) > 0 ? 1 : -1;
        theta = sign*theta;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      plot_x = 0;
      plot_y = 0;
      theta  = 0;
    }
    dcvmp.add_point(plot_x, t, 3);
    dcvmp.add_point(plot_y, t, 4);
    dcvmp.add_point(theta , t, 5);
}

void accelerationCallback(const vrep_msgs::IMU::ConstPtr &msg) //is this right?
{
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    dcvma.add_point(msg->linear_acceleration.x, t, 0);
    dcvma.add_point(msg->linear_acceleration.y, t, 1);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_controller_vis");
    ros::NodeHandle n;
    start_time = ros::Time::now();
    listener = new tf::TransformListener();
    ros::Subscriber posSub = n.subscribe("/vrep/pose", 100, poseCallback);
    ros::Subscriber imuSub = n.subscribe("/vrep/imu", 100, accelerationCallback);

    static const int plotsize_width  = 400;
    static const int plotsize_height = 330;
    cv::Mat plotarea1(plotsize_height,plotsize_width,CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat plotarea2(plotsize_height,plotsize_width,CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat plotarea3(plotsize_height,plotsize_width,CV_8UC3, cv::Scalar(0,0,0));
    
    dcvmp.draw(plotarea1);
    dcvmv.draw(plotarea2);
    dcvmv.draw(plotarea3);
        
    cv::Mat framebuff(950, 950, CV_8UC3, cv::Scalar(170,70,70));
    //First you draw a circle...
    cv::Point ppp(2,2);
    cv::circle(framebuff, ppp, 5, cv::Scalar(200,200,200));
    
    plotarea1.copyTo(framebuff(cv::Rect(50 ,35,plotsize_width,plotsize_height)));
    plotarea2.copyTo(framebuff(cv::Rect(500,35,plotsize_width, plotsize_height)));
    plotarea3.copyTo(framebuff(cv::Rect(500,330 + 2*35,plotsize_width, plotsize_height)));
    
    cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
    cv::imshow("disp", framebuff);
        
    std::cout << "Press q (in image window) to quit\n";
    int key_code = 0;
    int idex = 0;
    ros::Time last_time, curr_time;
    ros::Duration loop_time;
    curr_time = ros::Time::now();
    while( ros::ok())
    {
        last_time = curr_time;
        curr_time = ros::Time::now();
        loop_time = curr_time - last_time;

        dcvmp.draw(plotarea1);
        dcvmv.draw(plotarea2);
        dcvma.draw(plotarea3);
        
        plotarea1.copyTo(framebuff(cv::Rect(50 ,35,plotsize_width,plotsize_height)));
        plotarea2.copyTo(framebuff(cv::Rect(500,35,plotsize_width,plotsize_height)));
        plotarea3.copyTo(framebuff(cv::Rect(500,330 + 2*35,plotsize_width,plotsize_height)));
        cv::imshow("disp", framebuff);
        
        ros::spinOnce();        

        key_code = cv::waitKey(30); //needed to service UI thread
        if (key_code == 113 || key_code == 81)
        {
            std::cout << "quit registered" << '\n';
            delete listener;
            return 0;
        }
    }
}