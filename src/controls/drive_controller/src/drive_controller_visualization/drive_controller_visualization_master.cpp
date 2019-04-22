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
#include <sensor_msgs/Imu.h>
#include <localization/StateVector.h>
#include <drive_controller/ErrorStates.h>
#include <drive_controller/WheelStates.h>
#include <drive_controller/PathInfo.h>

#define NUM_SERIES_PLOT1 7
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

#define NUM_SERIES_PLOT3 7
#define Y_MIN_POS_PLOT3 -1.8
#define Y_MAX_POS_PLOT3 1.8
#define NUM_Y_TICKS_PLOT3 5
#define TIME_WINDOW_PLOT3 10.0
#define TIME_TICK_PERIOD_PLOT3 2.0

#define NUM_SERIES_PLOT4 2
#define Y_MIN_POS_PLOT4 -0.5
#define Y_MAX_POS_PLOT4 0.5
#define NUM_Y_TICKS_PLOT4 5
#define TIME_WINDOW_PLOT4 30.0
#define TIME_TICK_PERIOD_PLOT4 5.0

#define NUM_SERIES_PLOT5 7
#define Y_MIN_POS_PLOT5 -0.8
#define Y_MAX_POS_PLOT5 0.8
#define NUM_Y_TICKS_PLOT5 5
#define TIME_WINDOW_PLOT5 30.0
#define TIME_TICK_PERIOD_PLOT5 5.0

cv::String names[] = {"Xposest", "Yposest", "Thetaest", "Xposmea", "Yposmea", "Thetamea", "ThetaPath"};
cv::String names2[] = {"Xvelest", "Yvelest", "Omegaest", "Xvelpre", "Yvestpre", "Omegapre"};
cv::String names3[] = {"Xaccest", "Yaccest", "Xaccmea", "Yaccmea" ,"Omegamea", "Xaccpre", "Yaccpre"};
cv::String names4[] = {"Patherr", "Angleerr"};
cv::String names5[] = {"Leftcmd", "Rightcmd", "Leftact", "Rightact", "Leftideal", "Rightideal", "OmegaPath"};

cv::Scalar colors6[] = {cv::Scalar(60,180,120), cv::Scalar(240,180,60), cv::Scalar(120, 60, 180),
                       cv::Scalar(60, 60,120), cv::Scalar(120,180,60), cv::Scalar(120, 180,180)};
cv::Scalar colors7[] = {cv::Scalar(60,180,120), cv::Scalar(240,180,60), cv::Scalar(120, 60, 180),
                       cv::Scalar(60, 60,120), cv::Scalar(120,180,60), cv::Scalar(120, 180,180),
                       cv::Scalar(200, 140,140)};
cv::Scalar colors4[] = {cv::Scalar(60,180,120), cv::Scalar(240,180,60),
                       cv::Scalar(60, 60,120), cv::Scalar(120,180,60)};
                       
dcvis_multiplot dcvmp(NUM_SERIES_PLOT1, "Positions", Y_MIN_POS_PLOT1, Y_MAX_POS_PLOT1, NUM_Y_TICKS_PLOT1,
                      TIME_WINDOW_PLOT1, TIME_TICK_PERIOD_PLOT1, names, colors7);
                      
dcvis_multiplot dcvmv(NUM_SERIES_PLOT2, "Velocities", Y_MIN_POS_PLOT2, Y_MAX_POS_PLOT2, NUM_Y_TICKS_PLOT2,
                      TIME_WINDOW_PLOT2, TIME_TICK_PERIOD_PLOT2, names2, colors6);
                      
dcvis_multiplot dcvma(NUM_SERIES_PLOT3, "Accelerations", Y_MIN_POS_PLOT3, Y_MAX_POS_PLOT3, NUM_Y_TICKS_PLOT3,
                      TIME_WINDOW_PLOT3, TIME_TICK_PERIOD_PLOT3, names3, colors7);
                      
dcvis_multiplot dcerr(NUM_SERIES_PLOT4, "Error States", Y_MIN_POS_PLOT4, Y_MAX_POS_PLOT4, NUM_Y_TICKS_PLOT4,
                      TIME_WINDOW_PLOT4, TIME_TICK_PERIOD_PLOT4, names4, colors4);
                      
dcvis_multiplot dcwhe(NUM_SERIES_PLOT5, "Wheel States", Y_MIN_POS_PLOT5, Y_MAX_POS_PLOT5, NUM_Y_TICKS_PLOT5,
                      TIME_WINDOW_PLOT5, TIME_TICK_PERIOD_PLOT5, names5, colors7);
                      
ros::Time start_time;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //get plot time
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    //add point to plot
    dcvmp.add_point(msg->pose.position.x, t, 3);
    dcvmp.add_point(msg->pose.position.y, t, 4);
    double norm = std::sqrt(msg->pose.orientation.x * msg->pose.orientation.x +
                            msg->pose.orientation.y * msg->pose.orientation.y +
                            msg->pose.orientation.z * msg->pose.orientation.z);
    double theta = 2.0 * std::atan2(norm, msg->pose.orientation.w);
    int sign = (msg->pose.orientation.z / std::sin(theta/2.0)) > 0 ? 1 : -1;
    dcvmp.add_point(sign*theta, t, 5);
}

//void accelerationCallback(const vrep_msgs::IMU::ConstPtr &msg)
void accelerationCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //sensor_msgs::ImuConstPtr
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    dcvma.add_point(msg->linear_acceleration.x, t, 2);
    dcvma.add_point(msg->linear_acceleration.z, t, 3);
    dcvma.add_point(msg->angular_velocity.y, t, 4);
}

void stateVectorCallback(const localization::StateVector::ConstPtr &msg)
{
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    dcvmp.add_point(msg->x_pos, t, 0);
    dcvmp.add_point(msg->y_pos, t, 1);
    dcvmp.add_point(msg->theta, t, 2);
    
    dcvmv.add_point(msg->x_vel, t, 0);
    dcvmv.add_point(msg->y_vel, t, 1);
    dcvmv.add_point(msg->omega, t, 2);
    
    dcvma.add_point(msg->x_accel, t, 0);
    dcvma.add_point(msg->y_accel, t, 1);
    //dcvma.add_point(msg->alpha,   t, 2); //There is no useful alpha estimate or measurement
}

void deltaVectorCallback(const localization::StateVector::ConstPtr &msg)
{
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    dcvmv.add_point(msg->x_pos, t, 3); //this is predicted vel
    dcvmv.add_point(msg->y_pos, t, 4);
    dcvmv.add_point(msg->theta, t, 5); //predicted angular vel
    
    dcvma.add_point(msg->x_vel, t, 5); //predicted accel
    dcvma.add_point(msg->y_vel, t, 6); //this delta vector is the change
    //dont worry about predicted alpha
}

void errorStatesCallback(const drive_controller::ErrorStates::ConstPtr &msg)
{
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    dcerr.add_point(msg->path_error, t, 0);
    dcerr.add_point(msg->angle_error, t, 1);
}

void wheelStatesCallback(const drive_controller::WheelStates::ConstPtr &msg)
{
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    dcwhe.add_point(msg->left_wheel_command, t, 0);
    dcwhe.add_point(msg->right_wheel_command, t, 1);
    dcwhe.add_point(msg->left_wheel_actual, t, 2);
    dcwhe.add_point(msg->right_wheel_actual, t, 3);
    dcwhe.add_point(msg->left_wheel_planned, t, 4);
    dcwhe.add_point(msg->right_wheel_planned, t, 5);
}

void pathInfoCallback(const drive_controller::PathInfo::ConstPtr &msg)
{
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    dcvmp.add_point(msg->path_theta, t, 6);
    dcwhe.add_point(msg->path_omega, t, 6);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_controller_vis");
    ros::NodeHandle n;
    start_time = ros::Time::now();
    //listener = new tf::TransformListener();
    bool halfsim = true;
    ros::Subscriber posSub = n.subscribe(halfsim ? "/tracker0/pose_estimate" : "/vrep/pose", 100, poseCallback);
    ros::Subscriber imuSub = n.subscribe(halfsim ? "/imu" : "vrep/imu", 100, accelerationCallback);
    ros::Subscriber svSub  = n.subscribe("/position_controller/state_vector", 100, stateVectorCallback);
    ros::Subscriber dvSub  = n.subscribe("/position_controller/delta_vector", 100, deltaVectorCallback);
    ros::Subscriber esSub  = n.subscribe("/position_controller/error_states", 100, errorStatesCallback);
    ros::Subscriber wsSub  = n.subscribe("/position_controller/wheel_states", 100, wheelStatesCallback);
    ros::Subscriber piSub  = n.subscribe("/position_controller/path_info", 100, pathInfoCallback);

    static const int plotsize_width  = 400;
    static const int plotsize_height = 330;
    static const int plot_margin = 2;
    cv::Mat plotarea1(plotsize_height,plotsize_width,CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat plotarea2(plotsize_height,plotsize_width,CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat plotarea3(plotsize_height,plotsize_width,CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat plotarea4(plotsize_height,plotsize_width,CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat plotarea5(plotsize_height,plotsize_width,CV_8UC3, cv::Scalar(0,0,0));
    
    dcvmp.draw(plotarea1);
    dcvmv.draw(plotarea2);
    dcvmv.draw(plotarea3);
    dcerr.draw(plotarea4);
    dcwhe.draw(plotarea5);
        
    cv::Mat framebuff(1000, plotsize_width*2 + plot_margin*3, CV_8UC3, cv::Scalar(170,70,70));
    //First you draw a circle...
    cv::Point ppp(20,930);
    cv::circle(framebuff, ppp, 8, cv::Scalar(200,200,200));
    
    plotarea1.copyTo(framebuff(cv::Rect(plot_margin ,plot_margin,plotsize_width,plotsize_height)));
    plotarea2.copyTo(framebuff(cv::Rect(plotsize_width + 2*plot_margin,plot_margin,plotsize_width, plotsize_height)));
    plotarea3.copyTo(framebuff(cv::Rect(plotsize_width + 2*plot_margin,plotsize_height + 2*plot_margin,plotsize_width, plotsize_height)));
    plotarea4.copyTo(framebuff(cv::Rect(plot_margin,plotsize_height + 2*plot_margin,plotsize_width, plotsize_height)));
    plotarea5.copyTo(framebuff(cv::Rect(plot_margin,2*plotsize_height + 3*plot_margin,plotsize_width, plotsize_height)));
    
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
        dcerr.draw(plotarea4);
        dcwhe.draw(plotarea5);
        
        plotarea1.copyTo(framebuff(cv::Rect(plot_margin ,plot_margin,plotsize_width,plotsize_height)));
        plotarea2.copyTo(framebuff(cv::Rect(plotsize_width + 2*plot_margin,plot_margin,plotsize_width, plotsize_height)));
        plotarea3.copyTo(framebuff(cv::Rect(plotsize_width + 2*plot_margin,plotsize_height + 2*plot_margin,plotsize_width, plotsize_height)));
        plotarea4.copyTo(framebuff(cv::Rect(plot_margin,plotsize_height + 2*plot_margin,plotsize_width, plotsize_height)));
        plotarea5.copyTo(framebuff(cv::Rect(plot_margin,2*plotsize_height + 3*plot_margin,plotsize_width, plotsize_height)));

        cv::imshow("disp", framebuff);
        
        ros::spinOnce();        
        ros::spinOnce();        
        ros::spinOnce();        
        ros::spinOnce();        
        ros::spinOnce();        

        key_code = cv::waitKey(30); //needed to service UI thread
        if (key_code == 113 || key_code == 81)
        {
            std::cout << "quit registered" << '\n';
            //delete listener;
            return 0;
        }
    }
}