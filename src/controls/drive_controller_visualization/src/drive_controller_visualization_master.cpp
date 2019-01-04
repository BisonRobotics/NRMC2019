#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

#include <drive_controller_visualization/dcvis_multiplot.h>

#include <geometry_msgs/PoseStamped.h>

cv::String names[] = {"Xpos", "Ypos", "Angle"};
cv::Scalar colors[] = {cv::Scalar(60,180,120), cv::Scalar(240,180,60), 
                       cv::Scalar(120, 60, 180)};
dcvis_multiplot dcvmp(3, "twenter", -5, 5, 5, 30, 5, names, colors);
ros::Time start_time;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ros::Duration plot_time = msg->header.stamp - start_time;
    double t = plot_time.toSec();
    dcvmp.add_point(msg->pose.position.x, t, 0);
    dcvmp.add_point(msg->pose.position.y, t, 1);
    double norm = std::sqrt(msg->pose.orientation.x * msg->pose.orientation.x +
                            msg->pose.orientation.y * msg->pose.orientation.y +
                            msg->pose.orientation.z * msg->pose.orientation.z);
    double theta = 2.0 * std::atan2(norm, msg->pose.orientation.w);
    int sign = (msg->pose.orientation.z / std::sin(theta/2.0)) > 0 ? 1 : -1;
    dcvmp.add_point(sign*theta, t, 2);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_controller_vis");
    ros::NodeHandle n;
    start_time = ros::Time::now();
    ros::Subscriber posSub = n.subscribe("/vrep/pose", 100, poseCallback);

    cv::Mat plotarea1(300,300,CV_8UC3, cv::Scalar(0,0,0));
    
    //for (int idex = 0; idex < 270; idex++)
    //{
        //dcvmp.add_point(std::sin(5.0*((double)idex)/200.0 - 5.0), 0);
    //}
    
    dcvmp.draw(plotarea1);
        
    cv::Mat framebuff(800, 400, CV_8UC3, cv::Scalar(170,70,70));
    cv::Point ppp(2,2);
    cv::circle(framebuff, ppp, 5, cv::Scalar(200,200,200));
    
    plotarea1.copyTo(framebuff(cv::Rect(50,50,300,300)));
    
    cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
    cv::imshow("disp", framebuff);
    
    //ros::Rate r(30);
    
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
        //std::cout << "LT: " <<loop_time.toSec() <<'\n';
        //dcvmp.add_point(std::sin(5.0*((double)idex++)/200.0 - 5.0), 0);
        dcvmp.draw(plotarea1);
        
        plotarea1.copyTo(framebuff(cv::Rect(50,50,300,300)));
        cv::imshow("disp", framebuff);

        key_code = cv::waitKey(30); //needed to service UI thread
        if (key_code == 113 || key_code == 81)
        {
            std::cout << "quit registered" << '\n';
            return 0;
        }
        //r.sleep(); wait is handled by waitkey
        ros::spinOnce();
        
    }
    
}