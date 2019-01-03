#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_controller_vis");

    ros::NodeHandle n("~");
    
    cv::Mat framebuff(800, 400, CV_8UC3, cv::Scalar(170,70,70));
    cv::Point ppp(2,2);
    cv::circle(framebuff, ppp, 5, cv::Scalar(200,200,200));
    
    cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
    cv::imshow("disp", framebuff);
    
    ros::Rate r(30);
    
    std::cout << "Press q (in here) to quit\n";
    int key_code = 0;
    
    while( ros::ok())
    {
        key_code = cv::waitKey(60); //needed to service UI thread
        if (key_code == 113 || key_code == 81)
        {
            std::cout << "quit registered" << '\n';
            return 0;
        }
        r.sleep();
        ros::spinOnce();
        
    }
    
}