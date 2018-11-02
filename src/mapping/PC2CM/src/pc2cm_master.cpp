#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pc2_processor/pc2_processor.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>


using namespace std;


std::vector<float> *curr_points;
std::vector<pcl::PointXYZ> copied_points;

bool new_points_here;
pc2cmProcessor processor;
double defaultCellWidth = 0.25;
double defaultMapWidth  = 4.0;
double defaultMapHeight = 4.0;

ros::Publisher *pub;

void newPointsCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg)
{
    ROS_INFO("start callback");
    if (!new_points_here)
    {
        copied_points.clear();
        ROS_INFO("cleared points");

        for (const pcl::PointXYZ& pt : cloud_msg->points)
        {
            if (!(pt.x ==0 && pt.y ==0 && pt.z ==0))
            {

                copied_points.push_back(pt); // add the point to copied_points
                new_points_here = true;
            }
        }

        ROS_INFO("PCL: %d, %d\n", (int)copied_points.size(), 0);
    }
}

int main(int argc, char** argv)
{
    new_points_here = false;
    processor = pc2cmProcessor(defaultCellWidth, defaultMapWidth, defaultMapHeight);

    ros::init(argc, argv, "pc2cm_master");
    ros::NodeHandle node;

    ros::Rate rate(10);

    nav_msgs::OccupancyGrid obsticals ;

    ros::Subscriber sub = node.subscribe("camera/points", 2, newPointsCallback);
    // ros::Publisher pub = node.advertise<nav_msgs::OccupancyGrid.msg>("costMap", 1, true);

    ros::NodeHandle nh;
    pub = new ros::Publisher;
    (*pub) = nh.advertise<nav_msgs::OccupancyGrid>("map", 10, true);

    processor.takeDoG(9, .8, .2);


    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/my_frame";
        if (new_points_here)
        {
            for (int i=0; i < copied_points.size();i++)
            {
                processor.addPoint(copied_points.at(i));
                ROS_INFO("NEW Point: %.4f, %.4f, %.4f", copied_points.at(i).x,copied_points.at(i).y, copied_points.at(i).z);
            }
            std::cout << "making grid" << std::endl;
            processor.computeOccupancyGrid(&obsticals);

            // pub.publish(obsticals);

            pub->publish(obsticals);
            new_points_here = false;
            ros::spinOnce();
            rate.sleep();

        }
        processor.computeOccupancyGrid(&obsticals);

        ROS_INFO("CHUNK\n");
        int sum = 0;

        for (int i = 0; i < sizeof(obsticals.data); i++) {
            sum = sum + obsticals.data[i];
        }
        std::cout << sum << std::endl;
        ofstream myfile ("saveData.txt");
        if (myfile.is_open())
        {
            // myfile << "This is a line.\n";
            // myfile << "This is another line.\n";
            for(int count = 0; count < sizeof(obsticals.data); count ++){
                myfile << obsticals.data[count] << " " ;
            }
            myfile.close();
        }
        pub->publish(obsticals);

        ros::spinOnce();
        rate.sleep();
    }

}