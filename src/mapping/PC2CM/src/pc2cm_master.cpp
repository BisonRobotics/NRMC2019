#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pc2_processor/pc2_processor.h>



std::vector<float> *curr_points;
std::vector<pcl::PointXYZ> copied_points;

bool new_points_here;
pc2cmProcessor processor;
double defaultCellWidth = 0.25;
double defaultMapWidth  = 4.0;
double defaultMapHeight = 4.0;

void newPointsCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg)
{
    ROS_INFO("start callback");
    if (!new_points_here)
    {
        copied_points.clear();
        ROS_INFO("cleared points");

        for (const pcl::PointXYZ& pt : cloud_msg->points)
        {
            //ROS_INFO("%f, %f, %f", pt.x, pt.y, pt.z);
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

    ros::Subscriber sub = node.subscribe("camera/points", 2, newPointsCallback);
    ros::Publisher pub = node.advertise<visualization_msgs::Marker>("heights", 10000);

    visualization_msgs::Marker goal_markers;
    goal_markers.action = visualization_msgs::Marker::ADD;
    goal_markers.pose.orientation.w = 1;
    goal_markers.type = visualization_msgs::Marker::CUBE_LIST;
    goal_markers.scale.x = defaultCellWidth;
    goal_markers.scale.y = defaultCellWidth;
    goal_markers.scale.z = defaultCellWidth;
    goal_markers.color.b = .71;
    goal_markers.color.g = 1;
    goal_markers.color.a = .5;

    goal_markers.header.frame_id = "/camera_link";
    double gridTotal[][]
    geometry_msgs::Point my_point;
    // my_point.x = 2;
    // my_point.y = 1;
    // my_point.z = 2;
    // goal_markers.points.push_back(my_point);
    // my_point.x = 3;
    // my_point.y = 1;
    // my_point.z = 2;
    // goal_markers.points.push_back(my_point);
    // my_point.x = 3;
    // my_point.y = 1;
    // my_point.z = 4;
    // goal_markers.points.push_back(my_point);


    while (ros::ok())
    {
        if (new_points_here)
        {
            // int dim =0;
            for (int i=0; i < copied_points.size();i++)
            {
                processor.addPoint(copied_points.at(i));
                ROS_INFO("NEW Point: %.4f, %.4f, %.4f", copied_points.at(i).x,copied_points.at(i).y, copied_points.at(i).z);
                // ROS_INFO("Mapped to: %d, %d, %.4f\n", x_index, y_index, gridTotal[x_index][y_index]);
                // for (int j=0; j<6; j++)
                // {
                //     ROS_INFO("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                //         gridTotal[j][0],  gridTotal[j][1],  gridTotal[j][2],
                //         gridTotal[j][3],  gridTotal[j][4],  gridTotal[j][5]);
                // }
            }
            //bin new points
            //average new bins
            //add new bins average to old bins average (low pass)
            //(optional) update markers locations with bin heights
            // goal_markers.points.clear();
            // for (int i=0; i<defaultMapWidth/defaultCellWidth; i++)
            // {
            //     for (int j=0; j<defaultMapHeight/defaultCellWidth; j++)
            //     {
            //         my_point.x = defaultMapWidth - i * defaultCellWidth;
            //         my_point.y = j * defaultCellWidth - .5*defaultMapHeight;
            //         my_point.z = gridTotal[i][j];
            //         goal_markers.points.push_back(my_point);
            //     }
            // }
            //take DoG over total bins
            //project bins onto costmap

            // pub.publish(proccessoue);
            new_points_here = false;
            ros::spinOnce();
            rate.sleep();

        }
        ROS_INFO("CHUNK\n");
        pub.publish(proccesor.computeCostmap());
        ros::spinOnce();
        rate.sleep();
    }

}