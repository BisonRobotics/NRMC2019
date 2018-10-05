#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pc2_processor/pc2_processor.h>



std::vector<float> *curr_points;
std::vector<pcl::PointXYZ> copied_points;
uint32_t height;
uint32_t row_step;
uint32_t point_step;
bool new_points_here;
pc2cmProcessor processor;
double defualtCellWidth = 0.25;
double defualtMapWidth = 4.0;
double defualtMapHeight = 4.0;

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
    ros::init(argc, argv, "pc2cm_master");
    ros::NodeHandle node;

    ros::Rate rate(10);

    ros::Subscriber sub = node.subscribe("camera/points", 2, processor.addPoints);
    ros::Publisher pub = node.advertise<visualization_msgs::Marker>("heights", 10000);

    // visualization_msgs::Marker goal_markers;
    // goal_markers.action = visualization_msgs::Marker::ADD;
    // goal_markers.pose.orientation.w = 1;
    // goal_markers.type = visualization_msgs::Marker::CUBE_LIST;
    // goal_markers.scale.x = CELL_WIDTH;
    // goal_markers.scale.y = CELL_WIDTH;
    // goal_markers.scale.z = CELL_WIDTH;
    // goal_markers.color.b = .71;
    // goal_markers.color.g = 1;
    // goal_markers.color.a = .5;

    // goal_markers.header.frame_id = "/camera_link";

    // geometry_msgs::Point my_point;
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

    processor = pc2cmProcessor(defualtCellWidth, defualtMapWidth, defualtMapHeight);

    while (ros::ok())
    {
        if (new_points_here)
        {
            int dim =0;
            for (int i=0; i < copied_points.size();i++)
            {
                processor.addPoints(copied_points.at(i))
                ROS_INFO("NEW Point: %.4f, %.4f, %.4f", copied_points.at(i).x,copied_points.at(i).y, copied_points.at(i).z);
                ROS_INFO("Mapped to: %d, %d, %.4f\n", x_index, y_index, gridTotal[x_index][y_index]);
                for (int j=0; j<6; j++)
                {
                    ROS_INFO("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                        gridTotal[j][0],  gridTotal[j][1],  gridTotal[j][2],
                        gridTotal[j][3],  gridTotal[j][4],  gridTotal[j][5]);
                }
            }
            //bin new points
            //average new bins
            //add new bins average to old bins average (low pass)
            //(optional) update markers locations with bin heights
            goal_markers.points.clear();
            for (int i=0; i<MAP_LENGTH/CELL_WIDTH; i++)
            {
                for (int j=0; j<MAP_WIDTH/CELL_WIDTH; j++)
                {
                    my_point.x = MAP_LENGTH - i * CELL_WIDTH;
                    my_point.y = j * CELL_WIDTH - .5*MAP_WIDTH;
                    my_point.z = gridTotal[i][j];
                    // goal_markers.points.push_back(my_point);
                }
            }
            //take DoG over total bins
            //project bins onto costmap

            // pub.publish(goal_markers);
            new_points_here = false;
            ros::spinOnce();
            rate.sleep();

        }
        ROS_INFO("CHUNK\n");
        //pub.publish(goal_markers);
        ros::spinOnce();
        rate.sleep();
    }

}