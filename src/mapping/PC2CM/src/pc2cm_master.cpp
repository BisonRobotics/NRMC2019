#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>


#define CELL_WIDTH .05f
#define Z_LOWER -2.0f
#define Z_UPPER 2.0f

std::vector<float> *curr_points;
uint32_t height;
uint32_t row_step;
uint32_t point_step;
bool new_points_here;

void newPointsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    
    curr_points = new std::vector<float> ((float*)((void*)(cloud_msg->data.data())), 
                           (float*)((void*)(cloud_msg->data.data())) 
                           + sizeof(cloud_msg->data.data())/(sizeof(float)));
    height = cloud_msg->height;
    row_step = cloud_msg->row_step;
    point_step = cloud_msg->point_step;

    new_points_here = true;
    ROS_INFO("PCL\n");
}

int main(int argc, char** argv)
{
  new_points_here = false;
  ros::init(argc, argv, "pc2cm_master");
  ros::NodeHandle node;

  ros::Rate rate(20.0);

  ros::Subscriber sub = node.subscribe("camera/points", 200, newPointsCallback);
  ros::Publisher pub = node.advertise<visualization_msgs::Marker>("heights", 10000);

    visualization_msgs::Marker goal_markers;
    goal_markers.action = visualization_msgs::Marker::ADD;
    goal_markers.pose.orientation.w = 1;
    goal_markers.type = visualization_msgs::Marker::CUBE_LIST;
    goal_markers.scale.x = CELL_WIDTH;
    goal_markers.scale.y = CELL_WIDTH;
    goal_markers.scale.z = CELL_WIDTH;
    goal_markers.color.b = .71;
    goal_markers.color.g = 1;
    goal_markers.color.a = .5;

    goal_markers.header.frame_id = "/camera_link";

    geometry_msgs::Point my_point;
    my_point.x = 2;
    my_point.y = 1;
    my_point.z = 2;
    goal_markers.points.push_back(my_point);
    my_point.x = 3;
    my_point.y = 1;
    my_point.z = 2;
    goal_markers.points.push_back(my_point);
    my_point.x = 3;
    my_point.y = 1;
    my_point.z = 4;
    goal_markers.points.push_back(my_point);

  while (ros::ok())
  {
    if (false && new_points_here)
    {
       new_points_here = false;

       //bin new points
       //average new bins
       //add new bins average to old bins average (low pass)
       //(optional) update markers locations with bin heights
       //take DoG over total bins
       //project bins onto costmap

       pub.publish(goal_markers);
       delete curr_points;

    }
    ROS_INFO("CHUNK\n");
    pub.publish(goal_markers);
    ros::spinOnce();
    rate.sleep();
  }
 
}