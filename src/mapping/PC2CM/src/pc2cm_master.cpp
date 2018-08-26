#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>


#define CELL_WIDTH .10f
#define Z_LOWER -2.0f
#define Z_UPPER 2.0f

#define MAP_WIDTH 3.0f
#define MAP_LENGTH 3.0f

#define CLAMP(A, B, C) ((A < B) ? B : ((A>C) ? C : A))

std::vector<float> *curr_points;
uint32_t height;
uint32_t row_step;
uint32_t point_step;
bool new_points_here;
bool first_point;

void newPointsCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    if (!new_points_here)
    {
      float* point_data = (float*)(void*)(cloud_msg->data.data());
      if (first_point)
      {
         curr_points = new std::vector<float> (point_data, 
                       point_data + (cloud_msg->data.size()/sizeof(float)));
         first_point = false;
      }
      else 
      {
         delete curr_points;
         curr_points = new std::vector<float> (point_data, 
                       point_data + (cloud_msg->data.size()/sizeof(float)));
      }
    height = cloud_msg->height;
    row_step = cloud_msg->row_step;
    point_step = cloud_msg->point_step;

    new_points_here = true;
    ROS_INFO("PCL: %d, %d\n", (int)curr_points->size(), (int) cloud_msg->data.size());

    ROS_INFO("FIELDS: %d, name1: %s, offset: %d, datatype: %d, count: %d", cloud_msg->fields.size(), 
              cloud_msg->fields.at(0).name, cloud_msg->fields.at(0).offset,
              cloud_msg->fields.at(0).datatype, cloud_msg->fields.at(0).count);
    ROS_INFO("FIELDS: %d, name1: %s, offset: %d, datatype: %d, count: %d", cloud_msg->fields.size(), 
              cloud_msg->fields.at(1).name, cloud_msg->fields.at(1).offset,
              cloud_msg->fields.at(1).datatype, cloud_msg->fields.at(1).count);
    ROS_INFO("FIELDS: %d, name1: %s, offset: %d, datatype: %d, count: %d", cloud_msg->fields.size(), 
              cloud_msg->fields.at(2).name, cloud_msg->fields.at(2).offset,
              cloud_msg->fields.at(2).datatype, cloud_msg->fields.at(2).count);
    }
}

int main(int argc, char** argv)
{
  new_points_here = false;
  first_point = true;
  ros::init(argc, argv, "pc2cm_master");
  ros::NodeHandle node;

  ros::Rate rate(100.0);

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

    //grid is from +/- 2m on y and +3m on X
    float gridTotal[(int)(MAP_LENGTH/CELL_WIDTH)][(int) (MAP_WIDTH/CELL_WIDTH)] = {0}; 
    float gridNew[(int)(MAP_LENGTH/CELL_WIDTH)][(int) (MAP_WIDTH/CELL_WIDTH)] = {0}; 

  while (ros::ok())
  {
    if (new_points_here)
    {
       new_points_here = false;
       int dim =0;
       float* point_el = curr_points->data();
       for (int i=0; i < curr_points->size();i+=3)
       {
          //point (0,0) should map to index[0][.5*MAP_WIDTH]
          //point (1,0) should map to index[1/CELL_WIDTH][.5*MAP_WIDTH/CELL_WIDTH]
          // point (1,1) should map to index(1/CELL_WIDTH][1/CELL_WIDTH + .5*MAP_WIDTH/CELL_WIDTH
          int x_index = (int)(-point_el[i+1]/CELL_WIDTH );
          x_index = CLAMP(x_index, 0, MAP_LENGTH/CELL_WIDTH);

          int y_index = (int)(point_el[i+2]/CELL_WIDTH + (.5*MAP_WIDTH/CELL_WIDTH));
          y_index = CLAMP(y_index, 0, MAP_WIDTH/CELL_WIDTH);

          gridTotal[x_index][y_index] = (1.0)*gridTotal[x_index][y_index] 
                                      + .02*(point_el[i+1] - gridTotal[x_index][y_index]);

          ROS_INFO("NEW Point: %.4f, %.4f, %.4f", point_el[i+1], point_el[i], point_el[i+2]);
          ROS_INFO("Mapped to: %d, %d, %.4f\n", x_index, y_index, gridTotal[x_index][y_index]);
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
            my_point.x = i * CELL_WIDTH;
            my_point.y = j * CELL_WIDTH - .5*MAP_WIDTH;
            my_point.z = gridTotal[i][j];
            goal_markers.points.push_back(my_point);
         }
       }
       //take DoG over total bins
       //project bins onto costmap

       pub.publish(goal_markers);
       ros::spinOnce();
       rate.sleep();
       

    }
    ROS_INFO("CHUNK\n");
    //pub.publish(goal_markers);
    ros::spinOnce();
    rate.sleep();
  }
 
}