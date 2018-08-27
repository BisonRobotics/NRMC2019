#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


#define CELL_WIDTH .25f
#define Z_LOWER -8.0f
#define Z_UPPER 8.0f

#define MAP_WIDTH 3.0f
#define MAP_LENGTH 3.0f

#define CLAMP(A, B, C) ((A < B) ? B : ((A>C) ? C : A))

std::vector<float> *curr_points;
std::vector<pcl::PointXYZ> copied_points;
uint32_t height;
uint32_t row_step;
uint32_t point_step;
bool new_points_here;
bool first_point;

/**
   STOLEN FROM THE INTERNET
   Function to convert 2D pixel point to 3D point by extracting point
   from PointCloud2 corresponding to input pixel coordinate. This function
   can be used to get the X,Y,Z coordinates of a feature using an
   RGBD camera, e.g., Kinect.
   */
   void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
   {
     // get width and height of 2D point cloud data
     int width = pCloud.width;
     int height = pCloud.height;

     // Convert from u (column / width), v (row/height) to position in array
     // where X,Y,Z data starts
     int arrayPosition = u*pCloud.row_step + v*pCloud.point_step;

     // compute position in array where x,y,z data start
     int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
     int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
     int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

     float X = 0.0;
     float Y = 0.0;
     float Z = 0.0;

     memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
     memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
     memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

     ROS_INFO("%f, %f, %f", X, Y, Z);

    // put data into the point p
     p.x = X;
     p.y = -Y;
     p.z = Z;

   }

void newPointsCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg)
{
    ROS_INFO("start callback");
    if (!new_points_here)
    {
      //float* point_data = (float*)(void*)(cloud_msg->data.data());
      if (first_point)
      {
         first_point = false;
      }
      else 
      {
          //copied_points.clear();
      }
      copied_points.clear();
      ROS_INFO("cleared points");
    
//might be bad
    //for (long unsigned int index=0; index<cloud_msg->fields.at(0).count ; index++) //up to cloud_msg->fields.at(0).count?
    //{
        //ROS_INFO("going to call function");
        //pixelTo3DPoint(*cloud_msg, (index/(cloud_msg->row_step/cloud_msg->point_step)), (index % (cloud_msg->row_step/cloud_msg->point_step)), p);
        //ROS_INFO("pushing back %d", index);
        //if (!(p.x ==0 && p.y ==0 && p.z ==0))
        //{
        //   copied_points.push_back(p);
         //  new_points_here = true;
        //}

   // }
    for (const pcl::PointXYZ& pt : cloud_msg->points)
    {
       //ROS_INFO("%f, %f, %f", pt.x, pt.y, pt.z);
        if (!(pt.x ==0 && pt.y ==0 && pt.z ==0))
        {
           copied_points.push_back(pt);
           new_points_here = true;
        }
    }


    ROS_INFO("PCL: %d, %d\n", (int)copied_points.size(), 0);
/*
    ROS_INFO("FIELDS: %d, name1: %s, offset: %d, datatype: %d, count: %d", cloud_msg->fields.size(), 
              cloud_msg->fields.at(0).name, cloud_msg->fields.at(0).offset,
              cloud_msg->fields.at(0).datatype, cloud_msg->fields.at(0).count);
    ROS_INFO("FIELDS: %d, name1: %s, offset: %d, datatype: %d, count: %d", cloud_msg->fields.size(), 
              cloud_msg->fields.at(1).name, cloud_msg->fields.at(1).offset,
              cloud_msg->fields.at(1).datatype, cloud_msg->fields.at(1).count);
    ROS_INFO("FIELDS: %d, name1: %s, offset: %d, datatype: %d, count: %d", cloud_msg->fields.size(), 
              cloud_msg->fields.at(2).name, cloud_msg->fields.at(2).offset,
              cloud_msg->fields.at(2).datatype, cloud_msg->fields.at(2).count);
*/
    }
}

int main(int argc, char** argv)
{
  new_points_here = false;
  first_point = true;
  ros::init(argc, argv, "pc2cm_master");
  ros::NodeHandle node;

  ros::Rate rate(10);

  ros::Subscriber sub = node.subscribe("camera/points", 2, newPointsCallback);
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
       //float* point_el = curr_points->data();
       for (int i=0; i < copied_points.size();i++)
       {
          //point (0,0) should map to index[0][.5*MAP_WIDTH]
          //point (1,0) should map to index[1/CELL_WIDTH][.5*MAP_WIDTH/CELL_WIDTH]
          // point (1,1) should map to index(1/CELL_WIDTH][1/CELL_WIDTH + .5*MAP_WIDTH/CELL_WIDTH
          int x_index = (int)(copied_points.at(i).x/CELL_WIDTH );
          x_index = CLAMP(x_index, 0, MAP_LENGTH/CELL_WIDTH);

          int y_index = (int)(copied_points.at(i).y/CELL_WIDTH + (.5*MAP_WIDTH/CELL_WIDTH));
          y_index = CLAMP(y_index, 0, MAP_WIDTH/CELL_WIDTH);

          gridTotal[x_index][y_index] = (.98)*gridTotal[x_index][y_index] 
                                      + .02*(copied_points.at(i).z - gridTotal[x_index][y_index]);

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