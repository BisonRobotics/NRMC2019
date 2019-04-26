#ifndef DRIVE_CONTROLLER_DRIVE_CONTROL_SERVER_H
#define DRIVE_CONTROLLER_DRIVE_CONTROL_SERVER_H

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/Joy.h>
#include <drive_controller/drive_controller.h>
#include <navigation_msgs/FollowPathAction.h>
#include <drive_controller/DriveControlAction.h>
#include <teleop_interface/teleop_interface.h>
#include <localization/StateVector.h>
#include <drive_controller/drive_control_states.h>

namespace drive_controller
{
  using navigation_msgs::BezierSegment;
  typedef tf2::Stamped<tf2::Transform> StampedTransform;

  bezier_path toBezierPath(const navigation_msgs::BezierSegment &segment);

  class DriveControlServer
  {
  public:

    DriveControlServer(ros::NodeHandle *nh, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl);

    void goalCallback();
    void preemptCallback();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void stateVectorCallback(const localization::StateVector::ConstPtr &state_vector_msg);
    void update(double dt);
    void stop();

  private:
    bool debug, manual_safety, autonomy_safety, direction; // forward = true
    uint32_t seq;
    float teleop_left, teleop_right, max_velocity;
    double x, y, theta;
    bezier_path path, modified_path;
    LocalizerInterface::StateVector state_vector;

    ControlState state;
    iVescAccess *fl, *fr, *br, *bl;
    TeleopInterface teleop;
    DriveController controller;

    ros::NodeHandle *nh;
    ros::Subscriber joy_subscriber;
    ros::Subscriber state_subscriber;
    ros::Publisher debug_publisher;
    ros::Publisher joint_publisher;
    actionlib::SimpleActionServer<DriveControlAction> server;
    sensor_msgs::JointState joint_angles;

  };
}


#endif //DRIVE_CONTROLLER_DRIVE_CONTROL_SERVER_H
