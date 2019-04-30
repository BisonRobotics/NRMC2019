#include <waypoint_control/waypoint_control_server.h>
#include <wheel_params/wheel_params.h>
#include <utilities/joy.h>
#include <utilities/utilities.h>
#include <vector>
#include <iterator>
#include <algorithm>
#include <waypoint_control/Debug.h>

using namespace waypoint_control;


WaypointControlServer::WaypointControlServer(ros::NodeHandle *nh, Config *config,
    iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl) :
  nh(nh), tf_listener(tf_buffer), fl(fl), fr(fr), bl(bl), br(br), server(*nh, "action", false),
  config(config), controller(*config, fl, fr, br, bl),
  debug(true), manual_safety(false), autonomy_safety(false),
  dt(config->dt()), teleop_left(0.0), teleop_right(0.0), seq(0)
{
  joint_angles.header.stamp = ros::Time::now();
  joint_angles.header.seq = seq;
  joint_angles.name.emplace_back("front_left_wheel_joint");
  joint_angles.name.emplace_back("front_right_wheel_joint");
  joint_angles.name.emplace_back("back_right_wheel_joint");
  joint_angles.name.emplace_back("back_left_wheel_joint");
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);

  joy_subscriber = nh->subscribe("/joy", 1, &WaypointControlServer::joyCallback, this);
  joint_publisher = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
  debug_publisher = nh->advertise<waypoint_control::Debug>("debug", 1);
  server.registerGoalCallback(boost::bind(&WaypointControlServer::goalCallback, this));
  server.registerPreemptCallback(boost::bind(&WaypointControlServer::preemptCallback, this));
  server.start();
  ROS_INFO("[WaypointControlServer::WaypointControlServer]: Online");
}

void WaypointControlServer::goalCallback()
{
  auto goal = server.acceptNewGoal();
  ControlState request = toControlState(*goal);

  ROS_INFO("[WaypointControlServer::goalCallback] Request for %s to %s",
           to_string(controller.getControlState()).c_str(),
           to_string(request).c_str());
  if (request == ControlState::new_goal)
  {
    waypoints = goal->waypoints;
    std::reverse(std::begin(waypoints), std::end(waypoints)); // Easier to pop elements from the back
    controller.setControlState(request, waypoints);
  }
  else
  {
    controller.setControlState(request);
  }
}

void WaypointControlServer::preemptCallback()
{
  server.setPreempted(toResult(controller.getControlState()));
  ROS_INFO("[DigControlServer::preemptCallback] Preempting from %s",
      to_string(controller.getControlState()).c_str());
}

void WaypointControlServer::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  using utilities::Joy;
  Joy joy(joy_msg);

  manual_safety = joy.get(Joy::MANUAL_SAFETY);
  autonomy_safety = joy.get(Joy::AUTONOMY_SAFETY);
  if (manual_safety)
  {
    teleop_left  = config->maxManualDuty() * joy.get(Joy::TELEOP_LEFT);
    teleop_right = config->maxManualDuty() * joy.get(Joy::TELEOP_RIGHT);
  }
  else
  {
    teleop_left = 0.0;
    teleop_left = 0.0;
  }
}

void WaypointControlServer::update()
{

  // Update controller
  geometry_msgs::TransformStamped transform_msg;
  Pose2D pose;
  try
  {
    transform_msg = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
    pose = utilities::toPose2D(transform_msg);
    controller.update(pose, manual_safety, autonomy_safety || config->fullAutonomy(), teleop_left, teleop_right);
    ControlState state = controller.getControlState();

    switch (state)
    {
      case ControlState::error:
      {
        WaypointControlResult result;
        result.control_state = (uint)ControlState::error;
        server.setAborted(result);
        controller.setControlState(ControlState::manual);
      }
      case ControlState::in_progress:
      {
        WaypointControlFeedback feedback;
        feedback.control_state = (uint)ControlState::in_progress;
        feedback.angular_deviation = 0.0;
        feedback.linear_deviation = 0.0;
        feedback.progress = ((double)controller.remainingWaypoints()) / waypoints.size();
        server.publishFeedback(feedback);
        break;
      }
      case ControlState::finished:
      {
        WaypointControlResult result;
        result.progress = 1.0;
        result.angular_deviation = 0.0;
        result.linear_deviation = 0.0;
        result.control_state = (uint)ControlState::finished;
        server.setSucceeded(result);
        controller.setControlState(ControlState::manual);
        break;
      }
      case ControlState::ready:
      case ControlState::new_goal:
      case ControlState::cancel:
      case ControlState::manual:
      {
        // Do nothing
        break;
      }
    }
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("[WaypointControlServer::update]: %s",ex.what());
    ros::Duration(1.0).sleep();
    controller.stop();
  }

  // Visuals
  joint_angles.header.stamp = ros::Time::now();
  joint_angles.header.seq = seq++;
  joint_angles.position[0] += fl->getRadialVelocity() * dt;
  joint_angles.position[1] += fr->getRadialVelocity() * dt;
  joint_angles.position[2] += br->getRadialVelocity() * dt;
  joint_angles.position[3] += bl->getRadialVelocity() * dt;
  joint_publisher.publish(joint_angles);

  // Debug info
  debug_publisher.publish(controller.getDebugInfo());
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "waypoint_control_server");
  ros::NodeHandle nh("~");
  Config config(&nh);
  ros::Rate rate(config.rate());

  iVescAccess *fl, *fr, *br, *bl;
  fl = new VescAccess(front_left_param);
  fr = new VescAccess(front_right_param);
  br = new VescAccess(back_right_param);
  bl = new VescAccess(back_left_param);
  WaypointControlServer server(&nh, &config, fl, fr, br, bl);

  while (ros::ok())
  {
    server.update();
    ros::spinOnce();
    rate.sleep();
  }
}
