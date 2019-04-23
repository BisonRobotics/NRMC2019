#include <waypoint_control/waypoint_control_server.h>
#include <wheel_params/wheel_params.h>
#include <utilities/joy.h>


using namespace waypoint_control;


WaypointControlServer::WaypointControlServer(ros::NodeHandle *nh, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl) :
  nh(nh), fl(fl), fr(fr), bl(bl), br(br), debug(true), server(*nh, "action", false), direction(true),
  controller(fl, fr, br, bl), teleop(TeleopInterface::Mode::velocity, 0.4, fl, fr, br, bl),
  state(ControlState::manual), manual_safety(false), autonomy_safety(false), teleop_left(0.0f), teleop_right(0.0f), seq(0)
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

  nh->param<float>("max_velocity", max_velocity, 0.4);
  teleop.setMax(max_velocity);

  joy_subscriber = nh->subscribe("/joy", 1, &WaypointControlServer::joyCallback, this);
  joint_publisher = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
  server.registerGoalCallback(boost::bind(&WaypointControlServer::goalCallback, this));
  server.registerPreemptCallback(boost::bind(&WaypointControlServer::preemptCallback, this));
  server.start();
}

void WaypointControlServer::goalCallback()
{
  auto goal = server.acceptNewGoal();
  ControlState request = toControlState(*goal);

  ROS_INFO("[WaypointControlServer::goalCallback] Request for %s to %s",
           to_string(state).c_str(), to_string(request).c_str());
  switch (request)
  {
    case ControlState::new_goal:
    {
      // Only accepts one segment
      stop();
      direction = goal->path[0].direction_of_travel == 1;
      //controller.addPath(modified_path, direction);
      state = ControlState::in_progress;
      server.setSucceeded(toResult(state));
      break;
    }
    case ControlState::cancel:
    {
      stop();
      state = ControlState::ready;
      server.setSucceeded(toResult(state));
    }
    case ControlState::manual:
    {
      stop();
      state = ControlState::manual;
      server.setSucceeded(toResult(state));
      break;
    }
    default:
    {
      stop();
      state = ControlState::error;
      server.setAborted(toResult(state));
      ROS_ERROR("[WaypointControlServer::goalCallback] Unable to set control state from %s to %s",
                to_string(state).c_str(), to_string(request).c_str());
      break;
    }

  }
}

void WaypointControlServer::preemptCallback()
{
  server.setPreempted(toResult(state));
  ROS_INFO("[DigControlServer::preemptCallback] Preempting from %s", to_string(state).c_str());
}

void WaypointControlServer::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  using utilities::Joy;
  Joy joy(joy_msg);

  manual_safety = joy.get(Joy::MANUAL_SAFETY);
  autonomy_safety = joy.get(Joy::AUTONOMY_SAFETY);
  if (manual_safety)
  {
    teleop_left  = max_velocity * (float)joy.get(Joy::TELEOP_LEFT);
    teleop_right = max_velocity * (float)joy.get(Joy::TELEOP_RIGHT);
  }
  else
  {
    teleop_left = 0.0f;
    teleop_left = 0.0f;
  }
}

void WaypointControlServer::update(double dt)
{
  seq++;

  // Handle state
  switch (state)
  {
    case ControlState::error:
    {
      ROS_ERROR("[WaypointControlServer::update] In error state");
      stop();
      break;
    }
    case ControlState::ready:
    {
      // Do nothing
      stop();
      break;
    }
    case ControlState::in_progress:
    {
      if (autonomy_safety)
      {
        // TODO
      }
      else
      {
        stop();
      }
      break;
    }
    case ControlState::manual:
    {
      if (manual_safety)
      {
        teleop.update(teleop_left, teleop_right);
      }
      else
      {
        teleop.update(0.0f, 0.0f);
      }
      break;
    }
    case ControlState::new_goal:
    case ControlState::cancel:
    {
      ROS_ERROR("[WaypointControlServer::update] Shouldn't be in control state from %s", to_string(state).c_str());
      stop();
    }
  }

  // Visuals
  joint_angles.header.stamp = ros::Time::now();
  joint_angles.header.seq = seq;
  joint_angles.position[0] += fl->getRadialVelocity() * dt;
  joint_angles.position[1] += fr->getRadialVelocity() * dt;
  joint_angles.position[2] += br->getRadialVelocity() * dt;
  joint_angles.position[3] += bl->getRadialVelocity() * dt;
  joint_publisher.publish(joint_angles);
}

void WaypointControlServer::stop()
{

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "waypoint_control_server");
  ros::NodeHandle nh("~");

  iVescAccess *fl, *fr, *br, *bl;
  fl = new VescAccess(front_left_param);
  fr = new VescAccess(front_right_param);
  br = new VescAccess(back_right_param);
  bl = new VescAccess(back_left_param);
  WaypointControlServer server(&nh, fl, fr, br, bl);
  ros::Rate rate(50);
  double dt = rate.expectedCycleTime().toSec();
  while (ros::ok())
  {
    server.update(dt);
    ros::spinOnce();
    rate.sleep();
  }
  server.stop();
}
