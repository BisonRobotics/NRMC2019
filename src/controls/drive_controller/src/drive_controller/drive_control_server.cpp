#include <drive_controller/drive_control_server.h>
#include <wheel_params/wheel_params.h>


using namespace drive_controller;


DriveControlServer::DriveControlServer(ros::NodeHandle *nh, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl) :
  nh(nh), fl(fl), fr(fr), bl(bl), br(br), debug(true), server(*nh, "action", false),
  controller(fr, fl, bl, br), teleop(TeleopInterface::Mode::velocity, 0.4, fl, fr, br, bl),
  state(ControlState::manual), safety(false), teleop_left(0.0f), teleop_right(0.0f), seq(0)
{
  joint_angles.header.stamp = ros::Time::now();
  joint_angles.header.seq = seq;
  joint_angles.name.emplace_back("front_left_wheel");
  joint_angles.name.emplace_back("front_right_wheel");
  joint_angles.name.emplace_back("back_right_wheel");
  joint_angles.name.emplace_back("back_left_wheel");
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);

  nh->param<float>("max_velocity", max_velocity, 0.4);
  teleop.setMax(max_velocity);

  joy_subscriber = nh->subscribe("/joy", 1, &DriveControlServer::joyCallback, this);
  joint_publisher = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
  server.registerGoalCallback(boost::bind(&DriveControlServer::goalCallback, this));
  server.registerPreemptCallback(boost::bind(&DriveControlServer::preemptCallback, this));
  server.start();
}

DriveControlResult drive_controller::toResult(ControlState state)
{
  DriveControlResult result;
  result.control_state = (DriveControlResult::_control_state_type)state;
  return result;
}

ControlState drive_controller::toControlState(DriveControlGoal goal)
{
  return (ControlState)goal.control_state;
}

void DriveControlServer::goalCallback()
{
  auto goal = server.acceptNewGoal();
  ControlState request = toControlState(*goal);

  ROS_INFO("[DigControlServer::goalCallback] Request for %s to %s",
           to_string(state).c_str(), to_string(request).c_str());
  switch (request)
  {
    case ControlState::new_goal:
    {
      // TODO actually do something
      state = ControlState::in_progress;
      server.setSucceeded(toResult(state));
      break;
    }
    case ControlState::cancel:
    {
      state = ControlState::ready;
      server.setSucceeded(toResult(state));
      stop();
    }
    case ControlState::manual:
    {
      state = ControlState::manual;
      server.setSucceeded(toResult(state));
      break;
    }
    default:
    {
      state = ControlState::error;
      stop();
      server.setAborted(toResult(state));
      ROS_ERROR("[DriveControlServer::goalCallback] Unable to set control state from %s to %s",
                to_string(state).c_str(), to_string(request).c_str());
      break;
    }

  }
}

void DriveControlServer::preemptCallback()
{
  server.setPreempted(toResult(state));
  ROS_INFO("[DigControlServer::preemptCallback] Preempting from %s", to_string(state).c_str());
}

void DriveControlServer::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  bool lb = joy->buttons[4] == 1; // Safety
  float ls = joy->axes[1];        // Left wheels
  float rs = joy->axes[3];        // Right wheels

  safety = lb;

  if (safety)
  {
    teleop_left  = max_velocity * ls;
    teleop_right = max_velocity * rs;
  }
  else
  {
    teleop_left = 0.0f;
    teleop_left = 0.0f;
  }
}

void DriveControlServer::update(double dt)
{
  seq++;

  // Handle state
  switch (state)
  {
    case ControlState::error:
    {
      ROS_ERROR("[DriveControlServer::update] In error state");
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
      if (safety)
      {

      }
      else
      {

      }
      break;
    }
    case ControlState::manual:
    {
      if (safety)
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
      ROS_ERROR("[DriveControlServer::update] Shouldn't be in control state from %s", to_string(state).c_str());
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

void DriveControlServer::stop()
{
  controller.haltAndAbort();
}

std::string drive_controller::to_string(ControlState state)
{
  switch (state)
  {
    case ControlState::error:
      return "error";
    case ControlState::ready:
      return "ready";
    case ControlState::new_goal:
      return "new_goal";
    case ControlState::in_progress:
      return "in_progress";
    case ControlState::cancel:
      return "cancel";
    case ControlState::manual:
      return "manual";
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "drive_control_server");
  ros::NodeHandle nh("~");

  iVescAccess *fl, *fr, *br, *bl;
  fl = new VescAccess(front_left_param);
  fr = new VescAccess(front_right_param);
  br = new VescAccess(back_right_param);
  bl = new VescAccess(back_left_param);
  DriveControlServer server(&nh, fl, fr, br, bl);
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
