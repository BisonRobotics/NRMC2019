#include <drive_controller/drive_control_server.h>
#include <wheel_params/wheel_params.h>


using namespace drive_controller;


DriveControlServer::DriveControlServer(ros::NodeHandle *nh, DriveController *controller, TeleopInterface *teleop) :
  nh(nh), controller(controller), teleop(teleop), debug(true), server(*nh, "action", false),
  state(ControlState::ready), safety(false), teleop_left(0.0f), teleop_right(0.0f), seq(0)
{
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
    teleop_left  = MAX_WHEEL_VELOCITY * ls;
    teleop_right = MAX_WHEEL_VELOCITY * rs;
  }
  else
  {
    teleop_left = 0.0f;
    teleop_left = 0.0f;
  }
}

void DriveControlServer::update()
{
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
        teleop->update(teleop_left, teleop_right);
      }
      else
      {
        teleop->update(0.0f, 0.0f);
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
}

void DriveControlServer::stop()
{
  controller->haltAndAbort();
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
  iVescAccess *fr, *fl, *br, *bl;
  fr = new VescAccess(front_right_param);
  fl = new VescAccess(front_left_param);
  br = new VescAccess(back_right_param);
  bl = new VescAccess(back_left_param);
  DriveController controller(fr, fl, bl, br);
  TeleopInterface teleop(TeleopInterface::Mode::velocity, 0.5, fl, fr, br, bl);
  DriveControlServer server(&nh, &controller, &teleop);
  ros::Rate rate(50);
  while (ros::ok())
  {
    server.update();
    ros::spinOnce();
    rate.sleep();
  }
  server.stop();
}
