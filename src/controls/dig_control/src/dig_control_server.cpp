#include <dig_control/dig_control_server.h>


using namespace dig_control;


DigControlServer::DigControlServer(ros::NodeHandle *nh, DigController *controller) :
  nh(nh), controller(controller),
  server(*nh, "dig_control", boost::bind(&DigControlServer::goalCallback, this, _1), false)
{
  //server.registerPreemptCallback(boost::bind(&DigControlServer::preemptCallback, this));
  server.start();
}

void DigControlServer::goalCallback(const actionlib::SimpleActionServer<DigControlAction>::GoalConstPtr &goal)
{
  ROS_INFO("[digControlGoalCallback]");
  ControlState request = toControlState(*goal);
  ControlState current_state = controller->getControlState();

  ROS_INFO("[DigControlServer::goalCallback] Request to change control state from %s to %s",
            to_string(current_state).c_str(), to_string(request).c_str());
  switch (request)
  {
    case ControlState::manual:
    {
      // Set to manual regardless of current state
      controller->setControlState(request);
      controller->stop();
      server.setSucceeded(toResult(request));
      break;
    }
    case ControlState::dig:
    case ControlState::dump:
    {
      // Make sure controller is ready for new command
      if (current_state == ControlState::ready || current_state == ControlState::manual)
      {
        controller->setControlState(request);
        server.acceptNewGoal();
      }
      else
      {
        server.setAborted(toResult(current_state));
        controller->setControlState(ControlState::error);
        controller->stop();
        ROS_ERROR("[DigControlServer::goalCallback] Unable to set control state from %s to %s",
                  to_string(current_state).c_str(), to_string(request).c_str());
      }
      break;
    }
    case ControlState::finish_dig:
    {
      if (current_state == ControlState::dig)
      {
        controller->setControlState(request);
        server.acceptNewGoal();
      }
      else
      {
        server.setAborted(toResult(current_state));
        controller->setControlState(ControlState::error);
        controller->stop();
        ROS_ERROR("[DigControlServer::goalCallback] Unable to set control state from %s to %s",
                  to_string(current_state).c_str(), to_string(request).c_str());
      }
      break;
    }
    case ControlState::finish_dump:
    {
      if (current_state == ControlState::dump)
      {
        controller->setControlState(request);
        server.acceptNewGoal();
      }
      else
      {
        server.setAborted(toResult(current_state));
        controller->setControlState(ControlState::error);
        controller->stop();
        ROS_ERROR("[DigControlServer::goalCallback] Unable to set control state from %s to %s",
                  to_string(current_state).c_str(), to_string(request).c_str());
      }
      break;
    }
    default:
    {
      server.setAborted(toResult(current_state));
      controller->setControlState(ControlState::error);
      controller->stop();
      ROS_ERROR("[DigControlServer::goalCallback] Unable to set control state from %s to %s",
                to_string(current_state).c_str(), to_string(request).c_str());
    }
  }
}

void DigControlServer::preemptCallback()
{
  ROS_INFO("[digControlPreemptCallback]");
}

DigControlResult DigControlServer::toResult(ControlState state)
{
  DigControlResult result;
  result.control_state = (DigControlResult::_control_state_type)state;
  return result;
}

ControlState DigControlServer::toControlState(DigControlGoal goal)
{
  return (ControlState)goal.control_state;
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tracker_node");
  ros::NodeHandle nh;
  DigController controller;
  DigControlServer server(&nh, &controller);
  ros::Rate rate(50);
  while (ros::ok())
  {
    controller.update();
    ros::spinOnce();
    rate.sleep();
  }
  controller.stop();
}


