#include <competition/competition_controller.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace competition;

Controller::Controller(ros::NodeHandle *nh, Config *config) :
  nh(nh), config(config), tf_listener(tf_buffer), visuals(nh), state(ControlState::manual)
{
  joy_subscriber = nh->subscribe("joy", 1, &Controller::joyCallback, this);
  waypoint_client.setControlState(WaypointControlState::manual);
  dig_client.setControlState(DigControlState::manual);
  ROS_INFO("[Controller::Controller]: Online");
}

void Controller::update()
{
  // Get latest TF
  try
  {
    tf2::fromMsg(tf_buffer.lookupTransform("map", "base_link", ros::Time(0)), transform);
    visuals.update(transform);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  switch (state)
  {
    case ControlState::manual:
    case ControlState::assisted_autonomy:
    {
      // Controllers more or less handle themselves
      break;
    }
    case ControlState::wait_for_start:
    {
      // Make sure we're connected to all systems, wait for start command
      break;
    }
    case ControlState::start:
    {
      // Make sure systems are in starting positions
      start_time = ros::Time::now();
      state = ControlState::check_for_apriltag;
      break;
    }
    case ControlState::check_for_apriltag:
    {
      // Check to make sure we have received an apriltag, if not rotate in place
      ROS_INFO("[Controller::update]: %s to %s",
          to_string(state).c_str(),
          to_string(ControlState::wait_for_localization).c_str());
      state = ControlState::wait_for_localization;
      break;
    }
    case ControlState::wait_for_localization:
    {
      // Wait until localization stabalizes
      ROS_INFO("[Controller::update]: %s to %s",
               to_string(state).c_str(),
               to_string(ControlState::navigate_to_dig_zone_1).c_str());
      state = ControlState::navigate_to_dig_zone_1;
      waypoint_client.setControlState(WaypointControlState::new_goal, config->dig_path_1);
      break;
    }
    case ControlState::navigate_to_dig_zone_1:
    {
      if (waypoint_client.getControlState() == WaypointControlState::finished)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::dig_1).c_str());
        state = ControlState::dig_1;
        waypoint_client.setControlState(WaypointControlState::ready);
        dig_client.setControlState(DigControlState::dig);
      }
      break;
    }
    case ControlState::dig_1:
    {
      if (ros::Time::now() - start_time >= config->finish_dig_1_time)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::finish_dig_1).c_str());
        state = ControlState::finish_dig_1;
        dig_client.setControlState(DigControlState::finish_dig);
      }
      break;
    }
    case ControlState::finish_dig_1:
    {
      if (dig_client.getControlState() == DigControlState::ready)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::finish_dig_1).c_str());
        state = ControlState::navigate_to_hopper_1;
        waypoint_client.setControlState(WaypointControlState::new_goal, config->hopper_path_1);
      }
      break;
    }
    case ControlState::navigate_to_hopper_1:
    {
      if (waypoint_client.getControlState() == WaypointControlState::finished)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::dump_1).c_str());
        state = ControlState::dump_1;
        dig_client.setControlState(DigControlState::dump);
      }
      break;
    }
    case ControlState::dump_1:
    {
      if (dig_client.getControlState() == DigControlState::ready)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::navigate_to_dig_zone_2).c_str());
        state = ControlState::navigate_to_dig_zone_2;
        waypoint_client.setControlState(WaypointControlState::new_goal, config->dig_path_2);
      }
      break;
    }
    case ControlState::navigate_to_dig_zone_2:
    {
      if (waypoint_client.getControlState() == WaypointControlState::finished)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::dig_2).c_str());
        state = ControlState::dig_2;
        dig_client.setControlState(DigControlState::dig);
      }
      break;
    }
    case ControlState::dig_2:
    {
      if (ros::Time::now() - start_time >= config->finish_dig_2_time)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::finish_dig_2).c_str());
        state = ControlState::finish_dig_1;
        dig_client.setControlState(DigControlState::finish_dig);
      }
      break;
    }
    case ControlState::finish_dig_2:
    {
      if (dig_client.getControlState() == DigControlState::ready)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::navigate_to_hopper_2).c_str());
        state = ControlState::navigate_to_hopper_2;
        waypoint_client.setControlState(WaypointControlState::new_goal, config->hopper_path_2);
      }
      break;
    }
    case ControlState::navigate_to_hopper_2:
    {
      if (waypoint_client.getControlState() == WaypointControlState::finished)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::dump_2).c_str());
        state = ControlState::dump_2;
        dig_client.setControlState(DigControlState::dump);
      }
      break;
    }
    case ControlState::dump_2:
    {
      if (dig_client.getControlState() == DigControlState::ready)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::finished).c_str());
        state = ControlState::finished;
        waypoint_client.setControlState(WaypointControlState::new_goal, config->final_position);
      }
      break;
    }
    case ControlState::navigate_to_final_position:
    {
      if (waypoint_client.getControlState() == WaypointControlState::finished)
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::finished).c_str());
        state = ControlState::finished;
      }
      break;
    }
    case ControlState::finished:
    {
      break;
    }
  }

}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  this->joy = joy_msg;
  
  if (joy.get(Joy::AUTONOMY_SAFETY))
  {
    if (joy.get(Joy::START_DIG))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
          to_string(dig_client.getControlState()).c_str(),
          to_string(DigControlState::dig).c_str());
      dig_client.setControlState(DigControlState::dig);
    }
    else if (joy.get(Joy::END_DIG))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(dig_client.getControlState()).c_str(),
               to_string(DigControlState::finish_dig).c_str());
      dig_client.setControlState(DigControlState::finish_dig);
    }
    else if (joy.get(Joy::START_DUMP))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(dig_client.getControlState()).c_str(),
               to_string(DigControlState::dump).c_str());
      dig_client.setControlState(DigControlState::dump);
    }
    else if (joy.get(Joy::END_DUMP))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(dig_client.getControlState()).c_str(),
               to_string(DigControlState::finish_dump).c_str());
      dig_client.setControlState(DigControlState::finish_dump);
    }
    else if (joy.get(Joy::START_PATH))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(waypoint_client.getControlState()).c_str(),
               to_string(WaypointControlState::new_goal).c_str());
      active_waypoints = visuals.getWaypoints();
      waypoint_client.setControlState(WaypointControlState::new_goal, active_waypoints);
      visuals.followRobot(false);
    }
    else if (joy.get(Joy::CLEAR_WAYPOINTS))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(dig_client.getControlState()).c_str(),
               to_string(WaypointControlState::cancel).c_str());
      waypoint_client.setControlState(WaypointControlState::cancel);
      visuals.clearWaypoints();
    }
    else if (joy.get(Joy::FOLLOW_ROBOT))
    {
      ROS_INFO("[Controller::joyCallback]: Enabling follow_robot visual");
      visuals.followRobot(true);
    }
    else if (joy.get(Joy::UNFOLLOW_ROBOT))
    {
      ROS_INFO("[Controller::joyCallback]: Disabling follow_robot visual");
      visuals.followRobot(false);
    }
    else if (joy.get(Joy::FORWARD))
    {
      ROS_INFO("[Controller::joyCallback]: Setting direction of travel to forward");
      visuals.setReverse(false);
    }
    else if (joy.get(Joy::REVERSE))
    {
      ROS_INFO("[Controller::joyCallback]: Setting direction of travel to reverse");
      visuals.setReverse(true);
    }
  }
  else
  {
    if (dig_client.getControlState() != DigControlState::manual)
    {
      ROS_INFO("[Controller::joyCallback]: Dig client %s to %s",
               to_string(dig_client.getControlState()).c_str(),
               to_string(DigControlState::manual).c_str());
       dig_client.setControlState(DigControlState::manual);
    }
    if (waypoint_client.getControlState() != WaypointControlState::manual)
    {
      ROS_INFO("[Controller::joyCallback]: Waypoint client %s to %s",
               to_string(waypoint_client.getControlState()).c_str(),
               to_string(WaypointControlState::manual).c_str());
      waypoint_client.setControlState(WaypointControlState::manual);
    }
  }
}
