#include <competition/competition_controller.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tracker/GetUInt.h>

using namespace competition;

Controller::Controller(ros::NodeHandle *nh, Config config) :
  nh(nh), config(config), tf_listener(tf_buffer), visuals(nh), state(ControlState::manual)
{
  joy_subscriber = nh->subscribe("/joy", 1, &Controller::joyCallback, this);
  debug_publisher = nh->advertise<Debug>("debug", 1);
  waypoint_client.setControlState(WaypointControlState::manual);
  dig_client.setControlState(DigControlState::manual);
  ROS_INFO("[Controller::Controller]: Online");

  ROS_INFO("[Controller::Controller]: Checking tracker");
  tracker::GetUInt request;
  if (ros::service::call("/tracker/left/get_brightness", request))
  {
    ROS_INFO("Received response of %i", request.response.value);
  }
  else
  {
    ROS_INFO("Unable to retrieve brightness");
  }
  if (ros::service::call("/tracker/left/get_exposure", request))
  {
    ROS_INFO("Received response of %i", request.response.value);
  }
  else
  {
    ROS_INFO("Unable to retrieve exposure");
  }

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
      start_time = ros::Time::now();
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
      visuals.clearWaypoints();
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
      visuals.updateWaypoints(config.digPath1());
      waypoint_client.setControlState(WaypointControlState::new_goal, config.digPath1());
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
        dig_client.setControlState(DigControlState::dig);
        // TMP
        /*state = ControlState::navigate_to_hopper_1;
        visuals.updateWaypoints(config.hopperPath1());
        waypoint_client.setControlState(WaypointControlState::new_goal, config.hopperPath1());*/
        // TMP
      }
      break;
    }
    case ControlState::dig_1:
    {
      if (ros::Time::now() - start_time >= config.finishDig1Time())
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
        ros::Duration(5).sleep();
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::navigate_to_hopper_1).c_str());
        state = ControlState::navigate_to_hopper_1;
        visuals.updateWaypoints(config.hopperPath1());
        waypoint_client.setControlState(WaypointControlState::new_goal, config.hopperPath1());
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
        visuals.updateWaypoints(config.digPath2());
        waypoint_client.setControlState(WaypointControlState::new_goal, config.digPath2());
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
      if (ros::Time::now() - start_time >= config.finishDig2Time())
      {
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::finish_dig_2).c_str());
        state = ControlState::finish_dig_2;
        dig_client.setControlState(DigControlState::finish_dig);
      }
      break;
    }
    case ControlState::finish_dig_2:
    {
      if (dig_client.getControlState() == DigControlState::ready)
      {
        ros::Duration(5).sleep();
        ROS_INFO("[Controller::update]: %s to %s",
                 to_string(state).c_str(),
                 to_string(ControlState::navigate_to_hopper_2).c_str());
        state = ControlState::navigate_to_hopper_2;
        visuals.updateWaypoints(config.hopperPath2());
        waypoint_client.setControlState(WaypointControlState::new_goal, config.hopperPath2());
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
        visuals.updateWaypoints(config.finalPosition());
        waypoint_client.setControlState(WaypointControlState::new_goal, config.finalPosition());
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

  ros::Duration difference = ros::Time::now() - start_time;
  int64_t minutes = difference.sec / 60;
  int64_t seconds = difference.sec - minutes * 60;
  debug.header.seq++;
  debug.header.stamp = ros::Time::now();
  debug.competition_timer.m = minutes;
  debug.competition_timer.s = seconds;
  debug.competition_state = to_string(state);
  debug.waypoint_state = waypoint_control::to_string(waypoint_client.getControlState());
  debug.dig_state = dig_control::to_string(dig_client.getControlState());
  debug_publisher.publish(debug);

}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  this->joy = joy_msg;
  if (joy.get(Joy::MANUAL_SAFETY))
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
  else if (joy.get(Joy::AUTONOMY_SAFETY) || config.fullAutonomy())
  {
    if (joy.get(Joy::AUTONOMY_SAFETY))
    {
      ROS_INFO("[Controller::joyCallback::AUTONOMY_SAFETY]");
    }
    if (joy.get(Joy::START_COMPETITION))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(state).c_str(),
               to_string(ControlState::start).c_str());
      state = ControlState::start;
    }
    else if (joy.get(Joy::STOP_COMPETITION))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(state).c_str(),
               to_string(ControlState::manual).c_str());
      dig_client.setControlState(DigControlState::manual);
      waypoint_client.setControlState(WaypointControlState::manual);
      state = ControlState::manual;
    }
    else if (joy.get(Joy::DIG))
    {
      DigControlState current = dig_client.getControlState();
      if (current == DigControlState::ready)
      {
        ROS_INFO("[Controller::joyCallback]: %s to %s",
                 to_string(current).c_str(),
                 to_string(DigControlState::dig).c_str());
        dig_client.setControlState(DigControlState::dig);
      }
      else if (current == DigControlState::dig)
      {
        ROS_INFO("[Controller::joyCallback]: %s to %s",
                 to_string(current).c_str(),
                 to_string(DigControlState::finish_dig).c_str());
        dig_client.setControlState(DigControlState::finish_dig);
      }
      else
      {
        ROS_WARN("[Controller::joyCallback]: No dig transition from %s", to_string(current).c_str());
      }

    }
    else if (joy.get(Joy::DUMP))
    {
      DigControlState current = dig_client.getControlState();
      if (current == DigControlState::ready)
      {
        ROS_INFO("[Controller::joyCallback]: %s to %s",
                 to_string(current).c_str(),
                 to_string(DigControlState::dump).c_str());
        dig_client.setControlState(DigControlState::dump);
      }
      else
      {
        ROS_WARN("[Controller::joyCallback]: No dump transition from %s", to_string(current).c_str());
      }
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
}
