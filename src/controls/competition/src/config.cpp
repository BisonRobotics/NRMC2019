#include <competition/config.h>

using namespace competition;

using XmlRpc::XmlRpcValue;
using waypoint_control::Waypoint;

void parsePath(const std::string &path_name, XmlRpc::XmlRpcValue &xml_paths, Waypoints *waypoints)
{
  ROS_ASSERT(xml_paths.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  XmlRpc::XmlRpcValue xml_path = xml_paths[path_name];
  ROS_ASSERT(xml_path.getType() == XmlRpc::XmlRpcValue::TypeArray);
  Waypoint waypoint;
  waypoint.header.seq = 0;
  for (int i = 0; i < xml_path.size(); i++)
  {
    waypoint.header.seq++;
    waypoint.pose.x = xml_path[i]["x"];
    waypoint.pose.y = xml_path[i]["y"];
    waypoint.pose.theta = xml_path[i]["theta"];
    waypoint.reverse = (uint8_t)(bool)xml_path[i]["reverse"];
    waypoint.match_orientation =  (uint8_t)(bool)xml_path[i]["match_orientation"];
    waypoint.drive_profile = (uint8_t)(int)xml_path[i]["drive_profile"];
    waypoints->emplace_back(waypoint);
    ROS_INFO("[competiton:config::parsePath::%s] %f, %f, %f, %i, %i, %i",
        path_name.c_str(), waypoint.pose.x, waypoint.pose.y, waypoint.pose.theta,
        (int)waypoint.reverse, (int)waypoint.match_orientation, (int)waypoint.drive_profile);
  }
}

void parseTimer(const std::string &timer_name, XmlRpc::XmlRpcValue &xml_timers, ros::Duration *timer)
{
  ROS_ASSERT(xml_timers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  XmlRpc::XmlRpcValue xml_timer = xml_timers[timer_name];
  ROS_ASSERT(xml_timer.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  (*timer) = ros::Duration(60.0*(double)xml_timer["m"] + (double)xml_timer["s"]);
  int minutes = timer->sec / 60;
  ROS_INFO("[competiton:config::parseTimer::%s] m: %i, s: %i",
      timer_name.c_str(), minutes, timer->sec - 60 * minutes);
}

// Based on: https://answers.ros.org/question/189299/getting-hierarchy-level-of-yaml-parameter/
Config::Config(ros::NodeHandle *nh) : utilities::Config("competition"), rate_(50.0)
{
  // Load launch file parameters
  double rate_float;
  loadParam(nh, "rate", rate_float, 50.0);
  rate_ = ros::Rate(rate_float);
  dt_ = rate_.expectedCycleTime().toSec();
  loadParam(nh, "full_autonomy", full_autonomy_, false);

  // Load yaml parameters
  if(nh->hasParam("paths"))
  {
    ROS_INFO("Found paths parameter, attempting to load paths");
    XmlRpc::XmlRpcValue paths;
    nh->getParam("paths", paths);
    parsePath("dig_1", paths, &dig_path_1_);
    parsePath("dig_2", paths, &dig_path_2_);
    parsePath("hopper_1", paths, &hopper_path_1_);
    parsePath("hopper_2", paths, &hopper_path_2_);
    parsePath("final_position", paths, &final_position_);
  }
  else
  {
    ROS_WARN("Couldn't find paths parameter");
  }

  if(nh->hasParam("timers"))
  {
    ROS_INFO("Found timers parameter, attempting to load timers");

    XmlRpc::XmlRpcValue timers;
    nh->getParam("timers", timers);
    parseTimer("finish_dig_1", timers, &finish_dig_1_time_);
    parseTimer("finish_dig_2", timers, &finish_dig_2_time_);
  }
  else
  {
    ROS_WARN("Couldn't find timers parameter");
  }

  finish_time_ = ros::Duration(60.0*10.0);
}

const ros::Rate &Config::rate()
{
  return rate_;
}

const double &Config::dt()
{
  return dt_;
}

const bool &Config::fullAutonomy()
{
  return full_autonomy_;
}

const ros::Duration &Config::finishDig1Time()
{
  return finish_dig_1_time_;
}

const ros::Duration &Config::finishDig2Time()
{
  return finish_dig_2_time_;
}

const ros::Duration &Config::finishTime()
{
  return finish_time_;
}

const Waypoints &Config::digPath1()
{
  return dig_path_1_;
}

const Waypoints &Config::digPath2()
{
  return dig_path_2_;
}

const Waypoints &Config::hopperPath1()
{
  return hopper_path_1_;
}

const Waypoints &Config::hopperPath2()
{
  return hopper_path_2_;
}

const Waypoints &Config::finalPosition()
{
  return final_position_;
}

std::string competition::to_string(competition::ControlState state)
{
  switch (state)
  {
    case ControlState::manual:
      return "manual";
    case ControlState::assisted_autonomy:
      return "assisted_autonomy";
    case ControlState::wait_for_start:
      return "wait_for_start";
    case ControlState::start:
      return "start";
    case ControlState::check_for_apriltag:
      return "check_for_apriltag";
    case ControlState::wait_for_localization:
      return "wait_for_localization";
    case ControlState::navigate_to_dig_zone_1:
      return "navigate_to_dig_zone_1";
    case ControlState::dig_1:
      return "dig_1";
    case ControlState::finish_dig_1:
      return "finish_dig_1";
    case ControlState::navigate_to_hopper_1:
      return  "navigate_to_hopper_1";
    case ControlState::dump_1:
      return "dump_1";
    case ControlState::navigate_to_dig_zone_2:
      return "navigate_to_dig_zone_2";
    case ControlState::dig_2:
      return "dig_2";
    case ControlState::finish_dig_2:
      return "finish_dig_2";
    case ControlState::navigate_to_hopper_2:
      return "navigate_to_hopper_2";
    case ControlState::dump_2:
      return "dump_2";
    case ControlState::navigate_to_final_position:
      return "navigate_to_final_position";
    case ControlState::finished:
      return "finished";
  }
}