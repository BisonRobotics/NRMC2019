#include <competition/competition_config.h>

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
Config::Config(ros::NodeHandle *nh) : rate(50.0)
{
  // Load launch file parameters
  double rate_float;
  loadParam(nh, "rate", rate_float, 50.0);
  rate = ros::Rate(rate_float);
  dt = rate.expectedCycleTime().toSec();

  // Load yaml parameters
  if(nh->hasParam("paths"))
  {
    ROS_INFO("Found paths parameter, attempting to load paths");
    XmlRpc::XmlRpcValue paths;
    nh->getParam("paths", paths);
    parsePath("dig_1", paths, &dig_path_1);
    parsePath("dig_2", paths, &dig_path_2);
    parsePath("hopper_1", paths, &hopper_path_1);
    parsePath("hopper_2", paths, &hopper_path_2);
    parsePath("final_position", paths, &final_position);
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
    parseTimer("finish_dig_1", timers, &finish_dig_1_time);
    parseTimer("finish_dig_2", timers, &finish_dig_2_time);
  }
  else
  {
    ROS_WARN("Couldn't find paths parameter");
  }

  finish = ros::Duration(60.0*10.0);
}

void Config::loadParam(ros::NodeHandle *nh, std::string name, std::string &param, std::string default_param)
{
  nh->param<std::string>(name, param, default_param);
  ROS_INFO("[tracker::Config::loadParam::%s]: %s = %s ", nh->getNamespace().c_str(), name.c_str(), param.c_str());
}

void Config::loadParam(ros::NodeHandle *nh, std::string name, int &param, int default_param)
{
  nh->param<int>(name, param, default_param);
  ROS_INFO("[tracker::Config::loadParam]: %s = %i ", name.c_str(), param);
}

void Config::loadParam(ros::NodeHandle *nh, std::string name, double &param, double default_param)
{
  nh->param<double>(name, param, default_param);
  ROS_INFO("[tracker::Config::loadParam]: %s = %f ", name.c_str(), param);
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