#include <competition/competition_config.h>
#include <XmlRpcValue.h>

using namespace competition;

// Based on: https://answers.ros.org/question/189299/getting-hierarchy-level-of-yaml-parameter/
Config::Config(ros::NodeHandle *nh)
{
  using XmlRpc::XmlRpcValue;

  std::vector<double> dig_zone_path_default;
  std::vector<double> dig_zone_path_1;
  if(nh->hasParam("/dig_zone_path_1"))
  {
    ROS_INFO("Found dig_zone_path_1");

    XmlRpc::XmlRpcValue dig_zone_path;
    nh->getParam("/dig_zone_path_1", dig_zone_path);
    ROS_ASSERT(dig_zone_path.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < dig_zone_path.size(); ++i)
    {
      double x = dig_zone_path[i]["x"];
      double y = dig_zone_path[i]["y"];
      double theta = dig_zone_path[i]["theta"];
      std::string direction = dig_zone_path[i]["direction"];
      bool match_orientation = dig_zone_path[i]["match_orientation"];
      ROS_INFO("%f, %f, %f, %s, %i", x, y, theta, direction.c_str(), match_orientation);
    }
    /*for(XmlRpc::XmlRpcValue::ValueArray::const_iterator it = dig_zone_path.begin(); it != dig_zone_path.end(); ++it)
    {
      double x = dig_zone_path[it->first]["x"];
      double y = dig_zone_path[it->first]["y"];
      double theta = dig_zone_path[it->first]["theta"];
      ROS_INFO("%f, %f, %f", x, y, theta);
    }*/
    // nh->getParam("dig_zone_path_1", dig_zone_path_1);
    //ROS_INFO("Values %f, %f", dig_zone_path_1[0], dig_zone_path_1[1]);
  }
  else
  {
    ROS_WARN("Couldn't find dig_zone_path_1");
  }

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
    case ControlState::finish:
      return "finish";
  }
}