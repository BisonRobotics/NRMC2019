#include <dig_control/dig_control_server.h>


using namespace dig_control;


DigControlServer::DigControlServer(ros::NodeHandle *nh) :
  dig_server(*nh, "dig_server", boost::bind(&DigControlServer::digCallback, this, _1), false),
  dump_server(*nh, "dump_server", boost::bind(&DigControlServer::dumpCallback, this, _1), false)
{

}

void DigControlServer::digCallback(const actionlib::SimpleActionServer<DigAction>::GoalConstPtr &goal)
{
}

void DigControlServer::dumpCallback(const actionlib::SimpleActionServer<DumpAction>::GoalConstPtr &goal)
{
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tracker_node");
  ros::NodeHandle nh;
  DigControlServer server(&nh);
  ros::spin();
}


