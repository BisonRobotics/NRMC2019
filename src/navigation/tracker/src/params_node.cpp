#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tracker/GetUInt.h>
#include <tracker/SetUIntAction.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "params_node");
  ros::Rate rate(10);
  using actionlib::SimpleClientGoalState;
  actionlib::SimpleActionClient<tracker::SetUIntAction> brightness_client0("/tracker/left/set_brightness", false);
  actionlib::SimpleActionClient<tracker::SetUIntAction> brightness_client1("/tracker/right/set_brightness", false);
  actionlib::SimpleActionClient<tracker::SetUIntAction> exposure_client0("/tracker/left/set_exposure", false);
  actionlib::SimpleActionClient<tracker::SetUIntAction> exposure_client1("/tracker/right/set_exposure", false);

  int brightness = 255;
  int exposure = 89;

  ros::Time start = ros::Time::now();
  bool configured = false;
  ROS_INFO("[params_node]: Starting");
  while(ros::ok() && !configured)
  {
    ros::Time current = ros::Time::now();
    if (current - start > ros::Duration(20.0))
    {
      int value;
      SimpleClientGoalState state(SimpleClientGoalState::SUCCEEDED);

      // Brightness
      state = brightness_client0.getState();
      if (state != SimpleClientGoalState::PENDING && // TODO maybe check current brightness
          state != SimpleClientGoalState::ACTIVE)
      {
        brightness = value;
        tracker::SetUIntActionGoal request;
        request.goal.value = (uint)value;
        brightness_client0.sendGoal(request.goal);
      }

      state = brightness_client1.getState();
      if (state != SimpleClientGoalState::PENDING && // TODO maybe check current brightness
          state != SimpleClientGoalState::ACTIVE)
      {
        brightness = value;
        tracker::SetUIntActionGoal request;
        request.goal.value = (uint)value;
        brightness_client1.sendGoal(request.goal);
      }

      // Exposure
      state = exposure_client0.getState();
      if (state != SimpleClientGoalState::PENDING && // TODO maybe check current exposure
          state != SimpleClientGoalState::ACTIVE)
      {
        exposure = value;
        tracker::SetUIntActionGoal request;
        request.goal.value = (uint)value;
        exposure_client0.sendGoal(request.goal);
      }
      state = exposure_client1.getState();
      if (state != SimpleClientGoalState::PENDING && // TODO maybe check current exposure
          state != SimpleClientGoalState::ACTIVE)
      {
        exposure = value;
        tracker::SetUIntActionGoal request;
        request.goal.value = (uint)value;
        exposure_client1.sendGoal(request.goal);
      }
    }

    if (current - start > ros::Duration(40.0))
    {
      ROS_INFO("[params_node]: Done");
      return 0;
    }

    rate.sleep();
    ros::spinOnce();
  }
}