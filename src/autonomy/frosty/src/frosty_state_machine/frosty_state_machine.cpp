#include <frosty_state_machine/frosty_state_machine.h>
#include <ros/ros.h>

#include <navigation_msgs/FollowPathAction.h>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>

//This class functions as the state machine for the 2019 NDSU (N)RMC Competition.
/*
State 1 . Initialize
State 1a. Wait for localization to settle
State 2 . Go to dig zone
State 3 . Dig for x amount of time
State 4 . Return to Hopper
State 5 . Dump
State 6 . Check Time/Conditions 
          and GOTO 2.
*/

FrostyStateMachine::FrostyStateMachine(bool dig_sim, bool drive_sim, double dig_time_sec, double dump_time_sec):
dig_sim(dig_sim), drive_sim(drive_sim), dig_time(dig_time_sec), dump_time(dump_time_sec)
{
    state = 0;
    time = 0;
    //initialize actionlib clients
    path_alc = new actionlib::SimpleActionClient<navigation_msgs::FollowPathAction> ("follow_path", true);
    //dig one here too
    //and dump
}

int FrostyStateMachine::getState()
{
    return state;
}

Frosty_ns::StateResult FrostyStateMachine::update(double dt)
{
    
  Frosty_ns::StateResult res;
  switch (state)
  {
    case 0:
        state1StartInit(time);
        state = 1;
    break;
    case 1:
        res = state1CheckInit();
        if (res == Frosty_ns::StateResult::SUCCESS)
        {
            state = 2;
        }
        else
        {
            ROS_DEBUG("Init Check: %s", (res == Frosty_ns::StateResult::FAILED) ? "FAILED": "IN PROGRESS");
        }
    break;
    case 2:
        ROS_DEBUG("IN STATE 2");
        state2StartGoToDig();
        state = 3;
    break;
    case 3:
        res = state2CheckGoToDig();
        if (res == Frosty_ns::StateResult::SUCCESS)
        {
            state = 4;
        }
        else
        {
            ROS_DEBUG("Init Check: %s", (res == Frosty_ns::StateResult::FAILED) ? "FAILED": "IN PROGRESS");
        }

    break;
    case 4:
        ROS_DEBUG("IN STATE 4");

    break;
    case 5:
    break;
    case 6:
    break;
  }
  time += dt;
}

void FrostyStateMachine::state1StartInit(double t)
{
    ROS_DEBUG("Init called at time = %.4f", t);
        
}

Frosty_ns::StateResult FrostyStateMachine::state1CheckInit()
{
    if (time > 10 || path_alc->isServerConnected())
        return Frosty_ns::StateResult::SUCCESS;
    else return Frosty_ns::StateResult::IN_PROCESS;
    
    //TODO: check driving and digging nodes to make sure they are up and ready
}

void FrostyStateMachine::state2StartGoToDig()
{ 
    //post static path (and implicit starting zero point turn) 
    // to path actionlib server from path_alc actionlib client 
    //make goal
    
    navigation_msgs::BezierSegment segment_1;
    segment_1.p0.x = 1;
    segment_1.p0.y = 0;
    segment_1.p1.x = 2;
    segment_1.p1.y = 0;
    segment_1.p2.x = 3;
    segment_1.p2.y = 0;
    segment_1.p3.x = 4;
    segment_1.p3.y = 0;
    segment_1.path_cost = 0; //free
    segment_1.min_radius = .1;
    segment_1.direction_of_travel = 1; //static_cast<int8_t>(navigation_msgs::Direction::forward); //aka 1

    ROS_INFO("[path_planner_node]: Sending goal to action server");
    navigation_msgs::FollowPathGoal goal;
    std::vector<navigation_msgs::BezierSegment> p;
    p.push_back(segment_1);
    goal.path = p;
    path_alc->sendGoal(goal);
}

Frosty_ns::StateResult FrostyStateMachine::state2CheckGoToDig()
{
    //check progress from actinlib feedback
    actionlib::SimpleClientGoalState state = path_alc->getState();
    //state = path_alc->getState();
    if (state.isDone())
    {
        return Frosty_ns::StateResult::SUCCESS;
    }
    return Frosty_ns::StateResult::IN_PROCESS;
}
    //Frosty_ns::StateResult state2GoToDig();
    //Frosty_ns::StateResult state3Dig();
    //Frosty_ns::StateResult state4GoToHopper();
    //Frosty_ns::StateResult state5Dump();
    //Frosty_ns::StateResult state6CheckCondtitions();
