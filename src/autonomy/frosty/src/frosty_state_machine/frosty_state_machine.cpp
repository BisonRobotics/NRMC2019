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

bool FrostyStateMachine::state3_done;
bool FrostyStateMachine::need2_dig;
bool FrostyStateMachine::done_dumping;


FrostyStateMachine::FrostyStateMachine(bool dig_sim, bool drive_sim, double dig_time_sec, double dump_time_sec):
dig_sim(dig_sim), drive_sim(drive_sim), min_dig_time(dig_time_sec), min_dump_time(dump_time_sec)
{
    state = 0;
    time = 0;
    dig_timer = 0;
    //initialize actionlib clients
    path_alc = new actionlib::SimpleActionClient<navigation_msgs::FollowPathAction> ("follow_path", true);
    dig_alc  = new actionlib::SimpleActionClient<dig_control::DigAction> ("dig_server", true);
    dump_alc  = new actionlib::SimpleActionClient<dig_control::DumpAction> ("dump_server", true);
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
        res = state3CheckGoToDig();
        if (res == Frosty_ns::StateResult::SUCCESS)
        {
            state = 4;
            dig_timer = 0;
        }
        else
        {
            ROS_DEBUG("Path Check: %s", (res == Frosty_ns::StateResult::FAILED) ? "FAILED": "IN PROGRESS");
        }

    break;
    case 4:
        ROS_DEBUG("IN STATE 4");
        res = state4Dig(dt);
        if (res == Frosty_ns::StateResult::SUCCESS)
        {
            state = 5;
        }
        else
        {
            ROS_DEBUG("Dig Start: %s", (res == Frosty_ns::StateResult::FAILED) ? "FAILED": "IN PROGRESS");
        }
    break;
    case 5:
        ROS_DEBUG("IN STATE 5");
        state5StartGoToHopper();
        state = 6;
    break;
    case 6:
        res = state6CheckGoToHopper();
        if (res == Frosty_ns::StateResult::SUCCESS)
        {
            state = 7; 
        }
        else
        {
            ROS_DEBUG("return Path Check: %s", (res == Frosty_ns::StateResult::FAILED) ? "FAILED": "IN PROGRESS");
        }
    break;
    case 7:
        ROS_DEBUG("IN STATE 7");
        state7StartDump();
        state = 8;
    break;
    case 8:
        res = state8CheckDumpAndConditions();
        if (res == Frosty_ns::StateResult::SUCCESS)
        {
            state = 2;
        }
        else
        {
            ROS_DEBUG("DUMP CHECK: %s", (res == Frosty_ns::StateResult::FAILED) ? "FAILED": "IN PROGRESS");
        }
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
    // check driving and digging nodes to make sure they are up and ready
    // if actionlib server connects, they are ready.
    if (/*time > 10 ||*/ (path_alc->isServerConnected() /*&& dig_alc->isServerConnected()*/))
    {
        return Frosty_ns::StateResult::SUCCESS;
    }
    else 
    {
        return Frosty_ns::StateResult::IN_PROCESS;
    }
}

void FrostyStateMachine::state2StartGoToDig()
{ 
    //post static path (and implicit starting zero point turn) 
    // to path actionlib server from path_alc actionlib client 
    //make goal
    
    navigation_msgs::BezierSegment segment_1;
    segment_1.p0.x = 2;
    segment_1.p0.y = 1;
    segment_1.p1.x = 2;
    segment_1.p1.y = 3;
    segment_1.p2.x = 2;
    segment_1.p2.y = 4;
    segment_1.p3.x = 2;
    segment_1.p3.y = 5;
    segment_1.path_cost = 0; //free
    segment_1.min_radius = .1;
    segment_1.direction_of_travel = 1; //static_cast<int8_t>(navigation_msgs::Direction::forward); //aka 1

    ROS_INFO("[frosty]: Sending path to action server");
    navigation_msgs::FollowPathGoal goal;
    std::vector<navigation_msgs::BezierSegment> p;
    p.push_back(segment_1);
    goal.path = p;
    FrostyStateMachine::state3_done = false;
    path_alc->sendGoal(goal, &FrostyStateMachine::state3CheckDoneCallback);
}

void FrostyStateMachine::state3CheckDoneCallback(const actionlib::SimpleClientGoalState& state,
                                                 const navigation_msgs::FollowPathResultConstPtr& result)
{
    FrostyStateMachine::state3_done = true;
    ROS_WARN("PATH CALLED BACK AS DONE");
}

void FrostyStateMachine::state4CheckDigCallback(const actionlib::SimpleClientGoalState& state,
                                                const dig_control::DigResultConstPtr& result)
{
    FrostyStateMachine::need2_dig = true;
}

void FrostyStateMachine::state8CheckDumpCallback(const actionlib::SimpleClientGoalState& state,
                                                 const dig_control::DumpResultConstPtr& result)
{
    FrostyStateMachine::done_dumping = true;                                                     
}

Frosty_ns::StateResult FrostyStateMachine::state3CheckGoToDig()
{
    if (FrostyStateMachine::state3_done) 
    {
        FrostyStateMachine::need2_dig = true;
        return Frosty_ns::StateResult::SUCCESS;
    }
    return Frosty_ns::StateResult::IN_PROCESS;
}

Frosty_ns::StateResult FrostyStateMachine::state4Dig(double dt)
{ 
    dig_timer += dt;
    //actionlib::SimpleClientGoalState state = dig_alc->getState();
    if (FrostyStateMachine::need2_dig)
    {
      if (dig_timer < min_dig_time) //TODO use done callback instead for thread safety/sense stuff
      {
        ROS_INFO("[frosty]: Sending dig command.");
        dig_control::DigGoal goal;
        FrostyStateMachine::need2_dig = false;
        dig_alc->sendGoal(goal, &FrostyStateMachine::state4CheckDigCallback);
        return Frosty_ns::StateResult::IN_PROCESS;
      }
      else
      {
        dig_timer = 0;
        return Frosty_ns::StateResult::SUCCESS;
      }
    }
    else
    {
        return Frosty_ns::StateResult::IN_PROCESS;
    }
}

void FrostyStateMachine::state5StartGoToHopper()
{ 
    //post static path (and implicit starting zero point turn) 
    // to path actionlib server from path_alc actionlib client 
    //make goal
    
    navigation_msgs::BezierSegment segment_1;
    segment_1.p0.x = 2;
    segment_1.p0.y = 5;
    segment_1.p1.x = 2;
    segment_1.p1.y = 4;
    segment_1.p2.x = 2;
    segment_1.p2.y = 1; //Move this towards p3.y for a straighter entrance
    segment_1.p3.x = 1;
    segment_1.p3.y = 1;
    segment_1.path_cost = 0; //free
    segment_1.min_radius = .1;
    segment_1.direction_of_travel = 0; //static_cast<int8_t>(navigation_msgs::Direction::reverse); //aka 0

    ROS_INFO("[frosty]: Sending return path to action server");
    navigation_msgs::FollowPathGoal goal;
    std::vector<navigation_msgs::BezierSegment> p;
    p.push_back(segment_1);
    goal.path = p;
    FrostyStateMachine::state3_done = false;
    path_alc->sendGoal(goal, &FrostyStateMachine::state3CheckDoneCallback);
}

Frosty_ns::StateResult FrostyStateMachine::state6CheckGoToHopper()
{
    //check progress from actionlib feedback
    if (FrostyStateMachine::state3_done)
    {
        FrostyStateMachine::state3_done = false;
        return Frosty_ns::StateResult::SUCCESS;
    }
    return Frosty_ns::StateResult::IN_PROCESS;
}

void FrostyStateMachine::state7StartDump()
{
    dig_control::DumpGoal goal;
    FrostyStateMachine::done_dumping = false;
    dump_alc->sendGoal(goal, &FrostyStateMachine::state8CheckDumpCallback);
}

Frosty_ns::StateResult FrostyStateMachine::state8CheckDumpAndConditions()
{
    //TODO add timeout or count
    if (FrostyStateMachine::done_dumping)
    {
        return Frosty_ns::StateResult::SUCCESS;
    }
    else 
    {
        return Frosty_ns::StateResult::IN_PROCESS;
    }
}


FrostyStateMachine::~FrostyStateMachine()
{
    delete path_alc;
    delete dig_alc;
}
    //Frosty_ns::StateResult state2GoToDig();
    //Frosty_ns::StateResult state3Dig();
    //Frosty_ns::StateResult state4GoToHopper();
    //Frosty_ns::StateResult state5Dump();
    //Frosty_ns::StateResult state6CheckCondtitions();
