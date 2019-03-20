#include <frosty_state_machine/frosty_state_machine.h>
#include <ros/ros.h>
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
    break;
    case 3:
    break;
    case 4:
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
    if (time > 10)
        return Frosty_ns::StateResult::SUCCESS;
    else return Frosty_ns::StateResult::IN_PROCESS;
    //TODO: check driving and digging nodes to make sure they are up and ready
}

void FrostyStateMachine::state2StartGoToDig()
{ 
    //post static path (and implicit starting zero point turn) 
    // to path actionlib server from path_alc actionlib client 
        
}

Frosty_ns::StateResult FrostyStateMachine::state2CheckGoToDig()
{
    //check progress from actinlib feedback
}
    //Frosty_ns::StateResult state2GoToDig();
    //Frosty_ns::StateResult state3Dig();
    //Frosty_ns::StateResult state4GoToHopper();
    //Frosty_ns::StateResult state5Dump();
    //Frosty_ns::StateResult state6CheckCondtitions();
