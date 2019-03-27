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
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigation_msgs/FollowPathAction.h>
#include <std_msgs/Empty.h>
#include <dig_control/DumpAction.h>
#include <dig_control/DigAction.h>


namespace Frosty_ns 
{
  enum class StateResult
  {
    FAILED, IN_PROCESS, SUCCESS
  };
}

class FrostyStateMachine
{
public:
    FrostyStateMachine(bool dig_sim, bool drive_sim, double dig_time_sec, double dump_time_sec);
    int getState();
    Frosty_ns::StateResult update(double dt);
    ~FrostyStateMachine();
private:
    //need path planning actionlib client
    actionlib::SimpleActionClient<navigation_msgs::FollowPathAction> *path_alc;
    //need dig/dump actionlib client //wait
    actionlib::SimpleActionClient<dig_control::DigAction> *dig_alc;
    actionlib::SimpleActionClient<dig_control::DumpAction> *dump_alc;
    //SimpleActionClient<DumpAction> *dump_alc;
    double time;
    int state;
    bool dig_sim, drive_sim;
    double min_dig_time, min_dump_time;
    void state1StartInit(double t);
    Frosty_ns::StateResult state1CheckInit();
    void state2StartGoToDig();
    Frosty_ns::StateResult state3CheckGoToDig();
    Frosty_ns::StateResult state4Dig(double dt);
    void state5StartGoToHopper();
    Frosty_ns::StateResult state6CheckGoToHopper();
    void state7StartDump();
    Frosty_ns::StateResult state8CheckDumpAndConditions();

    double dig_timer;
    static void state3CheckDoneCallback(const actionlib::SimpleClientGoalState& state,
                                        const navigation_msgs::FollowPathResultConstPtr& result);
    static void state4CheckDigCallback(const actionlib::SimpleClientGoalState& state,
                                       const dig_control::DigResultConstPtr& result);
    static void state8CheckDumpCallback(const actionlib::SimpleClientGoalState& state,
                                       const dig_control::DumpResultConstPtr& result);
    static bool state3_done;
    static bool need2_dig;
    static bool done_dumping;
    //Frosty_ns::StateResult state4GoToHopper();
    //Frosty_ns::StateResult state5Dump();
    //Frosty_ns::StateResult state6CheckCondtitions();
};