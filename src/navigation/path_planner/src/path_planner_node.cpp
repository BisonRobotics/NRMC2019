/*******************************************************************************
 * @file    HW01.c
 * @brief   Homework 1
 * @author  Jacob Huesman
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <julia.h>

#include <sstream>
#include <boost/thread/thread.hpp>
#include <queue>
#include <boost/chrono.hpp>
#include <queue>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
JULIA_DEFINE_FAST_TLS()

using std::queue;
using std::string;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
volatile bool g_exit;

/*******************************************************************************
 * Callbacks
 ******************************************************************************/
void signal_handler(int signal)
{
  if (signal == SIGTERM)
  {
    ROS_INFO("SIGTERM received");
  }
  else
  {
    ROS_WARN("Signal %i received", signal);
  }
  g_exit = true;
}

/*******************************************************************************
 * Threads
 ******************************************************************************/
void juliaThread(queue<string> *input, queue<string> *output)
{
  // Setup Julia
  ROS_INFO("Started Julia thread");
  jl_init();
  ROS_INFO("Julia initialized");

  while(!g_exit)
  {
    if (!input->empty())
    {
      input->pop();
      jl_value_t *ret = jl_eval_string("sqrt(2.0)");
      if (jl_typeis(ret, jl_float64_type))
      {
        double unboxed = jl_unbox_float64(ret);
        ROS_INFO("Return value: %g", unboxed);
      }
      else
      {
        ROS_WARN("Invalid datatype returned");
      }
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
  }

  jl_atexit_hook(0);
  ROS_INFO("Exited Julia");
}

int main(int argc, char **argv)
{
  // Handle exit signal
  g_exit = false;
  signal(SIGTERM, signal_handler);

  // Start Julia thread
  queue<string> input, output;
  boost::thread julia_thread(juliaThread, &input, &output);

  // Setup ROS
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(1.0);

  // Run program
  int count = 0;
  while (ros::ok() && !g_exit)
  {
    std_msgs::String msg;
    input.push("Try this!");
    //std::stringstream ss;
    //ss << jl_eval_string("sqrt(2.0)") << count;
    //msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str());

    //chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  // Allow Julia to exit cleanly
  julia_thread.join();
  ROS_INFO("Cleanup successful");

  return 0;
}