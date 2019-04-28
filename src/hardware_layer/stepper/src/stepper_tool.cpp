#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <stepper/stepper.h>
#include <boost/timer/timer.hpp>
#include <map>
#include <algorithm>

using namespace stepper;
using namespace boost::program_options;

int main(int argc, char *argv[])
{
  ros::Time::init();
  Stepper *stepper;

  try
  {
    options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("disable,d", "Disable")
        ("id,c", value<uint>(), "Controller id")
        ("initialize,i", value<float>(), "Initialize stepper")
        ("setMode,m", value<std::string>(), "Set Mode")
        ("setPoint,p",  value<float>(), "Set Point")
        ("setScan,s", value<float>(), "Start Scan")
        ("getState,g", "Get state")
        ("pollState", value<float>(), "Poll state");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
    {
      std::cout << desc << '\n';
    }
    if (vm.count("id"))
    {
      uint id = vm["id"].as<uint>();
      uint computer_id = 0;
      if (id == 1)
      {
        computer_id = 3;
      }
      else if (id == 2)
      {
        computer_id = 4;
      }
      else
      {
        printf("Controller id must be 1 or 2\n");
        return 0;
      }
      printf("[ID %i,%i]: ", id, computer_id);
      stepper = new Stepper("tracker_can", id, computer_id);
    }
    else
    {
      printf("[ID 1,3]: ");
      stepper = new Stepper("tracker_can", 1, 3);
    }
    if (vm.count("disable"))
    {
      try
      {
        std::cout << "Disabling... ";
        stepper->setMode(Mode::Disabled, 0.0f);
        std::cout << "Success" << std::endl;
      }
      catch (std::runtime_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("initialize"))
    {
      float velocity = vm["initialize"].as<float>();
      try
      {
        std::cout << "Initializing at velocity: " << velocity << "... ";
        stepper->setMode(Mode::Initialize, velocity);
        std::cout << "Success" << std::endl;
      }
      catch (std::runtime_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("setMode"))
    {
      std::map<std::string, Mode> map;
      map.emplace("initialize", Mode::Initialize);
      map.emplace("velocity", Mode::Velocity); // TODO finish
      std::string mode = vm["setMode"].as<std::string>();
      std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
      try
      {
        std::cout << "Setting mode: " << mode << "... ";
        stepper->setMode(map[mode], 0.0);
        std::cout << "Success" << std::endl;
      }
      catch (std::runtime_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("setPoint"))
    {
      float set_point = vm["setPoint"].as<float>();
      try
      {
        std::cout << "Setting set point: " << set_point << "...";
        stepper->setPoint(set_point);
        std::cout << "Success" << std::endl;
      }
      catch (std::runtime_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("setScan"))
    {
      float velocity = vm["setScan"].as<float>();
      try
      {
        std::cout << "Setting scanning velocity to: " << velocity << "... ";
        stepper->setMode(Mode::Scan, velocity);
        std::cout << "Success" << std::endl;
      }
      catch (std::runtime_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("getState"))
    {
      try
      {
        std::cout << "Getting state... ";
        State state = stepper->pollState();
        printf("P = %f, V = %f\n", state.position, state.velocity);
      }
      catch (std::runtime_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("pollState"))
    {
      try
      {
        boost::timer::cpu_timer timer;
        float value = vm["pollState"].as<float>();
        ros::Rate rate(value);
        std::cout << "Getting state... " << std::endl;
        while (true)
        {
          timer.start(); State state = stepper->pollState(); timer.stop();
          double elapsed = (double)timer.elapsed().wall / 1.0e9;
          printf("P = %7.4f, V = %7.4f T = %7.4f\n", state.position, state.velocity, elapsed);
          rate.sleep();
        }
      }
      catch (std::runtime_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
  }
  catch (const error &ex)
  {
    std::cerr << ex.what() << '\n';
  }

  delete stepper;
}