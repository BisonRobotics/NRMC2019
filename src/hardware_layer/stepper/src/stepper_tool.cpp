#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <stepper/stepper.h>
#include <boost/timer/timer.hpp>

using namespace stepper;
using namespace boost::program_options;

int main(int argc, char *argv[])
{
  ros::Time::init();
  Stepper stepper("can0", 1, 3);

  try
  {
    options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("disable,d", "Disable")
        ("initialize,i", value<float>(), "Initialize stepper")
        ("setPosition,p",  value<float>(), "Set Position")
        ("setVelocity,v",  value<float>(), "Set Velocity")
        ("setScan,s", value<float>(), "Set Scan")
        ("getState,g", "Get state")
        ("pollState", value<float>(), "Poll state");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
    {
      std::cout << desc << '\n';
    }
    if (vm.count("setDisable"))
    {
      try
      {
        std::cout << "Disabling... ";
        stepper.setMode(Mode::Disabled, 0.0f);
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
        stepper.setMode(Mode::Initialize, velocity);
        std::cout << "Success" << std::endl;
      }
      catch (std::runtime_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("setPosition"))
    {
      float position = vm["setPosition"].as<float>();
      try
      {
        std::cout << "Setting position: " << position << "... ";
        stepper.setMode(Mode::Position, position);
        std::cout << "Success" << std::endl;
      }
      catch (std::runtime_error &e)
      {
        std::cout << e.what() << std::endl;
      }
    }
    if (vm.count("setVelocity"))
    {
      float velocity = vm["setVelocity"].as<float>();
      try
      {
        std::cout << "Setting velocity: " << velocity << "...";
        stepper.setMode(Mode::Velocity, velocity);
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
        stepper.setMode(Mode::Scan, velocity);
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
        State state = stepper.pollState();
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
          timer.start(); State state = stepper.pollState(); timer.stop();
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
}