#ifndef GAZEBO_INTERFACE_JOINT_CONTROLLER_PLUGIN_H
#define GAZEBO_INTERFACE_JOINT_CONTROLLER_PLUGIN_H

#include "../../../../../../../../usr/include/gazebo-9/gazebo/gazebo.hh"
#include "../../../../../../../../usr/include/gazebo-9/gazebo/physics/physics.hh"

#include <gazebo_drivers/gazebo_wheel_driver.h>

namespace gazebo
{
class DriverPlugin : public ModelPlugin
{
  public:
    DriverPlugin() {}
    ~DriverPlugin();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  private:
    physics::ModelPtr model;
    physics::JointPtr joint;
    common::PID pid;
    GazeboDriver *fl, *bl, *fr, *br;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(DriverPlugin)
}

#endif //GAZEBO_INTERFACE_JOINT_CONTROLLER_PLUGIN_H
