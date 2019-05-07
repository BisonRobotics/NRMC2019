#include <dig_control/dig_control_server.h>
#include <dig_control/Debug.h>
#include <utilities/joy.h>
#include <utilities/filter.h>

using namespace dig_control;
using utilities::simpleLowPassFilter;

DigControlServer::DigControlServer(ros::NodeHandle *nh, Config config) :
  manual_safety(false), autonomy_safety(false),
  backhoe_duty(0.0f), bucket_duty(0.0f), central_duty(0.0f), vibrator_duty(0.0f), central_drive_angle(0.0f),
  monoboom_params{-.0808, -0.0073,  0.0462,  0.9498,  -0.0029},
  flap_params{85.0010, -376.8576, 620.7329, -453.8172, 126.0475},
  backhoe_params{12.852515, -29.737412, 26.138260, -9.193020, 0.699974, 2.190548, 0.004798},
  nh(nh), config(config), controller(config), seq(0),
  server(*nh, "action", false)
{
  joint_angles.header.stamp = ros::Time::now();
  joint_angles.header.seq = seq;
  joint_angles.name.emplace_back("frame_to_central_drive");
  joint_angles.name.emplace_back("frame_to_monoboom");
  joint_angles.name.emplace_back("frame_to_gravel_bucket");
  joint_angles.name.emplace_back("monoboom_to_bucket");
  joint_angles.name.emplace_back("left_flap_joint");
  joint_angles.name.emplace_back("right_flap_joint");
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);

  joy_subscriber = nh->subscribe("/joy", 1, &DigControlServer::joyCallback, this);
  joint_publisher = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
  debug_publisher = nh->advertise<dig_control::Debug>("debug", 10);
  controller.setControlState(ControlState::manual);
  server.registerGoalCallback(boost::bind(&DigControlServer::goalCallback, this));
  server.registerPreemptCallback(boost::bind(&DigControlServer::preemptCallback, this));
  server.start();
}

void DigControlServer::goalCallback()
{
  auto goal = server.acceptNewGoal();
  ControlState request = toControlState(*goal);
  ControlState current_state = controller.getControlState();

  ROS_INFO("[DigControlServer::goalCallback] Request for %s to %s",
            to_string(current_state).c_str(), to_string(request).c_str());
  switch (request)
  {
    case ControlState::manual:
    {
      // Set to manual regardless of current state
      controller.setControlState(request);
      controller.stop();
      server.setSucceeded(toResult(request));
      break;
    }
    case ControlState::dig:
    case ControlState::dump:
    {
      // Make sure controller is ready for new command
      if (current_state == ControlState::ready || current_state == ControlState::manual)
      {
        controller.setControlState(request);
      }
      else
      {
        server.setAborted(toResult(current_state));
        controller.setControlState(ControlState::error);
        controller.stop();
        ROS_ERROR("[DigControlServer::goalCallback] Unable to set control state from %s to %s",
                  to_string(current_state).c_str(), to_string(request).c_str());
      }
      break;
    }
    case ControlState::finish_dig:
    {
      if (current_state == ControlState::dig)
      {
        controller.setControlState(request);
      }
      else
      {
        server.setAborted(toResult(current_state));
        controller.setControlState(ControlState::error);
        controller.stop();
        ROS_ERROR("[DigControlServer::goalCallback] Unable to set control state from %s to %s",
                  to_string(current_state).c_str(), to_string(request).c_str());
      }
      break;
    }
    case ControlState::finish_dump:
    {
      if (current_state == ControlState::dump)
      {
        controller.setControlState(request);
      }
      else
      {
        server.setAborted(toResult(current_state));
        controller.setControlState(ControlState::error);
        controller.stop();
        ROS_ERROR("[DigControlServer::goalCallback] Unable to set control state from %s to %s",
                  to_string(current_state).c_str(), to_string(request).c_str());
      }
      break;
    }
    default:
    {
      server.setAborted(toResult(current_state));
      controller.setControlState(ControlState::error);
      controller.stop();
      ROS_ERROR("[DigControlServer::goalCallback] Unable to set control state from %s to %s",
                to_string(current_state).c_str(), to_string(request).c_str());
    }
  }
}

void DigControlServer::preemptCallback()
{
  ControlState current_state = controller.getControlState();
  server.setPreempted(toResult(current_state));
  ROS_INFO("[DigControlServer::preemptCallback] Preempting from %s", to_string(current_state).c_str());
}

void DigControlServer::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  using utilities::Joy;
  Joy joy(joy_msg);
  manual_safety = joy.get(Joy::MANUAL_SAFETY);
  autonomy_safety = joy.get(Joy::AUTONOMY_SAFETY);
  if (manual_safety)
  {
    // Update bucket (Maintain state)
    if (joy.get(Joy::BUCKET_UP) && joy.get(Joy::BUCKET_DOWN))
    {
      ROS_WARN("[teleop] Conflicting commands, x and y are both pressed, stopping motion");
      bucket_duty = 0.0;
    }
    else if (joy.get(Joy::BUCKET_DOWN))
    {
      bucket_duty = -config.bucketDuty().fast;
    }
    else if (joy.get(Joy::BUCKET_UP))
    {
      bucket_duty = config.bucketDuty().fast;
    }

    // Update backhoe (Maintain state)
    if (joy.get(Joy::LINEAR_IN) && joy.get(Joy::LINEAR_OUT))
    {
      ROS_WARN("[teleop] Conflicting commands, a and b are both pressed, stopping motion");
      backhoe_duty = 0.0f;
    }
    else if (joy.get(Joy::LINEAR_IN))
    {
      //backhoe_duty = -config.backhoeDuty().normal;
      backhoe_duty = -config.backhoeDuty().normal;
    }
    else if (joy.get(Joy::LINEAR_OUT))
    {
      //backhoe_duty = config.backhoeDuty().fast;
      backhoe_duty = config.backhoeDuty().normal;
    }

    // Update central drive
    if (joy.get(Joy::CENTRAL_DRIVE_UP) && joy.get(Joy::CENTRAL_DRIVE_DOWN))
    {
      ROS_WARN("[teleop] Conflicting commands, up and dp are both pressed, stopping motion");
      central_duty = 0.0f;
    }
    else if (joy.get(Joy::CENTRAL_DRIVE_UP))
    {
      central_duty = config.centralDriveDuty().fast;
    }
    else if (joy.get(Joy::CENTRAL_DRIVE_DOWN))
    {
      central_duty = -config.centralDriveDuty().normal;
    }
    else
    {
      central_duty = 0.0f;
    }

    // Update vibrator (Maintain state)
    if (joy.get(Joy::VIBRATOR_OFF) && joy.get(Joy::VIBRATOR_ON))
    {
      ROS_WARN("[teleop] Conflicting commands, lt and rt are both pressed, stopping motion");
      vibrator_duty = 0.0f;
    }
    else if (joy.get(Joy::VIBRATOR_ON))
    {
      vibrator_duty = 0.0f;
    }
    else if (joy.get(Joy::VIBRATOR_OFF))
    {
      vibrator_duty = config.vibratorDuty().normal;
    }
  }
  else
  {
    central_duty = 0.0f;
    backhoe_duty = 0.0f;
    bucket_duty = 0.0f;
    vibrator_duty = 0.0f;
  }
}

DigControlResult DigControlServer::toResult(ControlState state)
{
  DigControlResult result;
  result.control_state = (DigControlResult::_control_state_type)state;
  return result;
}

ControlState DigControlServer::toControlState(DigControlGoal goal)
{
  return (ControlState)goal.control_state;
}

void DigControlServer::update()
{
  // Update
  controller.update();
  updateCentralDriveAngle();
  seq++;

  // Feedback
  if (server.isActive())
  {
    DigControlFeedback feedback;
    feedback.control_state       = (DigControlFeedback::_control_state_type)       controller.getControlState();
    feedback.dig_state           = (DigControlFeedback::_dig_state_type)           controller.getDigState();
    feedback.backhoe_state       = (DigControlFeedback::_backhoe_state_type)       controller.getBackhoeState();
    feedback.central_drive_state = (DigControlFeedback::_central_drive_state_type) controller.getCentralDriveState();
    feedback.bucket_state        = (DigControlFeedback::_bucket_state_type)        controller.getBucketState();
    server.publishFeedback(feedback);
  }

  // Safety check and teleop
  ControlState dig_state = controller.getControlState();
  if (manual_safety)
  {
    if (dig_state == ControlState::manual)
    {
      controller.setCentralDriveDuty(central_duty);
      controller.setBackhoeDuty(backhoe_duty);
      controller.setVibratorDuty(vibrator_duty);
      controller.setBucketDuty(bucket_duty);
    }
  }
  else if (!(autonomy_safety || config.fullAutonomy()))
  {
    controller.setControlState(ControlState::manual);
    controller.stop();
  }

  // Info
  if (config.debug())
  {
    Debug message;
    message.header.stamp = ros::Time::now();
    message.header.seq = seq;
    message.position.central  = controller.getCentralDrivePosition();
    message.position.backhoe  = controller.getBackhoePosition();
    message.position.bucket   = controller.getBucketPosition();
    message.duty.backhoe      = controller.getBackhoeDuty();
    message.duty.vibrator     = controller.getVibratorDuty();
    message.duty.bucket       = controller.getBucketDuty();
    message.duty.central      = controller.getCentralDriveDuty();
    message.current.backhoe   = controller.getBackhoeCurrent();
    message.current.vibrator  = controller.getVibratorCurrent();
    message.current.bucket    = controller.getBucketCurrent();
    message.current.central   = controller.getCentralDriveCurrent();
    message.state.control     = controller.getControlStateString();
    message.battery_voltage   = controller.getBatteryVoltage();
    message.state.central     = controller.getCentralDriveStateString();
    message.state.backhoe     = controller.getBackhoeStateString();
    message.state.bucket      = controller.getBucketStateString();
    message.state.dig         = controller.getDigStateString();
    //message.state_i.control = (uint8_t)controller.getControlState();
    //message.state_i.central = (uint8_t)controller.getCentralDriveState();
    //message.state_i.dig     = (uint8_t)controller.getDigState();
    //message.state_i.backhoe = (uint8_t)controller.getBackhoeState();
    //message.state_i.bucket  = (uint8_t)controller.getBucketState();
    debug_publisher.publish(message);
  }

  // Visuals
  joint_angles.header.stamp = ros::Time::now();
  joint_angles.header.seq = seq;
  joint_angles.position[0] = getCentralDriveAngle();
  joint_angles.position[1] = getMonoBoomAngle();
  joint_angles.position[2] = getBucketAngle();
  joint_angles.position[3] = getBackhoeAngle();
  joint_angles.position[4] = getFlapsAngle();
  joint_angles.position[5] = getFlapsAngle();
  joint_publisher.publish(joint_angles);
}

double DigControlServer::polyFit(const std::vector<double> &p, double x)
{
  int s = (int)p.size() - 1;
  double y = 0.0;
  for (int i = 0; i <= s; i++)
  {
    y += p[i]*std::pow(x, s-i);
  }
  return y;
}

void DigControlServer::updateCentralDriveAngle()
{
  simpleLowPassFilter<double>(central_drive_angle,
      9.1473E-4 * controller.getCentralDrivePosition() - 1.04, config.centralDriveAngleFilterK());
}

double DigControlServer::getCentralDriveAngle() const
{
  return central_drive_angle;
}

// The point where the backhoe starts moving back down on the potentiometer is at 2714
double DigControlServer::getMonoBoomAngle() const
{
  return -polyFit(monoboom_params, getCentralDriveAngle());
}

double DigControlServer::getFlapsAngle() const
{
  double central_angle = getCentralDriveAngle();
  if (central_angle < 1.275 && central_angle > 0.785)
  {
    return -(polyFit(flap_params, central_angle) - M_PI_4);
  }
  else if (central_angle >= 1.275)
  {
    return M_PI_4;
  }
  else
  {
    return -1.45;
  }
}

double DigControlServer::getBackhoeAngle() const
{
  return -polyFit(backhoe_params, controller.getBackhoePosition() / 10500.0 * 0.85 + 0.15);
}

double DigControlServer::getBucketAngle() const
{
  // arccos((5^2 + 12^2 - x^2)/(2*5*12)) - arccos((5^2 + 12^2 - 10^2)/(2*5*12))
  return std::acos((169.0 - std::pow(controller.getBucketPosition(), 2.0)) / 120.0) - 0.958192;
}

void DigControlServer::stop()
{
  controller.stop();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dig_control_server");
  ros::NodeHandle nh("~");
  Config config(&nh);
  DigControlServer server(&nh, config);
  ros::Rate rate(50);
  while (ros::ok())
  {
    server.update();
    ros::spinOnce();
    rate.sleep();
  }
  server.stop();
}


