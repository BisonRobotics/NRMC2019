#include <waypoint_control/waypoint_controller.h>
#include <boost/algorithm/clamp.hpp>
#include <math.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include <boost/math/constants/constants.hpp>


using namespace waypoint_control;

using boost::algorithm::clamp;
using boost::math::double_constants::pi;
using std::signbit;
using std::abs;
using utilities::simpleLowPassFilter;


WaypointController::WaypointController(Config config, iVescAccess *front_left, iVescAccess *front_right,
    iVescAccess *back_right, iVescAccess *back_left) :
    fl(front_left), fr(front_right), br(back_right), bl(back_left), config(config),
    state(ControlState::manual), waypoint_state(WaypointState::ready), last_left(0.0), last_right(0.0),
    battery_voltage(0.0)
{
  dt = 1.0/config.rate();
  ROS_INFO("[WaypointController::WaypointController]: Waiting for VESCs to come online");
  while (ros::ok())
  {
    if (fl->isAlive() && fr->isAlive() && br->isAlive() && bl->isAlive())
    {
      break;
    }
    ROS_WARN("[WaypointController::WaypointController]: VESCs not online");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("[WaypointController::WaypointController]: VESCs online");
}

void WaypointController::setControlState(ControlState state)
{
  switch (state)
  {
    case ControlState::ready:
    case ControlState::manual:
    {
      waypoint_state = WaypointState::ready;
      waypoints.clear();
      if (this->state != state)
      {
        stop();
        this->state = state;
        ROS_INFO("[WaypointController::setControlState]: %s to %s",
                 to_string(this->state).c_str(), to_string(state).c_str());
      }
      else
      {
        ROS_WARN("[WaypointController::setControlState]: %s to %s",
                 to_string(this->state).c_str(), to_string(state).c_str());
      }

      break;
    }
    case ControlState::cancel:
    {
      stop();
      this->state = ControlState::ready;
      ROS_INFO("[WaypointController::setControlState]: %s to %s",
               to_string(this->state).c_str(), to_string(state).c_str());
      break;
    }
    case ControlState::error:
    case ControlState::new_goal:
    case ControlState::in_progress:
    {
      stop();
      this->state = ControlState::error;
      ROS_WARN("[WaypointController::setControlState]: Invalid transition %s to %s",
               to_string(this->state).c_str(), to_string(state).c_str());
      break;
    }
  }
}

void WaypointController::setControlState(ControlState state, const Waypoints &waypoints)
{
  stop();
  if (state == ControlState::new_goal)
  {
    if (!this->waypoints.empty())
    {
      ROS_WARN("[WaypointController::setControlState]: Flushing current waypoints");
      this->waypoints.clear();
    }
    ROS_INFO("[WaypointController::setControlState]: %s to %s, %lu waypoints",
             to_string(this->state).c_str(),
             to_string(ControlState::in_progress).c_str(),
             waypoints.size());
    this->waypoints = waypoints;
    this->state = ControlState::in_progress;
    this->waypoint_state = WaypointState::ready;
  }
  else
  {
    this->state = ControlState::error;
    ROS_WARN("[WaypointController::setControlState]: Invalid transition %s to %s",
             to_string(this->state).c_str(),
             to_string(state).c_str());
  }
}


void WaypointController::update(const Pose2D &pose, bool manual_safety, bool autonomy_safety, double left, double right)
{
  debug_info.waypoint = Waypoint();
  switch (state)
  {
    case ControlState::in_progress:
    {
      if (autonomy_safety)
      {
        if (waypoints.empty())
        {
          ROS_INFO("[WaypointController::update]: Processed all waypoints");
          state = ControlState::finished;
        }
        else
        {
          debug_info.waypoint = waypoints.back();
          updateControls(pose);
        }
      }
      else
      {
        stop();
      }
      break;
    }
    case ControlState::manual:
    {
      if (manual_safety)
      {
        setPoint(left, right);
      }
      else
      {
        stop();
      }
      break;
    }
    case ControlState::ready:
    case ControlState::finished:
    case ControlState::cancel:
    case ControlState::new_goal:
    case ControlState::error:
    {
      stop();
      break;
    }
  }
  this->last_pose = pose;

  // Update debug info
  debug_info.command_state = to_string(state);
  debug_info.waypoint_state = to_string(waypoint_state);
  debug_info.feedback.dx = feedback.x();
  debug_info.feedback.dy = feedback.y();
  debug_info.feedback.dr = feedback.r();
  debug_info.feedback.tr = feedback.theta();
  debug_info.feedback.td = feedback.theta()/pi*180.0;
  debug_info.battery_voltage = battery_voltage;
  debug_info.pose = pose;
}

void WaypointController::updateControls(const geometry_msgs::Pose2D& pose)
{
  if (waypoints.empty())
  {
    ROS_WARN("[WaypointController::updateControls::empty]: No waypoints");
    return;
  }

  Waypoint waypoint = waypoints.back();
  last_feedback = feedback;
  feedback = Feedback(pose, waypoint);

  switch (waypoint_state)
  {
    case WaypointState::error:
    {
      ROS_ERROR("[WaypointController::updateControls::error]: Error state");
      break;
    }
    case WaypointState::ready:
    {
      ROS_INFO("[WaypointController::updateControls::ready]: [P%i]: %s to %s",
               waypoint.header.seq,
               to_string(waypoint_state).c_str(),
               to_string(WaypointState::driving).c_str());
      waypoint_state = WaypointState::initial_angle_correction;
      break;
    }
    case WaypointState::initial_angle_correction:
    {
      //ROS_INFO("[WaypointController::updateControls::initial_angle_correction]: angle = %f",
      //         feedback.theta()/pi*180.0);
      if (waypoint.drive_profile == 3)
      {
        setPoint(0.0, 0.0, waypoint.reverse);
        waypoint_state = WaypointState::driving;
      }
      if (signbit(feedback.theta()) != signbit(last_feedback.theta()))
      {
        ROS_INFO("[WaypointController::updateControls::initial_angle_correction]: %s to %s, angle = %f",
                 to_string(waypoint_state).c_str(),
                 to_string(WaypointState::driving).c_str(),
                 feedback.theta()/pi*180.0);
        setPoint(0.0, 0.0, waypoint.reverse);
        waypoint_state = WaypointState::driving;
      }
      else
      {
        int i = waypoint.drive_profile;
        double duty = clamp(config.driveProfile(i).inPlaceK() * abs(feedback.theta()),
            config.driveProfile(i).minInPlaceDuty(), config.driveProfile(i).maxInPlaceDuty());
        //ROS_INFO("Duty %f", duty);
        if (feedback.theta() > 0.0)
        {
          setPoint(-duty, duty, waypoint.reverse);
        }
        else
        {
          setPoint(duty, -duty, waypoint.reverse);
        }
      }
      break;
    }
    case WaypointState::driving:
    {
      if (signbit(feedback.x()) != signbit(last_feedback.x()))
      {
        ROS_INFO("[WaypointController::updateControls::driving]: %s to %s, dx = %3f, dy = %3f",
                 to_string(waypoint_state).c_str(), to_string(WaypointState::driving).c_str(),
                 feedback.x(), feedback.y());
        setPoint(0.0, 0.0, waypoint.reverse);
        waypoint_state = WaypointState::final_angle_correction;
      }
      else
      {
        int i = waypoint.drive_profile;
        double kx  = config.driveProfile(i).drivingKx();
        double ky  = config.driveProfile(i).drivingKy();
        double min = config.driveProfile(i).minDrivingDuty();
        double max = config.driveProfile(i).maxDrivingDuty();
        double p = clamp(kx * abs(feedback.x()), 0.0, 1.0);
        double high = clamp(p*max, min, max);
        double low = clamp(p*max*(1 - ky * abs(feedback.y())), min, max);
        if (feedback.y() > 0.0)
        {
          setPoint(low, high, waypoint.reverse);
        }
        else
        {
          setPoint(high, low, waypoint.reverse);
        }
      }
      break;
    }
    case WaypointState::angle_correction:
    {
      ROS_WARN("[WaypointController::updateControls::angle_correction]: Not implemented yet");
      break;
    }
    case WaypointState::final_angle_correction:
    {
      //ROS_WARN("[WaypointController::updateControls::final_angle_correction]: Not implemented yet");
      waypoint_state = WaypointState::finished;
      break;
    }
    case WaypointState::finished:
    {
      ROS_INFO("[WaypointController::updateControls::finish]: Finished waypoint %i (%f, %f)",
          waypoint.header.seq, waypoint.pose.x, waypoint.pose.y);
      waypoints.pop_back();
      waypoint_state = WaypointState::ready;
      break;
    }
  }
}

void WaypointController::updateBatteryVoltage()
{
  double sample = (fl->getVin() + fr->getVin() + br->getVin() + bl->getVin()) / 4.0;
  simpleLowPassFilter<double>(battery_voltage, sample, config.batteryFilterK());
  if (battery_voltage < config.minVoltage())
  {
    ROS_WARN("[WaypointController::setPoint]: Vin = %f < %f", battery_voltage, config.minVoltage());
    battery_voltage = config.minVoltage();
  }
}

void WaypointController::setPoint(double left, double right, bool reverse)
{
  // Clamp duty
  double tmp_left, tmp_right;
  tmp_left  = clamp((reverse ? -right : left),  -config.maxDuty(), config.maxDuty());
  tmp_right = clamp((reverse ?  -left : right), -config.maxDuty(), config.maxDuty());
  left  = clampAcceleration(tmp_left,  last_left, config.maxAcceleration(), dt);
  right = clampAcceleration(tmp_right, last_right, config.maxAcceleration(), dt);
  last_left = left;
  last_right = right;

  // Compensate for change in battery voltage
  updateBatteryVoltage();
  if (config.voltageCompensation())
  {
      left = clamp(left * config.fullVoltage() / battery_voltage,
          -1.0*config.maxCompensatedDuty(), config.maxCompensatedDuty());
      right = clamp(right * config.fullVoltage() / battery_voltage,
          -1.0*config.maxCompensatedDuty(), config.maxCompensatedDuty());
  }

  // Set duty
  fl->setDuty((float)left);
  bl->setDuty((float)left);
  fr->setDuty((float)right);
  br->setDuty((float)right);

  debug_info.command.left = left;
  debug_info.command.right = right;
}

void WaypointController::stop()
{
  setPoint(0.0, 0.0);
}

ControlState WaypointController::getControlState() const
{
  return state;
}

size_t WaypointController::remainingWaypoints()
{
  return waypoints.size();
}

Debug WaypointController::getDebugInfo() const
{
  return debug_info;
}

double waypoint_control::clampAcceleration(double value, double last_value, double limit, double dt)
{
  double acceleration = (value - last_value) / dt;
  if (abs(acceleration) > limit)
  {
    return (acceleration >= 0.0 ? 1.0 : -1.0) * limit * dt + last_value;
  }
  return value;
}
