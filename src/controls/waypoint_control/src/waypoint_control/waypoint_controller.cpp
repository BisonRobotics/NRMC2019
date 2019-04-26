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


WaypointController::WaypointController(iVescAccess *front_left, iVescAccess *front_right,
    iVescAccess *back_right, iVescAccess *back_left, Config *config) :
    fl(front_left), fr(front_right), br(back_right), bl(back_left), config(config),
    state(ControlState::manual), waypoint_state(WaypointState::ready), last_left(0.0), last_right(0.0)
{
  dt = 1.0/config->rate;
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


void WaypointController::update(bool manual_safety, bool autonomy_safety,
    tf2::Transform transform, double left, double right)
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
          state = ControlState::ready;
        }
        else
        {
          debug_info.waypoint = waypoints.back();
          updateControls(transform);
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

  // Update debug info
  debug_info.command_state = to_string(state);
  debug_info.waypoint_state = to_string(waypoint_state);
  debug_info.feedback.dx = feedback.x();
  debug_info.feedback.dy = feedback.y();
  debug_info.feedback.dr = feedback.r();
  debug_info.feedback.tr = feedback.theta();
  debug_info.feedback.td = feedback.theta()/pi*180.0;
  debug_info.transform = tf2::toMsg(transform);
  this->last_transform = transform;
}

void WaypointController::updateControls(const tf2::Transform &transform)
{
  if (waypoints.empty())
  {
    ROS_WARN("[WaypointController::updateControls::empty]: No waypoints");
    return;
  }

  Waypoint waypoint = waypoints.back();
  last_feedback = feedback;
  feedback = Feedback(transform, waypoint);

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
        double duty = clamp(config->in_place_k * abs(feedback.theta()),
            config->min_in_place_duty, config->max_in_place_duty);
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
        double max = config->max_driving_duty;
        double k = config->driving_k;
        double brake = clamp(max * (1 - k * abs(feedback.y())),
                             config->min_driving_duty, config->max_driving_duty);
        if (feedback.y() > 0.0)
        {

          setPoint(brake, max, waypoint.reverse);
        }
        else
        {
          setPoint(max, brake, waypoint.reverse);
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
          waypoint.header.seq, waypoint.pose.position.x, waypoint.pose.position.y);
      waypoints.pop_back();
      waypoint_state = WaypointState::ready;
      break;
    }
  }
}




void WaypointController::setPoint(double left, double right, bool reverse)
{
  left  = clampAcceleration( left,  last_left, config->max_acceleration, dt);
  right = clampAcceleration(right, last_right, config->max_acceleration, dt);

  if (state == ControlState::manual)
  {
    if (std::abs(left) > 1.0e-3)
    {
      left  = clamp(left, -config->max_manual_duty, config->max_manual_duty);
      fl->setDuty((float)left);
      bl->setDuty((float)left);
      debug_info.command.left = left;
    }
    else
    {
      left = 0.0;
      fl->setTorque(0.0f);
      bl->setTorque(0.0f);
      debug_info.command.left = 0.0;
    }
    if (std::abs(right) > 1.0e-3)
    {
      right  = clamp(right, -config->max_manual_duty, config->max_manual_duty);
      fr->setDuty((float)right);
      br->setDuty((float)right);
      debug_info.command.right = right;
    }
    else
    {
      right = 0.0;
      fr->setTorque(0.0f);
      br->setTorque(0.0f);
      debug_info.command.right = 0.0;
    }
  }
  else
  {
    // Adjust sign and clamp
    left  = clamp(reverse ?  -left : left,  -config->max_duty, config->max_duty);
    right = clamp(reverse ? -right : right, -config->max_duty, config->max_duty);
    fl->setDuty((float)left);
    bl->setDuty((float)left);
    fr->setDuty((float)right);
    br->setDuty((float)right);
    debug_info.command.left = left;
    debug_info.command.right = right;
  }
  last_left = left;
  last_right = right;
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
    return (acceleration > 0.0 ? 1.0 : -1.0) * limit * dt + last_value;
  }
  return value;
}