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


WaypointController::WaypointController(iVescAccess *front_left, iVescAccess *front_right,
    iVescAccess *back_right, iVescAccess *back_left, Config *config) :
    fl(front_left), fr(front_right), br(back_right), bl(back_left), config(config),
    state(ControlState::manual), waypoint_state(WaypointState::ready)
{

}

void WaypointController::setControlState(ControlState state)
{
  stop();
  switch (state)
  {
    case ControlState::ready:
    case ControlState::manual:
    {
      this->state = state;
      ROS_INFO("[WaypointController::setControlState]: %s to %s",
               to_string(this->state).c_str(), to_string(state).c_str());
      break;
    }
    case ControlState::cancel:
    {
      this->state = ControlState::ready;
      ROS_INFO("[WaypointController::setControlState]: %s to %s",
               to_string(this->state).c_str(), to_string(state).c_str());
      break;
    }
    case ControlState::error:
    case ControlState::new_goal:
    case ControlState::in_progress:
    {
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
    case ControlState::cancel:
    case ControlState::new_goal:
    case ControlState::error:
    {
      stop();
      break;
    }
  }
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
      ROS_INFO("[WaypointController::updateControls::initial_angle_correction]: angle = %f",
               feedback.theta()/pi*180.0);
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
        double duty = clamp(config->max_in_place_duty, config->min_in_place_duty, config->max_in_place_duty);
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
      ROS_INFO("[WaypointController::updateControls::driving]: theta = %f", feedback.theta()/pi*180.0);
      if (signbit(feedback.x()) != signbit(last_feedback.x()))
      {
        ROS_INFO("[WaypointController::updateControls::driving]: %s to %s, dx = %3f, dy = %3f",
                 to_string(waypoint_state).c_str(),
                 to_string(WaypointState::driving).c_str(),
                 feedback.x(),
                 feedback.y());
        setPoint(0.0, 0.0, waypoint.reverse);
        waypoint_state = WaypointState::final_angle_correction;
      }
      else
      {
        ROS_INFO("[WaypointController::updateControls::driving]: dx = %3f, dy = %3f",
                 feedback.x(),
                 feedback.y());
        double duty = clamp(config->max_driving_duty, config->min_driving_duty, config->max_driving_duty);
        if (feedback.y() > 0.0)
        {
          setPoint(duty, duty, waypoint.reverse);
        }
        else
        {
          setPoint(duty, duty, waypoint.reverse);
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
  if (state == ControlState::manual)
  {
    if (std::abs(left) > 1.0e-6)
    {
      left  = clamp(left, -config->max_manual_duty, config->max_manual_duty);
      fl->setDuty((float)left);
      bl->setDuty((float)left);
    }
    else
    {
      fl->setTorque(0.0f);
      bl->setTorque(0.0f);
    }
    if (std::abs(right) > 1.0e-6)
    {
      right  = clamp(right, -config->max_manual_duty, config->max_manual_duty);
      fr->setDuty((float)right);
      br->setDuty((float)right);
    }
    else
    {
      fr->setTorque(0.0f);
      br->setTorque(0.0f);
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
  }
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
