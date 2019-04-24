#include <waypoint_control/waypoint_controller.h>
#include <boost/algorithm/clamp.hpp>

using namespace waypoint_control;

using boost::algorithm::clamp;

WaypointController::WaypointController(iVescAccess *front_left, iVescAccess *front_right,
    iVescAccess *back_right, iVescAccess *back_left, double max_velocity) :
    fl(front_left), fr(front_right), br(back_right), bl(back_left),
    max_velocity(max_velocity), state(ControlState::manual), waypoint_state(WaypointState::ready)
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
      ROS_WARN("Flushing current waypoints");
      this->waypoints.clear();
    }
    this->waypoints = waypoints;
    this->state = ControlState::in_progress;
    this->waypoint_state = WaypointState::initial_angle_correction;
    ROS_INFO("[WaypointController::setControlState]: %s to %s",
        to_string(this->state).c_str(), to_string(state).c_str());
  }
  else
  {
    this->state = ControlState::error;
    ROS_WARN("[WaypointController::setControlState]: Invalid transition %s to %s",
             to_string(this->state).c_str(), to_string(state).c_str());
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

      }
      else
      {
        stop();
      }
      ROS_WARN("Not implemented yet");
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

void WaypointController::setPoint(double left, double right)
{
  fl->setLinearVelocity((float)clamp(left,  -max_velocity, max_velocity));
  bl->setLinearVelocity((float)clamp(left,  -max_velocity, max_velocity));
  fr->setLinearVelocity((float)clamp(right, -max_velocity, max_velocity));
  br->setLinearVelocity((float)clamp(right, -max_velocity, max_velocity));
}

void WaypointController::stop()
{
  setPoint(0.0, 0.0);
}

ControlState WaypointController::getControlState() const
{
  return state;
}
