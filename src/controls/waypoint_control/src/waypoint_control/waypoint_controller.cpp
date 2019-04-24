#include <waypoint_control/waypoint_controller.h>
#include <boost/algorithm/clamp.hpp>

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

void WaypointController::updateControls(const tf2::Transform &transform)
{
  if (waypoints.empty())
  {
    waypoint_state = WaypointState::ready;
    return;
  }

  switch (waypoint_state)
  {
    case WaypointState::error:
    {
      ROS_ERROR("[WaypointController::updateControls]: Error state");
      break;
    }
    case WaypointState::ready:
    {
      if (!waypoints.empty())
      {
        waypoint_state = WaypointState::starting_orientation;
      }
      break;
    }
    case WaypointState::starting_orientation:
    {
      Waypoint waypoint = waypoints.front();
      angular_error = getAngularError(transform, waypoint);

      if (angular_error.smallestAngle() > config.initial_angular_variation)
      {
        setPoint(config.max_angular_velocity, -config.max_angular_velocity);
      }
      else if (angular_error.smallestAngle() < -config.initial_angular_variation)
      {
        setPoint(-config.max_angular_velocity, config.max_angular_velocity);
      }
      else
      {
        ROS_INFO("Made it!");
      }
      break;
    }
    case WaypointState::initial_angle_correction:
    case WaypointState::driving:
    case WaypointState::angle_correction:
    case WaypointState::final_angle_correction:
    {
      break;
    }
  }
}

/*
 * A   = position vector of robot
 * B   = position vector of goal position
 * C   = is the vector that goes from A to B
 * th1 = angular difference between A and B with respect to the map's orientation
 * th2 = angle of the robot with respect to the map's orientation
 */
Rotation2D waypoint_control::getAngularError(const tf2::Transform &current, const Waypoint &waypoint)
{
  double dy = waypoint.pose.position.y - current.getOrigin().y();
  double dx = waypoint.pose.position.x - current.getOrigin().x();
  Rotation2D T1(std::atan2(dy, dx));

  tf2::Matrix3x3 C(current.getRotation());
  double roll, pitch, yaw;
  C.getRPY(roll, pitch, yaw);
  Rotation2D T2(-yaw);

  return T1*T2;
}


void WaypointController::setPoint(double left, double right)
{
if (state == ControlState::manual)
{
if (std::abs(left) > 0.001f)
{
  fl->setLinearVelocity((float)clamp(left, -max_velocity, max_velocity));
  bl->setLinearVelocity((float)clamp(left, -max_velocity, max_velocity));
}
else
{
  fl->setTorque(0.0f);
  bl->setTorque(0.0f);
}
if (std::abs(right) > 0.001f)
{
  fr->setLinearVelocity((float)clamp(right, -max_velocity, max_velocity));
  br->setLinearVelocity((float)clamp(right, -max_velocity, max_velocity));
}
else
{
  fr->setTorque(0.0f);
  br->setTorque(0.0f);
}
}
else
{
fl->setLinearVelocity((float)clamp(left, -max_velocity, max_velocity));
bl->setLinearVelocity((float)clamp(left, -max_velocity, max_velocity));
fr->setLinearVelocity((float)clamp(right, -max_velocity, max_velocity));
br->setLinearVelocity((float)clamp(right, -max_velocity, max_velocity));
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
