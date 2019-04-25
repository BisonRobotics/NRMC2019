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
    this->waypoint_state = WaypointState::starting_orientation;
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
        updateControls(transform);
      }
      else
      {
        ROS_WARN("[WaypointController::update]: Stopped autonomy");
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

  Waypoint waypoint = waypoints.front();
  linear_error;
  angular_error = getAngularError(transform, waypoint);

  switch (waypoint_state)
  {
    case WaypointState::error:
    {
      ROS_ERROR("[WaypointController::updateControls::error]: Error state");
      break;
    }
    case WaypointState::ready:
    {
      ROS_WARN("[WaypointController::updateControls::ready]: Should be in a different state");
      break;
    }
    case WaypointState::starting_orientation:
    {
      ROS_INFO("[WaypointController::updateControls::starting_orientation]: %s to %s, theta = %f",
               to_string(waypoint_state).c_str(),
               to_string(WaypointState::initial_angle_correction).c_str(),
               angular_error.smallestAngle());
      break;
    }
    case WaypointState::initial_angle_correction:
    {
      ROS_INFO("[WaypointController::updateControls::initial_angle_correction]: theta = %f",
               angular_error.smallestAngle());
      if (signbit(angular_error.smallestAngle()) == signbit(last_angular_error.smallestAngle()))
      {
        setPoint(0.0, 0.0);
        ROS_INFO("Made it!");
      }
      else
      {
        if (angular_error.smallestAngle() > 0.0)
        {
          setPoint(config->max_angular_velocity, -config->max_angular_velocity, waypoint.reverse);
        }
        else
        {
          setPoint(-config->max_angular_velocity, config->max_angular_velocity, waypoint.reverse);
        }
      }
      break;
    }
    case WaypointState::driving:
    {
      ROS_INFO("[WaypointController::updateControls::driving]: theta = %f", angular_error.smallestAngle());
      if (signbit(angular_error.smallestAngle()) == signbit(last_angular_error.smallestAngle()))
      {
        setPoint(0.0, 0.0);
        ROS_INFO("Made it!");
      }
      else
      {
        if (angular_error.smallestAngle() > 0.0)
        {
          setPoint(config->max_angular_velocity, config->max_angular_velocity, waypoint.reverse);
        }
        else
        {
          setPoint(-config->max_angular_velocity, config->max_angular_velocity, waypoint.reverse);
        }
      }
      break;
    }
    case WaypointState::angle_correction:
    case WaypointState::final_angle_correction:
    {
      ROS_WARN("[WaypointController::updateControls]: Not implemented yet");
      break;
    }
  }
  last_angular_error = angular_error;
}

/*
 * A   = position vector of robot
 * B   = position vector of goal position
 * C   = is the vector that goes from A to B
 * th1 = angular difference between A and B with respect to the map's orientation
 * th2 = angle of the robot with respect to the map's orientation
 */
Rotation2D waypoint_control::getAngularError(const tf2::Transform &current, const Waypoint &waypoint, bool reverse)
{
  Rotation2D T1, T2, T3;
  double dy = waypoint.pose.position.y - current.getOrigin().y();
  double dx = waypoint.pose.position.x - current.getOrigin().x();
  T1 = Rotation2D(std::atan2(dy, dx));

  tf2::Matrix3x3 O(current.getRotation());
  double roll, pitch, yaw;
  O.getRPY(roll, pitch, yaw);
  T2 = Rotation2D(-yaw);
  T3 = Rotation2D((reverse) ? pi : 0.0);

  return T1*T2*T3;
}


void WaypointController::setPoint(double left, double right, bool reverse)
{
  // Adjust sign and clamp
  left  = clamp(reverse ?  -left : left,  -config->max_velocity, config->max_velocity);
  right = clamp(reverse ? -right : right, -config->max_velocity, config->max_velocity);
  if (state == ControlState::manual)
  {
    if (std::abs(left) > 0.001f)
    {
      fl->setLinearVelocity((float)left);
      bl->setLinearVelocity((float)left);
    }
    else
    {
      fl->setTorque(0.0f);
      bl->setTorque(0.0f);
    }
    if (std::abs(right) > 0.001f)
    {
      fr->setLinearVelocity((float)right);
      br->setLinearVelocity((float)right);
    }
    else
    {
      fr->setTorque(0.0f);
      br->setTorque(0.0f);
    }
  }
  else
  {
    fl->setLinearVelocity((float)left);
    bl->setLinearVelocity((float)left);
    fr->setLinearVelocity((float)right);
    br->setLinearVelocity((float)right);
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
