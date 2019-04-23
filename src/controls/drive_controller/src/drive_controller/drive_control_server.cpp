#include <drive_controller/drive_control_server.h>
#include <wheel_params/wheel_params.h>
#include <utilities/joy.h>


using namespace drive_controller;


DriveControlServer::DriveControlServer(ros::NodeHandle *nh, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl) :
  nh(nh), fl(fl), fr(fr), bl(bl), br(br), debug(true), server(*nh, "action", false), direction(true),
  controller(fr, fl, bl, br), teleop(TeleopInterface::Mode::velocity, 0.4, fl, fr, br, bl),
  state(ControlState::manual), manual_safety(false), autonomy_safety(false), teleop_left(0.0f), teleop_right(0.0f), seq(0)
{
  joint_angles.header.stamp = ros::Time::now();
  joint_angles.header.seq = seq;
  joint_angles.name.emplace_back("front_left_wheel_joint");
  joint_angles.name.emplace_back("front_right_wheel_joint");
  joint_angles.name.emplace_back("back_right_wheel_joint");
  joint_angles.name.emplace_back("back_left_wheel_joint");
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);
  joint_angles.position.emplace_back(0.0);

  nh->param<float>("max_velocity", max_velocity, 0.4);
  teleop.setMax(max_velocity);

  joy_subscriber = nh->subscribe("/joy", 1, &DriveControlServer::joyCallback, this);
  state_subscriber = nh->subscribe("/state_vector", 1, &DriveControlServer::stateVectorCallback, this);
  joint_publisher = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
  server.registerGoalCallback(boost::bind(&DriveControlServer::goalCallback, this));
  server.registerPreemptCallback(boost::bind(&DriveControlServer::preemptCallback, this));
  server.start();
}

DriveControlResult drive_controller::toResult(ControlState state)
{
  DriveControlResult result;
  result.control_state = (DriveControlResult::_control_state_type)state;
  return result;
}

ControlState drive_controller::toControlState(DriveControlGoal goal)
{
  return (ControlState)goal.control_state;
}

void DriveControlServer::goalCallback()
{
  auto goal = server.acceptNewGoal();
  ControlState request = toControlState(*goal);

  ROS_INFO("[DriveControlServer::goalCallback] Request for %s to %s",
           to_string(state).c_str(), to_string(request).c_str());
  switch (request)
  {
    case ControlState::new_goal:
    {
      // Only accepts one segment
      direction = goal->path[0].direction_of_travel == 1;
      path = toBezierPath(goal->path[0]);
      modified_path = path;
      controller.cleanPath(&modified_path, x, y, theta, false);
      controller.addPath(modified_path, direction);
      state = ControlState::in_progress;
      server.setSucceeded(toResult(state));
      break;
    }
    case ControlState::cancel:
    {
      state = ControlState::ready;
      server.setSucceeded(toResult(state));
      stop();
    }
    case ControlState::manual:
    {
      state = ControlState::manual;
      server.setSucceeded(toResult(state));
      break;
    }
    default:
    {
      state = ControlState::error;
      stop();
      server.setAborted(toResult(state));
      ROS_ERROR("[DriveControlServer::goalCallback] Unable to set control state from %s to %s",
                to_string(state).c_str(), to_string(request).c_str());
      break;
    }

  }
}

void DriveControlServer::preemptCallback()
{
  server.setPreempted(toResult(state));
  ROS_INFO("[DigControlServer::preemptCallback] Preempting from %s", to_string(state).c_str());
}

void DriveControlServer::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  using utilities::Joy;
  Joy joy(joy_msg);

  manual_safety = joy.get(Joy::MANUAL_SAFETY);
  autonomy_safety = joy.get(Joy::AUTONOMY_SAFETY);
  if (manual_safety)
  {
    teleop_left  = max_velocity * (float)joy.get(Joy::TELEOP_LEFT);
    teleop_right = max_velocity * (float)joy.get(Joy::TELEOP_RIGHT);
  }
  else
  {
    teleop_left = 0.0f;
    teleop_left = 0.0f;
  }
}

void DriveControlServer::update(double dt)
{
  seq++;

  // Handle state
  switch (state)
  {
    case ControlState::error:
    {
      ROS_ERROR("[DriveControlServer::update] In error state");
      stop();
      break;
    }
    case ControlState::ready:
    {
      // Do nothing
      stop();
      break;
    }
    case ControlState::in_progress:
    {
      if (autonomy_safety)
      {
        controller.update(state_vector, dt);
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
        teleop.update(teleop_left, teleop_right);
      }
      else
      {
        teleop.update(0.0f, 0.0f);
      }
      break;
    }
    case ControlState::new_goal:
    case ControlState::cancel:
    {
      ROS_ERROR("[DriveControlServer::update] Shouldn't be in control state from %s", to_string(state).c_str());
      stop();
    }
  }

  // Visuals
  joint_angles.header.stamp = ros::Time::now();
  joint_angles.header.seq = seq;
  joint_angles.position[0] += fl->getRadialVelocity() * dt;
  joint_angles.position[1] += fr->getRadialVelocity() * dt;
  joint_angles.position[2] += br->getRadialVelocity() * dt;
  joint_angles.position[3] += bl->getRadialVelocity() * dt;
  joint_publisher.publish(joint_angles);
}

void DriveControlServer::stop()
{
  controller.haltAndAbort();
}

void DriveControlServer::stateVectorCallback(const localization::StateVector::ConstPtr &state_vector_msg)
{
  state_vector.x_pos = state_vector_msg->x_pos;
  state_vector.y_pos = state_vector_msg->y_pos;
  state_vector.theta = state_vector_msg->theta;
  state_vector.x_vel = state_vector_msg->x_vel;
  state_vector.y_vel = state_vector_msg->y_vel;
  state_vector.omega = state_vector_msg->omega;
  state_vector.x_accel = state_vector_msg->x_accel;
  state_vector.y_accel = state_vector_msg->y_accel;
  state_vector.alpha   = state_vector_msg->alpha;
}

std::string drive_controller::to_string(ControlState state)
{
  switch (state)
  {
    case ControlState::error:
      return "error";
    case ControlState::ready:
      return "ready";
    case ControlState::new_goal:
      return "new_goal";
    case ControlState::in_progress:
      return "in_progress";
    case ControlState::cancel:
      return "cancel";
    case ControlState::manual:
      return "manual";
  }
}

bezier_path drive_controller::toBezierPath(const navigation_msgs::BezierSegment &segment)
{
  bezier_path path;
  path.x1 = segment.p0.x;
  path.y1 = segment.p0.y;
  path.x2 = segment.p1.x;
  path.y2 = segment.p1.y;
  path.x3 = segment.p2.x;
  path.y3 = segment.p2.y;
  path.x4 = segment.p3.x;
  path.y4 = segment.p3.y;
  return path;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "drive_control_server");
  ros::NodeHandle nh("~");

  iVescAccess *fl, *fr, *br, *bl;
  fl = new VescAccess(front_left_param);
  fr = new VescAccess(front_right_param);
  br = new VescAccess(back_right_param);
  bl = new VescAccess(back_left_param);
  DriveControlServer server(&nh, fl, fr, br, bl);
  ros::Rate rate(50);
  double dt = rate.expectedCycleTime().toSec();
  while (ros::ok())
  {
    server.update(dt);
    ros::spinOnce();
    rate.sleep();
  }
  server.stop();
}
