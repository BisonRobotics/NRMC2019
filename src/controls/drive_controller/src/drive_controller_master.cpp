#include <ros/ros.h>
#include <ros/console.h> //for debug/info
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <drive_controller/drive_controller.h>
#include <actionlib/server/simple_action_server.h>
#include <navigation_msgs/FollowPathAction.h>
#include <navigation_msgs/Point2D.h>
#include <occupancy_grid/bezier.h>

#include <geometry_msgs/Pose2D.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Float64.h>

#include <driver_access/driver_access_interface.h>
#include <vrep_driver_access/vrep_driver_access.h>
#include <ros_driver_access/ros_driver_access.h>
#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <wheel_params/wheel_params.h>

#include <super_localizer/super_localizer.h>
#include <sensor_msgs/JointState.h>

#include <visualization_msgs/Marker.h>
//#include <imperio/DriveStatus.h>

#include <sim_robot/sim_robot.h>

#include <lp_research/lpresearchimu.h>
#include <apriltag_tracker_interface/apriltag_tracker_interface.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <utility>

#define UPDATE_RATE_HZ 50.0
bool newWaypointHere = false;
bool firstWaypointHere = true;
bool halt = false;
bool doing_path = false;

using navigation_msgs::FollowPathAction;
using navigation_msgs::FollowPathGoalConstPtr;
using navigation_msgs::FollowPathFeedback;
using navigation_msgs::FollowPathResult;
using navigation_msgs::BezierSegment;
using actionlib::SimpleActionServer;
using occupancy_grid::Bezier;

DriveController_ns::bezier_path curr_path; //eventually will be a vector of these.

SimpleActionServer<FollowPathAction> *server;

void newGoalCallback(const FollowPathGoalConstPtr &goal) //technically called in another thread
{
/*
  curr_path.x1 = 1;
  curr_path.y1 = 1;
  curr_path.x2 = 2; 
  curr_path.y2 = 1;
  curr_path.x3 = 2.5;
  curr_path.y3 = .3;
  curr_path.x4 = 2;
  curr_path.y4 = -.3;
  newWaypointHere = true;
*/
 ROS_INFO("[action_server] Moving toward goal");
  //ros::Rate rate(1.0);

  // Get path
  BezierSegment segment = goal->path[0];
  curr_path.x1 = segment.p0.x;
  curr_path.y1 = segment.p0.y;
  curr_path.x2 = segment.p1.x;
  curr_path.y2 = segment.p1.y;
  curr_path.x3 = segment.p2.x;
  curr_path.y3 = segment.p2.y;
  curr_path.x4 = segment.p3.x;
  curr_path.y4 = segment.p3.y;

  newWaypointHere = true;
}

geometry_msgs::TransformStamped create_tf(double x, double y, double theta, tf2::Quaternion imu_orientation, double z)
{
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  if (!firstWaypointHere)
  {
    transform.transform.translation.x = .6;//x;
    transform.transform.translation.y = 0;//y;
    transform.transform.translation.z =z;
    tf2::Quaternion robot_orientation;
    robot_orientation.setRPY(0, 0, 0);//theta);
    transform.transform.rotation.x = robot_orientation.x();
    transform.transform.rotation.y = robot_orientation.y();
    transform.transform.rotation.z = robot_orientation.z();
    transform.transform.rotation.w = robot_orientation.w();
  }
  else
  {
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z =z;
    tf2::Quaternion robot_orientation;
    robot_orientation.setRPY(0.0, 0.0, theta);
    robot_orientation = robot_orientation * imu_orientation;
    transform.transform.rotation.x = robot_orientation.x();
    transform.transform.rotation.y = robot_orientation.y();
    transform.transform.rotation.z = robot_orientation.z();
    transform.transform.rotation.w = robot_orientation.w();
  }
  return transform;
}

geometry_msgs::TransformStamped create_sim_tf(double x, double y, double theta)
{
  geometry_msgs::TransformStamped tfStamp;
  tfStamp.header.stamp = ros::Time::now();
  tfStamp.header.frame_id = "map";
  tfStamp.child_frame_id = "sim_base_link";
  tfStamp.transform.translation.x = x;
  tfStamp.transform.translation.y = y;
  tfStamp.transform.translation.z = 0.5;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  tfStamp.transform.rotation.x = q.x();
  tfStamp.transform.rotation.y = q.y();
  tfStamp.transform.rotation.z = q.z();
  tfStamp.transform.rotation.w = q.w();
  return tfStamp;
}

void haltCallback(const std_msgs::Empty::ConstPtr &msg)
{
  halt = true;
}

class DriverVescCrossover : public iVescAccess
{
   private:
    driver_access::VREPDriverAccess *face;
    iVescAccess *vesc;
   public:
    DriverVescCrossover(driver_access::VREPDriverAccess *f, iVescAccess *v)
    :face(f), vesc(v) {}
    void setLinearVelocity(float meters_per_second) 
      {face->setVelocity(meters_per_second); vesc->setLinearVelocity(meters_per_second);}
    void setTorque(float current) 
      {face->setEffort(current); vesc->setTorque(current);}
    float getLinearVelocity(void) 
      {return (true ? face->getVelocity() : vesc->getLinearVelocity());} // for now, needs to come from the vesc, o/w the control system doesn't work
    float getTorque(void) {return face->getEffort();}
    nsVescAccess::limitSwitchState getLimitSwitchState(void) 
      {return nsVescAccess::limitSwitchState::inTransit;}
    float getPotPosition(void) 
      {return face->getPosition();}
    void setDuty(float d) {vesc->setDuty(d);}
};

int main(int argc, char **argv)
{
  // read ros param for simulating
  ros::init(argc, argv, "my_tf2_listener");
if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}

  ros::NodeHandle node("~");
  ros::NodeHandle globalNode;
  ros::Rate rate(UPDATE_RATE_HZ);
  ros::Rate vesc_init_rate (10);
  bool simulating;
  if (node.hasParam("simulating_driving"))
  {
    node.getParam("simulating_driving", simulating);
    if (simulating)
    {
      ROS_WARN("\nRUNNING DRIVING AS SIMULATION\n");
    }
    else
    {
      ROS_WARN("\nRUNNING DRIVING AS PHYSICAL\n");
    }
  }
  else
  {
    ROS_ERROR("\n\nsimulating_driving param not defined! aborting.\n\n");
    return -1;
  }

  ros::Subscriber sub = node.subscribe("additional_path", 100, newGoalCallback);

  ros::Publisher jspub = globalNode.advertise<sensor_msgs::JointState>("joint_states", 500);

  double settle_time;
  if(!node.getParam("localization_settling_time", settle_time))
  {
    settle_time = 5;
    ROS_INFO_STREAM ("localization settling time " << settle_time);
  }

  tf2_ros::Buffer tfBuffer;
  geometry_msgs::TransformStamped transformStamped;

  SimRobot *sim;
  iVescAccess *fl, *fr, *br, *bl;
  driver_access::VREPDriverAccess *dfl, *dfr, *dbr, *dbl;
  ImuSensorInterface *imu;
  PosSensorInterface *pos;

  // initialize the localizer here
  ros::Time lastTime = ros::Time::now();
  ros::Time currTime;

  if (simulating)
  {
    sim = new SimRobot(0, 0, 0, .1);//.5, 1, .7, .1); //axel len, x, y, theta //this is temporary, its needed for the imu and pos
    pos = new AprilTagTrackerInterface("/vrep/pose", .1);

    driver_access::Limits limits(0, 0, 0, 1, 0, 1);
    dfl = new driver_access::VREPDriverAccess(limits, driver_access::ID::front_left_wheel,  driver_access::Mode::velocity);
    dfr = new driver_access::VREPDriverAccess(limits, driver_access::ID::front_right_wheel, driver_access::Mode::velocity);
    dbr = new driver_access::VREPDriverAccess(limits, driver_access::ID::back_right_wheel,  driver_access::Mode::velocity);
    dbl = new driver_access::VREPDriverAccess(limits, driver_access::ID::back_left_wheel,   driver_access::Mode::velocity);
    fl = new DriverVescCrossover(dfl, sim->getFLVesc());
    fr = new DriverVescCrossover(dfr, sim->getFRVesc());
    br = new DriverVescCrossover(dbr, sim->getBRVesc());
    bl = new DriverVescCrossover(dbl, sim->getBLVesc());

    imu = sim->getImu(); //if these can be replaced, we can get rid of the SimRobot
    //pos = sim->getPos();
  }
  else
  {
    sim = NULL;  // Make no reference to the sim if not simulating
    bool no_except = false;
    while (!no_except  && ros::ok()) {
      try {
        fl = new VescAccess(front_left_param);
        fr = new VescAccess(front_right_param);
        br = new VescAccess(back_right_param);
        bl = new VescAccess(back_left_param);
        no_except = true;
      } catch (VescException e) {
        ROS_WARN("%s",e.what ());
        no_except = false;
      }
      if (!no_except && (ros::Time::now() - lastTime).toSec() > 10){
        ROS_ERROR ("Vesc exception thrown for more than 10 seconds");
        ros::shutdown ();
      }
      vesc_init_rate.sleep();
    }
    //these will need updated with new sensors, using the same interface
    //pos = new AprilTagTrackerInterface("/pose_estimate_filter/pose_estimate", .1);
    //imu = new LpResearchImu("imu_base_link");
  }


  std::vector<driver_access::DriverAccess*> drivers = {dfl, dfr, dbr, dbl};
  driver_access::ROSDriverAccess ros_drivers(drivers);

  ros::Duration loopTime;
  bool firstTime = true;
  tf2_ros::TransformBroadcaster tfBroad;
  std::vector<std::pair<double, double> > waypoint_set;
  SuperLocalizer superLocalizer(ROBOT_AXLE_LENGTH, 1, .5, 0, fl, fr, br, bl, /*imu,*/ pos, SuperLocalizer_default_gains);
  
  LocalizerInterface::stateVector stateVector;
  ros::Subscriber haltsub = node.subscribe("halt", 100, haltCallback);

  double wheel_positions[4] = { 0 };

  geometry_msgs::Point vis_point;
  // hang here until someone knows where we are
  ROS_INFO("Going into wait loop for localizer and initial theta...");

  ros::Duration idealLoopTime(1.0 / UPDATE_RATE_HZ);

  while ((!superLocalizer.getIsDataGood() && ros::ok()))
  {
    // do initial localization
    if (firstTime)
    {
      firstTime = false;
      currTime = ros::Time::now();
      lastTime = currTime - idealLoopTime;
      loopTime = (currTime - lastTime);
    }
    else
    {
      lastTime = currTime;
      currTime = ros::Time::now();
      loopTime = (currTime - lastTime);
    }
    if (simulating)
    {
      sim->update((loopTime).toSec());
      tfBroad.sendTransform(create_sim_tf(sim->getX(), sim->getY(), sim->getTheta()));
    }
    superLocalizer.updateStateVector(loopTime.toSec());

    ros::spinOnce();
    rate.sleep();
    if (imu->receiveData() == ReadableSensors::ReadStatus::READ_FAILED)
    {
        ROS_WARN ("BAD IMU DATA!");
    }
    if (pos->receiveData()==ReadableSensors::ReadStatus::READ_FAILED)
    {
        ROS_WARN ("BAD POS");
    }
  }

  lastTime = ros::Time::now ();
  while (((ros::Time::now()-lastTime).toSec()<settle_time) && (ros::ok()))
  {
    superLocalizer.updateStateVector(.02);
    rate.sleep();
  }

  ROS_INFO ("Localization Settled!");
  ros::spinOnce ();

  double init_angle = pos->getTheta();
  int init_y = (pos->getY() > 0) ? 1 : -1;

  fl->setLinearVelocity(0);
  fr->setLinearVelocity(0);
  bl->setLinearVelocity(0);
  br->setLinearVelocity(0);
  stateVector = superLocalizer.getStateVector();
  tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta, imu->getOrientation(), pos->getZ()));

  ros::spinOnce();

  fr->setLinearVelocity(0);
  br->setLinearVelocity(0);
  bl->setLinearVelocity(0);
  fl->setLinearVelocity(0);

  tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta, imu->getOrientation(), pos->getZ()));
  ros::spinOnce ();

  DriveController dc = DriveController(fr, fl, bl, br);
  ROS_INFO("DC Init");

  server = new SimpleActionServer<FollowPathAction>(globalNode, "follow_path", &newGoalCallback, false);
  server->start();
  ROS_INFO("[action_server] Started");

  firstTime = true;
  while (ros::ok())
  {
    ROS_DEBUG("\n");
    ROS_DEBUG("Top");
    // update localizer here
    if (firstTime)
    {
      firstTime = false;
      currTime = ros::Time::now();
      lastTime = currTime - idealLoopTime;
      loopTime = currTime - lastTime;
    }
    else
    {
      lastTime = currTime;
      currTime = ros::Time::now();
      loopTime = currTime - lastTime;
    }
    ROS_DEBUG("Looptime : %.5f", loopTime.toSec());
    if (simulating)
    {
      sim->update(loopTime.toSec());
      //this would be handled by the new sim? sending tf's to rviz
      tfBroad.sendTransform(create_sim_tf(pos->getX(), pos->getY(), pos->getTheta()));
    }

    superLocalizer.updateStateVector(loopTime.toSec());
    stateVector = superLocalizer.getStateVector();
    DriveController_ns::robot_state_vector sv;
    sv.x = stateVector.x_pos;
    sv.y = stateVector.y_pos;
    sv.theta = stateVector.theta;

    tfBroad.sendTransform(create_tf(stateVector.x_pos, stateVector.y_pos, stateVector.theta, imu->getOrientation(), pos->getZ()));

    ros_drivers.publish();

   if (newWaypointHere)
   {
       dc.addPath(curr_path);
       newWaypointHere = false;
   }
    // update controller
     dc.update(sv, loopTime.toSec());

  ROS_INFO("Paths on Stack: %d", dc.getPPaths());
  if (dc.getPPaths() >=1)
  {
    // Provide feedback
    FollowPathFeedback feedback;
    feedback.deviation = 0.01;
    feedback.progress = dc.getPClosestT();
    server->publishFeedback(feedback);
    doing_path = true;
   }
   else if (doing_path && dc.getPPaths() ==0/*last PPaths was >1 but this one is zero*/)
  {

    // Publish result
    FollowPathResult result;
    result.pose.position.x = sv.x;
    result.pose.position.y = sv.y;
    result.pose.position.z = 0.0;

    result.pose.orientation.w = std::cos(.5*sv.theta);
    result.pose.orientation.x = 0;
    result.pose.orientation.y = 0;
    result.pose.orientation.z = std::sin(.5*sv.theta);
    result.status = 0;
    server->setSucceeded(result);
    doing_path = false;
  }

    if (halt)
    {
      dc.haltAndAbort();
      break;
    }
    // ros end stuff
    ros::spinOnce();
    rate.sleep();
  }
  delete fr;
  delete fl;
  delete br;
  delete bl;
  if (simulating)
  {
    delete sim;
  }
  else
  {
    delete imu;
    delete pos;
  }
}
