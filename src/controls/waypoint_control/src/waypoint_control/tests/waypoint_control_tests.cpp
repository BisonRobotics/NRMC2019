#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <waypoint_control/waypoint_controller.h>
#include <boost/math/constants/constants.hpp>

using boost::math::double_constants::pi;

using namespace waypoint_control;

TEST(WaypointControlTests, getAngularErrorGoalPosition)
{
  tf2::Transform current; // Origin with no rotation
  current.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  Waypoint goal;

  goal.pose.position.x =  1.0;
  goal.pose.position.y =  0.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), 0.0, 1e-3);

  goal.pose.position.x =  1.0;
  goal.pose.position.y =  1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), pi/4, 1e-3);

  goal.pose.position.x =  0.0;
  goal.pose.position.y =  1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), pi/2, 1e-3);


  goal.pose.position.x = -1.0;
  goal.pose.position.y =  1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), 3*pi/4, 1e-3);

  goal.pose.position.x = -1.0;
  goal.pose.position.y =  0.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), pi, 1e-3);

  goal.pose.position.x = -1.0;
  goal.pose.position.y = -1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), -3*pi/4, 1e-3);

  goal.pose.position.x =  0.0;
  goal.pose.position.y = -1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), -pi/2, 1e-3);

  goal.pose.position.x =  1.0;
  goal.pose.position.y = -1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), -pi/4, 1e-3);
}

TEST(WaypointControlTests, getAngularErrorRobotOrientation)
{
  tf2::Transform current;
  current.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  Waypoint goal;

  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi/2));
  goal.pose.position.x =  1.0;
  goal.pose.position.y =  0.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), -pi/2, 1e-3);

  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), -pi/2));
  goal.pose.position.x =  1.0;
  goal.pose.position.y =  0.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), pi/2, 1e-3);

  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), 3*pi/4));
  goal.pose.position.x =  1.0;
  goal.pose.position.y =  0.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), -3*pi/4, 1e-3);

  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), -3*pi/4));
  goal.pose.position.x =  1.0;
  goal.pose.position.y =  0.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), 3*pi/4, 1e-3);
}

TEST(WaypointControlTests, getAngularErrorBoth)
{
  tf2::Transform current;
  current.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  Waypoint goal;

  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi/4));
  goal.pose.position.x =  1.0;
  goal.pose.position.y =  1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), 0.0, 1e-3);

  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi/2));
  goal.pose.position.x =  0.0;
  goal.pose.position.y =  1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), 0.0, 1e-3);

  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), -pi/2));
  goal.pose.position.x =   0.0;
  goal.pose.position.y =  -1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), 0.0, 1e-3);

  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi));
  goal.pose.position.x =  -1.0;
  goal.pose.position.y =   0.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), 0.0, 1e-3);


  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi));
  goal.pose.position.x =  -1.0;
  goal.pose.position.y =  -1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), pi/4, 1e-3);

  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi));
  goal.pose.position.x =  -1.0;
  goal.pose.position.y =   1.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), -pi/4, 1e-3);

  current.setOrigin(tf2::Vector3(2.0, 3.0, 0.0));
  current.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi));
  goal.pose.position.x =  1.0;
  goal.pose.position.y =  2.0;
  EXPECT_NEAR(getAngularError(current, goal).smallestAngle(), 3*pi/4, 1e-3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}












