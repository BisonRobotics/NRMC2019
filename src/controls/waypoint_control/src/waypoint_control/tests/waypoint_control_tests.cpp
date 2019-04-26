#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <waypoint_control/waypoint_controller.h>
#include <boost/math/constants/constants.hpp>

using boost::math::double_constants::pi;
using std::abs;
using std::sqrt;
using std::pow;

using namespace waypoint_control;

TEST(FeedbackTests, invalidOrientation)
{
  tf2::Transform robot;
  Waypoint goal;

  // Throw error if robot quaternion is invalid
  ASSERT_THROW(Feedback(robot, goal), std::runtime_error);

  // Set default orientation if waypoint quaternion is invalid
  robot.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  Feedback feedback(robot, goal);
  EXPECT_NEAR(feedback.theta(), 0.0, 1.0e-6);
}

TEST(FeedbackTests, goalPoses)
{
  tf2::Transform robot;
  robot.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  Waypoint goal;
  goal.pose.orientation.w = 1.0;
  Feedback feedback;

  goal.pose.position.x =  1.0;
  goal.pose.position.y =  0.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),     1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),     0.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     1.0, 1e-6);
  EXPECT_NEAR(feedback.theta(), 0.0, 1e-6);

  goal.pose.position.x =  1.0;
  goal.pose.position.y =  1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),           1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),           1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),      pi/4, 1e-6);

  goal.pose.position.x =  0.0;
  goal.pose.position.y =  1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),           0.0, 1e-6);
  EXPECT_NEAR(feedback.y(),           1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),      pi/2, 1e-6);

  goal.pose.position.x = -1.0;
  goal.pose.position.y =  1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),          -1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),           1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),    3*pi/4, 1e-6);

  goal.pose.position.x = -1.0;
  goal.pose.position.y =  0.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),          -1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),           0.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),        pi, 1e-6);

  goal.pose.position.x = -1.0;
  goal.pose.position.y = -1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),          -1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),          -1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),   -3*pi/4, 1e-6);

  goal.pose.position.x =  0.0;
  goal.pose.position.y = -1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),           0.0, 1e-6);
  EXPECT_NEAR(feedback.y(),          -1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),     -pi/2, 1e-6);

  goal.pose.position.x =  1.0;
  goal.pose.position.y = -1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),           1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),          -1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),     -pi/4, 1e-6);
}

TEST(Feedback, robotPoses)
{
  tf2::Transform robot;
  robot.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  Waypoint goal;
  goal.pose.position.x =  1.0;
  goal.pose.position.y =  0.0;
  goal.pose.orientation.w = 1.0;
  Feedback feedback;

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi/2));
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),         0.0, 1e-6);
  EXPECT_NEAR(feedback.y(),        -1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),         1.0, 1e-6);
  EXPECT_NEAR(feedback.theta(),   -pi/2, 1e-6);

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), -pi/2));
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),         0.0, 1e-6);
  EXPECT_NEAR(feedback.y(),         1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),         1.0, 1e-6);
  EXPECT_NEAR(feedback.theta(),    pi/2, 1e-6);

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), 3*pi/4));
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(), cos(-3*pi/4), 1e-6);
  EXPECT_NEAR(feedback.y(), sin(-3*pi/4), 1e-6);
  EXPECT_NEAR(feedback.r(),          1.0, 1e-6);
  EXPECT_NEAR(feedback.theta(),  -3*pi/4, 1e-6);

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), -3*pi/4));
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),  cos(3*pi/4), 1e-6);
  EXPECT_NEAR(feedback.y(),  sin(3*pi/4), 1e-6);
  EXPECT_NEAR(feedback.r(),         1.0, 1e-6);
  EXPECT_NEAR(feedback.theta(),  3*pi/4, 1e-6);

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi));
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),    -1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),     0.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     1.0, 1e-6);
  EXPECT_NEAR(feedback.theta(), -pi, 1e-6);
}

TEST(Feedback, robotAndGoalPoses)
{
  tf2::Transform robot;
  robot.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  Waypoint goal;
  goal.pose.orientation.w = 1.0;
  Feedback feedback;

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi/4));
  goal.pose.position.x =  1.0;
  goal.pose.position.y =  1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.y(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),       0.0, 1e-6);

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi/2));
  goal.pose.position.x =  0.0;
  goal.pose.position.y =  1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.y(),     sqrt(0.0), 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),       0.0, 1e-6);

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), -pi/2));
  goal.pose.position.x =   0.0;
  goal.pose.position.y =  -1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),           1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),           0.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),       0.0, 1e-6);

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi));
  goal.pose.position.x =  -1.0;
  goal.pose.position.y =   0.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.y(),     sqrt(0.0), 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),       0.0, 1e-6);

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi));
  goal.pose.position.x =  -1.0;
  goal.pose.position.y =  -1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.y(),     sqrt(1.0), 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),      pi/4, 1e-6);

  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi));
  goal.pose.position.x =  -1.0;
  goal.pose.position.y =   1.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),           1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),          -1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),     -pi/4, 1e-6);

  robot.setOrigin(tf2::Vector3(2.0, 3.0, 0.0));
  robot.setRotation(tf2::Quaternion(tf2::Vector3(0, 0, 1), pi));
  goal.pose.position.x =  1.0;
  goal.pose.position.y =  2.0;
  feedback = Feedback(robot, goal);
  EXPECT_NEAR(feedback.x(),         -1.0, 1e-6);
  EXPECT_NEAR(feedback.y(),         -1.0, 1e-6);
  EXPECT_NEAR(feedback.r(),     sqrt(2.0), 1e-6);
  EXPECT_NEAR(feedback.theta(),      pi/4, 1e-6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}












