#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>
#include <ros/ros.h>

#include <dig_control_2/dig_controller/dig_controller.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;

using namespace dig_control_2;

TEST(DigControllerTests, allocatesCorrectly)
{
  int argc = 0;
  char **argv = nullptr;
  ros::init(argc, argv, "dig_controller_tests");

  NiceMock<MockVescAccess> central_drive;
  NiceMock<MockVescAccess> backhoe_actuator;
  NiceMock<MockVescAccess> bucket_actuator;
  NiceMock<MockVescAccess> vibrator;
  DigController controller_mock(&central_drive, &backhoe_actuator, &bucket_actuator, &vibrator);
  DigController controller;

  EXPECT_EQ(controller_mock.isInternallyAllocated(), false);
  EXPECT_EQ(controller.isInternallyAllocated(), true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}