#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>
#include <ros/ros.h>

#include <dig_control/dig_controller.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;
using ::testing::InSequence;

using namespace dig_control;

// TODO only works with a running ROS master and a connected or virtual can network
// Should change to an integration test
// sudo ip link add vesc_can type vcan
// sudo ip link set vesc_can up
/*TEST(DigControllerTests, allocatesCorrectly)
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
}*/

TEST(DigControllerTests, initializesCorrectly)
{
  using nsVescAccess::limitSwitchState;

  MockVescAccess central_drive;
  MockVescAccess backhoe_actuator;
  MockVescAccess bucket_actuator;
  MockVescAccess vibrator;

  // Initial stop and call to update
  EXPECT_CALL(central_drive, setCustom(0)).Times(2);
  EXPECT_CALL(backhoe_actuator, setCustom(0)).Times(2);
  EXPECT_CALL(bucket_actuator, setDuty(0)).Times(2);
  EXPECT_CALL(vibrator, setDuty(0)).Times(2);

  // Respond to initial state checks
  EXPECT_CALL(central_drive, getLimitSwitchState()).WillOnce(Return(limitSwitchState::bottomOfMotion));
  EXPECT_CALL(central_drive, getADC()).WillOnce(Return(0));
  EXPECT_CALL(backhoe_actuator, getLimitSwitchState()).WillOnce(Return(limitSwitchState::bottomOfMotion));
  EXPECT_CALL(backhoe_actuator, getLinearVelocity()).WillOnce(Return(0.0f));
  EXPECT_CALL(bucket_actuator, getTorque()).WillOnce(Return(0.0f));
  DigController controller(&central_drive, &backhoe_actuator, &bucket_actuator, &vibrator);

  // Make sure everything initialized correctly
  EXPECT_EQ(controller.isInternallyAllocated(), false);
  EXPECT_EQ(controller.getBucketDuty(), 0.0f);
  EXPECT_EQ(controller.getVibratorDuty(), 0.0f);
  EXPECT_EQ(controller.getBackhoeDuty(), 0.0f);
  EXPECT_EQ(controller.getCentralDriveDuty(), 0.0f);
  EXPECT_EQ(controller.getBackhoeState(), BackhoeState::open);
  EXPECT_EQ(controller.getCentralDriveState(), CentralDriveState::at_bottom_limit);
  EXPECT_EQ(controller.getBucketState(), BucketState::down);
  EXPECT_EQ(controller.getControlState(), ControlState::ready);
}

TEST(DigControllerTests, setControlState)
{
  using nsVescAccess::limitSwitchState;

  MockVescAccess central_drive;
  MockVescAccess backhoe_actuator;
  MockVescAccess bucket_actuator;
  MockVescAccess vibrator;

  // Initial Stop
  EXPECT_CALL(central_drive, setCustom(0)).Times(2);
  EXPECT_CALL(backhoe_actuator, setCustom(0)).Times(2);
  EXPECT_CALL(bucket_actuator, setDuty(0)).Times(2);
  EXPECT_CALL(vibrator, setDuty(0)).Times(2);

  // Respond to initial state checks
  EXPECT_CALL(central_drive, getLimitSwitchState()).WillOnce(Return(limitSwitchState::bottomOfMotion));
  EXPECT_CALL(central_drive, getADC()).WillOnce(Return(0));
  EXPECT_CALL(backhoe_actuator, getLimitSwitchState()).WillOnce(Return(limitSwitchState::bottomOfMotion));
  EXPECT_CALL(backhoe_actuator, getLinearVelocity()).WillOnce(Return(0.0f));
  EXPECT_CALL(bucket_actuator, getTorque()).WillOnce(Return(0.0f));
  DigController controller(&central_drive, &backhoe_actuator, &bucket_actuator, &vibrator);

  // Test setting control states
  EXPECT_EQ(controller.getControlState(), ControlState::ready);
  controller.setControlState(ControlState::manual);
  EXPECT_CALL(central_drive, setCustom(0));
  EXPECT_CALL(backhoe_actuator, setCustom(0));
  EXPECT_CALL(bucket_actuator, setDuty(0));
  EXPECT_CALL(vibrator, setDuty(0));
  EXPECT_EQ(controller.getControlState(), ControlState::manual);
  controller.setControlState(ControlState::ready);
  EXPECT_EQ(controller.getControlState(), ControlState::ready);
}

/*TEST(DigControllerTests, manualControls)
{
  using nsVescAccess::limitSwitchState;

  MockVescAccess central_drive;
  MockVescAccess backhoe_actuator;
  MockVescAccess bucket_actuator;
  MockVescAccess vibrator;

  // Initial Stop
  EXPECT_CALL(central_drive, setDuty(0)).Times(2);
  EXPECT_CALL(backhoe_actuator, setDuty(0)).Times(2);
  EXPECT_CALL(bucket_actuator, setDuty(0)).Times(2);
  EXPECT_CALL(vibrator, setDuty(0)).Times(2);

  // Respond to initial state checks
  EXPECT_CALL(central_drive, getLimitSwitchState()).WillOnce(Return(limitSwitchState::bottomOfMotion));
  EXPECT_CALL(central_drive, getADC()).WillOnce(Return(0.0f));
  EXPECT_CALL(backhoe_actuator, getLimitSwitchState()).WillOnce(Return(limitSwitchState::bottomOfMotion));
  EXPECT_CALL(backhoe_actuator, getLinearVelocity()).WillOnce(Return(0.0f));
  EXPECT_CALL(bucket_actuator, getTorque()).WillOnce(Return(0.0f));
  DigController controller(&central_drive, &backhoe_actuator, &bucket_actuator, &vibrator);

  // Respond to some updates
  EXPECT_CALL(central_drive, getLimitSwitchState()).Times(5).WillRepeatedly(Return(limitSwitchState::bottomOfMotion));
  EXPECT_CALL(central_drive, getADC()).Times(5).WillRepeatedly(Return(0.0f));
  EXPECT_CALL(backhoe_actuator, getLimitSwitchState()).Times(5).WillRepeatedly(Return(limitSwitchState::bottomOfMotion));
  EXPECT_CALL(backhoe_actuator, getLinearVelocity()).Times(5).WillRepeatedly(Return(0.0f));
  EXPECT_CALL(bucket_actuator, getTorque()).Times(5).WillRepeatedly(Return(0.0f));
  EXPECT_CALL(central_drive, setDuty(0)).Times(5);
  EXPECT_CALL(backhoe_actuator, setDuty(0)).Times(5);
  EXPECT_CALL(bucket_actuator, setDuty(0)).Times(5);
  EXPECT_CALL(vibrator, setDuty(0)).Times(5);
  for (int i = 0; i < 5; i++) controller.update();

  // Set control state to manual
  EXPECT_EQ(controller.getControlState(), ControlState::ready);
  controller.setControlState(ControlState::manual);
}*/


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}