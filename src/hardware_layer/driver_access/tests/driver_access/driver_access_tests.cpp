#include <gmock/gmock.h>

#include <driver_access/driver_access_mock.h>
#include <driver_access/mode.h>
#include <driver_access/params.h>

using ::testing::NiceMock;
using ::testing::InSequence;

using namespace driver_access;

TEST(DriverAccessTests, Limits)
{
  ASSERT_NO_THROW(Limits(0, 1, 0, 1, 0, 1));

  ASSERT_THROW(Limits(1, 0, 0, 1, 0, 1), limits_error);
  ASSERT_THROW(Limits(0, 1, 1, 0, 0, 1), limits_error);
  ASSERT_THROW(Limits(0, 1, 0, 1, 1, 0), limits_error);

  ASSERT_THROW(Limits(-1,  1,  0,  1,  1,  0), limits_error);
  ASSERT_THROW(Limits(-2, -1,  0,  1,  1,  0), limits_error);
  ASSERT_THROW(Limits( 0,  1, -1,  1,  0,  1), limits_error);
  ASSERT_THROW(Limits( 0,  1, -2, -1,  0,  1), limits_error);

  Limits object1(1, 2, 3, 4, 5, 6);
  ASSERT_DOUBLE_EQ(1.0, object1.min_position);
  ASSERT_DOUBLE_EQ(2.0, object1.max_position);
  ASSERT_DOUBLE_EQ(3.0, object1.min_velocity);
  ASSERT_DOUBLE_EQ(4.0, object1.max_velocity);
  ASSERT_DOUBLE_EQ(5.0, object1.min_effort);
  ASSERT_DOUBLE_EQ(6.0, object1.max_effort);

  ASSERT_NO_THROW(new Limits(object1));

  Limits object2(object1);
  ASSERT_DOUBLE_EQ(1.0, object2.min_position);
  ASSERT_DOUBLE_EQ(2.0, object2.max_position);
  ASSERT_DOUBLE_EQ(3.0, object2.min_velocity);
  ASSERT_DOUBLE_EQ(4.0, object2.max_velocity);
  ASSERT_DOUBLE_EQ(5.0, object2.min_effort);
  ASSERT_DOUBLE_EQ(6.0, object2.max_effort);
}

TEST(DriverAccessTests, Position)
{
  InSequence in_sequence;

  Limits limits(1, 2, 3, 4, 5, 6);
  NiceMock<DriverAccessMock> controller(limits);

  EXPECT_CALL(controller, setDriverPosition( 1.5));
  EXPECT_CALL(controller, setDriverPosition( 1.0)).Times(3);
  EXPECT_CALL(controller, setDriverPosition( 2.0));

  controller.setPosition( 1.5);
  controller.setPosition( 0.5);
  controller.setPosition(-0.5);
  controller.setPosition(-9.0);
  controller.setPosition( 9.0);
}

TEST(DriverAccessTests, Velocity)
{
  InSequence in_sequence;

  Limits limits(1, 2, 3, 4, 5, 6);
  NiceMock<DriverAccessMock> controller(limits);

  EXPECT_CALL(controller, setDriverVelocity( 3.5));
  EXPECT_CALL(controller, setDriverVelocity(-3.5));
  EXPECT_CALL(controller, setDriverVelocity( 3.0));
  EXPECT_CALL(controller, setDriverVelocity( 4.0));
  EXPECT_CALL(controller, setDriverVelocity(-3.0));
  EXPECT_CALL(controller, setDriverVelocity(-4.0));

  controller.setVelocity( 3.5);
  controller.setVelocity(-3.5);
  controller.setVelocity( 0.5);
  controller.setVelocity( 9.0);
  controller.setVelocity(-0.5);
  controller.setVelocity(-9.0);
}

TEST(DriverAccessTests, Effort)
{
  InSequence in_sequence;

  Limits limits(1, 2, 3, 4, 5, 6);
  NiceMock<DriverAccessMock> controller(limits);

  EXPECT_CALL(controller, setDriverEffort( 5.5));
  EXPECT_CALL(controller, setDriverEffort(-5.5));
  EXPECT_CALL(controller, setDriverEffort( 5.0));
  EXPECT_CALL(controller, setDriverEffort( 6.0));
  EXPECT_CALL(controller, setDriverEffort(-5.0));
  EXPECT_CALL(controller, setDriverEffort(-6.0));

  controller.setEffort( 5.5);
  controller.setEffort(-5.5);
  controller.setEffort( 0.5);
  controller.setEffort( 9.0);
  controller.setEffort(-0.5);
  controller.setEffort(-9.0);
}


TEST(DriverAccessTests, SetPoint)
{
  InSequence in_sequence;

  Limits limits(1, 2, 3, 4, 5, 6);
  NiceMock<DriverAccessMock> controller(limits);
  NiceMock<DriverAccessMock> position_controller(limits, Mode::position);
  NiceMock<DriverAccessMock> velocity_controller(limits, Mode::velocity);
  NiceMock<DriverAccessMock> effort_controller(limits, Mode::effort);

  ASSERT_THROW(controller.setPoint(0.5), mode_error);
  ASSERT_NO_THROW(controller.setPosition(0.5));
  ASSERT_NO_THROW(controller.setVelocity(0.5));
  ASSERT_NO_THROW(controller.setEffort(0.5));

  ASSERT_THROW(position_controller.setVelocity(0.5), mode_error);
  ASSERT_THROW(position_controller.setEffort(0.5), mode_error);
  ASSERT_NO_THROW(position_controller.setPosition(0.5));

  ASSERT_THROW(velocity_controller.setPosition(0.5), mode_error);
  ASSERT_THROW(velocity_controller.setEffort(0.5), mode_error);
  ASSERT_NO_THROW(velocity_controller.setVelocity(0.5));

  ASSERT_THROW(effort_controller.setPosition(0.5), mode_error);
  ASSERT_THROW(effort_controller.setVelocity(0.5), mode_error);
  ASSERT_NO_THROW(effort_controller.setEffort(0.5));

  EXPECT_CALL(position_controller, setDriverPosition(1.0));
  EXPECT_CALL(velocity_controller, setDriverVelocity(3.0));
  EXPECT_CALL(effort_controller, setDriverEffort(5.0));
  EXPECT_CALL(controller, setDriverPosition(1.0));
  EXPECT_CALL(controller, setDriverVelocity(3.0));
  EXPECT_CALL(controller, setDriverEffort(5.0));

  position_controller.setPoint(0.5);
  velocity_controller.setPoint(0.5);
  effort_controller.setPoint(0.5);
  controller.setMode(Mode::position);
  controller.setPoint(0.5);
  controller.setMode(Mode::velocity);
  controller.setPoint(0.5);
  controller.setMode(Mode::effort);
  controller.setPoint(0.5);

  ASSERT_EQ(Mode::position, static_cast<Mode>(1));
  ASSERT_EQ(Mode::velocity, static_cast<Mode>(2));
  ASSERT_EQ(Mode::effort, static_cast<Mode>(3));

}

TEST(DriverAccessTests, Params)
{
  ASSERT_EQ(1, static_cast<uint8_t>(ID::front_left_wheel));
  ASSERT_EQ(2, static_cast<uint8_t>(ID::front_right_wheel));
  ASSERT_EQ(3, static_cast<uint8_t>(ID::back_right_wheel));
  ASSERT_EQ(4, static_cast<uint8_t>(ID::back_left_wheel));
  ASSERT_EQ(5, static_cast<uint8_t>(ID::central_drive));
  ASSERT_EQ(6, static_cast<uint8_t>(ID::linear_motor));

  ASSERT_EQ("front_left_wheel",  name(ID::front_left_wheel));
  ASSERT_EQ("front_right_wheel", name(ID::front_right_wheel));
  ASSERT_EQ("back_right_wheel",  name(ID::back_right_wheel));
  ASSERT_EQ("back_left_wheel",   name(ID::back_left_wheel));
  ASSERT_EQ("central_drive",     name(ID::central_drive));
  ASSERT_EQ("linear_actuator",   name(ID::linear_motor));

  ASSERT_EQ("front_left_wheel",  name(1));
  ASSERT_EQ("front_right_wheel", name(2));
  ASSERT_EQ("back_right_wheel",  name(3));
  ASSERT_EQ("back_left_wheel",   name(4));
  ASSERT_EQ("central_drive",     name(5));
  ASSERT_EQ("linear_actuator",   name(6));

  ASSERT_EQ("",  name(0));
  ASSERT_EQ("",  name(7));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}