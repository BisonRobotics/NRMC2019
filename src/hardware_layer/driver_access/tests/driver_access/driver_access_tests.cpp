#include <gmock/gmock.h>

#include <driver_access/driver_access_mock.h>

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
  ASSERT_DOUBLE_EQ(1.0, object1.min_velocity);
  ASSERT_DOUBLE_EQ(2.0, object1.max_velocity);
  ASSERT_DOUBLE_EQ(3.0, object1.min_torque);
  ASSERT_DOUBLE_EQ(4.0, object1.max_torque);
  ASSERT_DOUBLE_EQ(5.0, object1.min_position);
  ASSERT_DOUBLE_EQ(6.0, object1.max_position);

  ASSERT_NO_THROW(new Limits(object1));

  Limits object2(object1);
  ASSERT_DOUBLE_EQ(1.0, object2.min_velocity);
  ASSERT_DOUBLE_EQ(2.0, object2.max_velocity);
  ASSERT_DOUBLE_EQ(3.0, object2.min_torque);
  ASSERT_DOUBLE_EQ(4.0, object2.max_torque);
  ASSERT_DOUBLE_EQ(5.0, object2.min_position);
  ASSERT_DOUBLE_EQ(6.0, object2.max_position);
}

TEST(DriverAccessTests, Velocity)
{
  InSequence in_sequence;

  Limits limits(1, 2, 3, 4, 5, 6);
  NiceMock<DriverAccessMock> controller(limits);

  EXPECT_CALL(controller, setDriverVelocity( 1.5));
  EXPECT_CALL(controller, setDriverVelocity(-1.5));
  EXPECT_CALL(controller, setDriverVelocity( 1.0));
  EXPECT_CALL(controller, setDriverVelocity( 2.0));
  EXPECT_CALL(controller, setDriverVelocity(-1.0));
  EXPECT_CALL(controller, setDriverVelocity(-2.0));

  controller.setVelocity( 1.5);
  controller.setVelocity(-1.5);
  controller.setVelocity( 0.5);
  controller.setVelocity( 9.0);
  controller.setVelocity(-0.5);
  controller.setVelocity(-9.0);
}

TEST(DriverAccessTests, Torque)
{
  InSequence in_sequence;

  Limits limits(1, 2, 3, 4, 5, 6);
  NiceMock<DriverAccessMock> controller(limits);

  EXPECT_CALL(controller, setDriverTorque( 3.5));
  EXPECT_CALL(controller, setDriverTorque(-3.5));
  EXPECT_CALL(controller, setDriverTorque( 3.0));
  EXPECT_CALL(controller, setDriverTorque( 4.0));
  EXPECT_CALL(controller, setDriverTorque(-3.0));
  EXPECT_CALL(controller, setDriverTorque(-4.0));

  controller.setTorque( 3.5);
  controller.setTorque(-3.5);
  controller.setTorque( 0.5);
  controller.setTorque( 9.0);
  controller.setTorque(-0.5);
  controller.setTorque(-9.0);
}

TEST(DriverAccessTests, Position)
{
  InSequence in_sequence;

  Limits limits(1, 2, 3, 4, 5, 6);
  NiceMock<DriverAccessMock> controller(limits);

  EXPECT_CALL(controller, setDriverPosition( 5.5));
  EXPECT_CALL(controller, setDriverPosition( 5.0)).Times(3);
  EXPECT_CALL(controller, setDriverPosition( 6.0));

  controller.setPosition( 5.5);
  controller.setPosition( 0.5);
  controller.setPosition(-0.5);
  controller.setPosition(-9.0);
  controller.setPosition( 9.0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}