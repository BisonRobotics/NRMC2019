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

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}