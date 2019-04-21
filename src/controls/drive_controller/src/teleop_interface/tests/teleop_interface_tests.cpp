
// Bring in gtest
#include <gtest/gtest.h>
#include <teleop_interface/teleop_interface.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;

TEST(TeleopTests, maxAndMode)
{
  float max = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  TeleopInterface teleop = TeleopInterface(TeleopInterface::velocity, max, &fl, &fr, &br, &bl);

  EXPECT_EQ(teleop.getMax(), max);
  EXPECT_EQ(teleop.getMode(), TeleopInterface::velocity);

  teleop.setMode(TeleopInterface::duty);
  EXPECT_EQ(teleop.getMode(), TeleopInterface::duty);

  teleop.setMax(2.0f);
  EXPECT_EQ(teleop.getMax(), 0.95f);

  teleop.setMax(-3.0f);
  EXPECT_EQ(teleop.getMax(), 0.95f);

  teleop.setMode(TeleopInterface::velocity);
  teleop.setMax(-5.0f);
  EXPECT_EQ(teleop.getMode(), TeleopInterface::velocity);
  EXPECT_EQ(teleop.getMax(), 5.0f);
}

TEST(TeleopTests, startsWithNoVelocity)
{
  float max_velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  EXPECT_CALL(br, setTorque(0.0f));
  EXPECT_CALL(bl, setTorque(0.0f));
  EXPECT_CALL(fr, setTorque(0.0f));
  EXPECT_CALL(fl, setTorque(0.0f));

  TeleopInterface teleop = TeleopInterface(TeleopInterface::velocity, max_velocity, &fl, &fr, &br, &bl);
}

TEST(TeleopTests, startsWithNoDuty)
{
  float max_duty = 0.8f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  EXPECT_CALL(br, setTorque(0.0f));
  EXPECT_CALL(bl, setTorque(0.0f));
  EXPECT_CALL(fr, setTorque(0.0f));
  EXPECT_CALL(fl, setTorque(0.0f));

  TeleopInterface teleop = TeleopInterface(TeleopInterface::duty, max_duty, &fl, &fr, &br, &bl);
}

TEST(TeleopTests, velocityCanBeUpdated)
{
  float max_velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  EXPECT_CALL(br, setTorque(0.0f)).Times(2);
  EXPECT_CALL(bl, setTorque(0.0f)).Times(2);
  EXPECT_CALL(fr, setTorque(0.0f)).Times(2);
  EXPECT_CALL(fl, setTorque(0.0f)).Times(2);
  EXPECT_CALL(br, setLinearVelocity(10.0f));
  EXPECT_CALL(bl, setLinearVelocity(10.0f));
  EXPECT_CALL(fr, setLinearVelocity(10.0f));
  EXPECT_CALL(fl, setLinearVelocity(10.0f));
  EXPECT_CALL(br, setLinearVelocity(-10.0f));
  EXPECT_CALL(bl, setLinearVelocity(-10.0f));
  EXPECT_CALL(fr, setLinearVelocity(-10.0f));
  EXPECT_CALL(fl, setLinearVelocity(-10.0f));
  EXPECT_CALL(br, setLinearVelocity(10.0f/2.0f));
  EXPECT_CALL(bl, setLinearVelocity(10.0f/2.0f));
  EXPECT_CALL(fr, setLinearVelocity(10.0f/2.0f));
  EXPECT_CALL(fl, setLinearVelocity(10.0f/2.0f));
  EXPECT_CALL(br, setLinearVelocity(-10.0f/2.0f));
  EXPECT_CALL(bl, setLinearVelocity(-10.0f/2.0f));
  EXPECT_CALL(fr, setLinearVelocity(-10.0f/2.0f));
  EXPECT_CALL(fl, setLinearVelocity(-10.0f/2.0f));

  TeleopInterface teleop = TeleopInterface(TeleopInterface::velocity, max_velocity, &fl, &fr, &br, &bl);

  teleop.update( 1.0f,  1.0f);
  teleop.update(-1.0f, -1.0f);
  teleop.update( 0.5f,  0.5f);
  teleop.update(-0.5f, -0.5f);
  teleop.update(-0.0001f, 0.0001f);
}

TEST(TeleopTests, dutyCanBeUpdated)
{
  float max_duty = 0.95f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  EXPECT_CALL(br, setTorque(0.0f)).Times(2);
  EXPECT_CALL(bl, setTorque(0.0f)).Times(2);
  EXPECT_CALL(fr, setTorque(0.0f)).Times(2);
  EXPECT_CALL(fl, setTorque(0.0f)).Times(2);
  EXPECT_CALL(br, setDuty(0.95f));
  EXPECT_CALL(bl, setDuty(0.95f));
  EXPECT_CALL(fr, setDuty(0.95f));
  EXPECT_CALL(fl, setDuty(0.95f));
  EXPECT_CALL(br, setDuty(-0.95f));
  EXPECT_CALL(bl, setDuty(-0.95f));
  EXPECT_CALL(fr, setDuty(-0.95f));
  EXPECT_CALL(fl, setDuty(-0.95f));
  EXPECT_CALL(br, setDuty(0.95f/2.0f));
  EXPECT_CALL(bl, setDuty(0.95f/2.0f));
  EXPECT_CALL(fr, setDuty(0.95f/2.0f));
  EXPECT_CALL(fl, setDuty(0.95f/2.0f));
  EXPECT_CALL(br, setDuty(-0.95f/2.0f));
  EXPECT_CALL(bl, setDuty(-0.95f/2.0f));
  EXPECT_CALL(fr, setDuty(-0.95f/2.0f));
  EXPECT_CALL(fl, setDuty(-0.95f/2.0f));

  TeleopInterface teleop = TeleopInterface(TeleopInterface::duty, max_duty, &fl, &fr, &br, &bl);

  teleop.update( 1.0f,  1.0f);
  teleop.update(-1.0f, -1.0f);
  teleop.update( 0.5f,  0.5f);
  teleop.update(-0.5f, -0.5f);
  teleop.update(-0.0001f, 0.0001f);
}

TEST(TeleopTests, saturatesVelocities)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  EXPECT_CALL (br, setLinearVelocity(10.0f));
  EXPECT_CALL (bl, setLinearVelocity(10.0f));
  EXPECT_CALL (fr, setLinearVelocity(10.0f));
  EXPECT_CALL (fl, setLinearVelocity(10.0f));
  TeleopInterface teleop = TeleopInterface(TeleopInterface::velocity, velocity, &fl, &fr, &br, &bl);

  teleop.update (2.0f, 2.0f);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
