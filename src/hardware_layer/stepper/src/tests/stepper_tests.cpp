#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <stepper/stepper.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;

using namespace stepper;

TEST(TagTests, getCanID)
{
  EXPECT_EQ(Stepper::generateCanID(1, MessageType::Error), 0b1);
}

TEST(TagTests, getMessageType)
{
  EXPECT_EQ(Stepper::getMessageType(0b00001), MessageType::Error);

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}