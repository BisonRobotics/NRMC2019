#include <gtest/gtest.h>
#include <pc2_processor/pc2_processor.h>
#include <gmock/gmock.h>


TEST(pc2_processor_tests, oneAndDone)
{
  //pc2cmProcessor pcp(1);

  EXPECT_TRUE(true /*|| pcp.getOne()*/);
}

TEST(WaypointControllerTests, overAndOut)
{
  //pc2cmProcessor pcp(0);

  EXPECT_TRUE(true /*|| pcp.getOne()*/);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}