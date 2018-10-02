#include <gtest/gtest.h>
#include <drive_controller/drive_controller.h>
#include <vesc_access/mock_vesc_access.h>

#include <gmock/gmock.h>
#include <vector>
#include <utility>

#define _USE_MATH_DEFINES
#include <cmath>

using ::testing::NiceMock;

TEST(DriveControllerTests, angleDiffWorks_1)
{

  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  DriveController dc(&fr, &fl, &bl, &br);

  //EXPECT_TRUE(false);
  EXPECT_NEAR(dc.angleDiff(1,2), -1, .0001);
  EXPECT_NEAR(dc.angleDiff(1,-2), 3, .0001);
}

TEST(DriveControllerTests, sscworks_1)
{

  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  DriveController dc(&fr, &fl, &bl, &br);

  std::pair<double, double> UlUr;
  double speed=.5; double steering = 1;double axelLen = .5;double maxSpeed = 1.3;
  UlUr = dc.speedSteeringControl(speed, steering, axelLen, maxSpeed);

  EXPECT_NEAR(UlUr.first, .375, .0001);
  EXPECT_NEAR(UlUr.second, .625, .0001);
}

TEST(DriveControllerTests, gaaliWorks_1)
{

  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  DriveController dc(&fr, &fl, &bl, &br);

  DriveController_ns::bezier_path path = {.x1 = 1, .y1 =1, .x2 = 2, .y2 = 1,
                                          .x3 = 2.5, .y3 = .3, .x4 = 2, .y4 = -.3};

  int chopsize = 100;
  std::vector<double>  theta(chopsize);
  std::vector<double>  omega(chopsize);
  std::vector<double>  alpha(chopsize);
  std::vector<double>  lengths(chopsize);
  std::vector<double>  x(chopsize);
  std::vector<double>  y(chopsize);
  double length;
  
  dc.getAngleAndLengthInfo(path, 
                           theta, omega, 
                           alpha, lengths, 
                           x,  y, length,
                           chopsize);

  EXPECT_NEAR(length, 2.0913, .001);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}