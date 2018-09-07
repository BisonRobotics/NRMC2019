#include <gmock/gmock.h>
#include <occupancy_grid/occupancy_grid.h>

#include <occupancy_grid/point.h>
#include <occupancy_grid/line.h>

using namespace occupancy_grid;
using namespace testing;

TEST(GeometryTests, Point)
{
  Point a(1.0, 2.0);

  ASSERT_DOUBLE_EQ(1.0, a[0]);
  ASSERT_DOUBLE_EQ(2.0, a[1]);
  ASSERT_DOUBLE_EQ(1.0, a.x);
  ASSERT_DOUBLE_EQ(2.0, a.y);

  ASSERT_THROW(a[-1], std::out_of_range);
  ASSERT_THROW(a[2],  std::out_of_range);
}

TEST(GeometryTests, Line)
{
  Point a(1.0, 2.0);
  Point b(3.0, 4.0);
  Line c(a, b);

  ASSERT_DOUBLE_EQ(1.0, c[0][0]);
  ASSERT_DOUBLE_EQ(2.0, c[0][1]);
  ASSERT_DOUBLE_EQ(3.0, c[1][0]);
  ASSERT_DOUBLE_EQ(4.0, c[1][1]);
  ASSERT_DOUBLE_EQ(1.0, c.p0[0]);
  ASSERT_DOUBLE_EQ(2.0, c.p0[1]);
  ASSERT_DOUBLE_EQ(3.0, c.p1[0]);
  ASSERT_DOUBLE_EQ(4.0, c.p1[1]);
  ASSERT_THROW(c[-1], std::out_of_range);
  ASSERT_THROW(c[2],  std::out_of_range);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}