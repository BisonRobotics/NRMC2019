#include <gmock/gmock.h>

#include <occupancy_grid/point.h>
#include <occupancy_grid/line.h>
#include <occupancy_grid/circle.h>

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

  Point b(a);
  ASSERT_DOUBLE_EQ(1.0, b[0]);
  ASSERT_DOUBLE_EQ(2.0, b[1]);

  Point c = b;
  ASSERT_DOUBLE_EQ(1.0, c[0]);
  ASSERT_DOUBLE_EQ(2.0, c[1]);
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

  Line d(c);
  ASSERT_DOUBLE_EQ(1.0, d[0][0]);
  ASSERT_DOUBLE_EQ(2.0, d[0][1]);
  ASSERT_DOUBLE_EQ(3.0, d[1][0]);

  Line e = d;
  ASSERT_DOUBLE_EQ(1.0, e[0][0]);
  ASSERT_DOUBLE_EQ(2.0, e[0][1]);
  ASSERT_DOUBLE_EQ(3.0, e[1][0]);
}

TEST(GeometryTests, Circle)
{
  Point a(1.0, 2.0);
  Circle b(a, 3.0);

  ASSERT_DOUBLE_EQ(1.0, b.p[0]);
  ASSERT_DOUBLE_EQ(2.0, b.p[1]);
  ASSERT_DOUBLE_EQ(3.0, b.r);

  Circle c(4.0, 5.0, 6.0);

  ASSERT_DOUBLE_EQ(4.0, c.p[0]);
  ASSERT_DOUBLE_EQ(5.0, c.p[1]);
  ASSERT_DOUBLE_EQ(6.0, c.r);

  Circle d(c);
  ASSERT_DOUBLE_EQ(4.0, d.p[0]);
  ASSERT_DOUBLE_EQ(5.0, d.p[1]);
  ASSERT_DOUBLE_EQ(6.0, d.r);

  Circle e = d;
  ASSERT_DOUBLE_EQ(4.0, e.p[0]);
  ASSERT_DOUBLE_EQ(5.0, e.p[1]);
  ASSERT_DOUBLE_EQ(6.0, e.r);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}