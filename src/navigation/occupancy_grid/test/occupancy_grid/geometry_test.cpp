#include <gmock/gmock.h>

#include <occupancy_grid/point.h>
#include <occupancy_grid/line.h>
#include <occupancy_grid/circle.h>
#include <occupancy_grid/bezier.h>

using namespace occupancy_grid;
using namespace testing;

TEST(GeometryTests, PointConstructor)
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

TEST(GeometryTests, PointMath)
{
  Point a(1.0, 2.0);
  Point b(3.0, 4.0);
  Point c = a + b;
  ASSERT_DOUBLE_EQ(c.x, 4.0);
  ASSERT_DOUBLE_EQ(c.y, 6.0);

  Point d = 2.0 * a;
  ASSERT_DOUBLE_EQ(d.x, 2.0);
  ASSERT_DOUBLE_EQ(d.y, 4.0);
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

TEST(GeometryTests, Bezier)
{
  Bezier a(0,1,2,3,4,5,6,7);
  ASSERT_DOUBLE_EQ(0.0, a.p0.x);
  ASSERT_DOUBLE_EQ(1.0, a.p0.y);
  ASSERT_DOUBLE_EQ(2.0, a.p1.x);
  ASSERT_DOUBLE_EQ(3.0, a.p1.y);
  ASSERT_DOUBLE_EQ(4.0, a.p2.x);
  ASSERT_DOUBLE_EQ(5.0, a.p2.y);
  ASSERT_DOUBLE_EQ(6.0, a.p3.x);
  ASSERT_DOUBLE_EQ(7.0, a.p3.y);

  Bezier b(a);
  ASSERT_DOUBLE_EQ(0.0, b.p0.x);
  ASSERT_DOUBLE_EQ(1.0, b.p0.y);
  ASSERT_DOUBLE_EQ(2.0, b.p1.x);
  ASSERT_DOUBLE_EQ(3.0, b.p1.y);
  ASSERT_DOUBLE_EQ(4.0, b.p2.x);
  ASSERT_DOUBLE_EQ(5.0, b.p2.y);
  ASSERT_DOUBLE_EQ(6.0, b.p3.x);
  ASSERT_DOUBLE_EQ(7.0, b.p3.y);

  Bezier c(a);
  ASSERT_DOUBLE_EQ(0.0, c.p0.x);
  ASSERT_DOUBLE_EQ(1.0, c.p0.y);
  ASSERT_DOUBLE_EQ(2.0, c.p1.x);
  ASSERT_DOUBLE_EQ(3.0, c.p1.y);
  ASSERT_DOUBLE_EQ(4.0, c.p2.x);
  ASSERT_DOUBLE_EQ(5.0, c.p2.y);
  ASSERT_DOUBLE_EQ(6.0, c.p3.x);
  ASSERT_DOUBLE_EQ(7.0, c.p3.y);

  Bezier d(Point(0,1), Point(1,2), Point(2,3), Point(3,4));
  ASSERT_DOUBLE_EQ(0.0, d.p0.x);
  ASSERT_DOUBLE_EQ(1.0, d.p0.y);
  ASSERT_DOUBLE_EQ(1.0, d.p1.x);
  ASSERT_DOUBLE_EQ(2.0, d.p1.y);
  ASSERT_DOUBLE_EQ(2.0, d.p2.x);
  ASSERT_DOUBLE_EQ(3.0, d.p2.y);
  ASSERT_DOUBLE_EQ(3.0, d.p3.x);
  ASSERT_DOUBLE_EQ(4.0, d.p3.y);

  ASSERT_NO_THROW(a(0,0.5));
  ASSERT_NO_THROW(a(0,0.0));
  ASSERT_NO_THROW(a(0,1.0));

  ASSERT_THROW(a(0,-0.1), std::out_of_range);
  ASSERT_THROW(a(0, 1.1), std::out_of_range);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}