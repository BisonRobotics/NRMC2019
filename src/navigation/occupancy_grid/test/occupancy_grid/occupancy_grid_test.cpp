#include <gmock/gmock.h>
#include <occupancy_grid/occupancy_grid.h>

using namespace occupancy_grid;
using namespace testing;

TEST(OccupancyGridTests, Constructor)
{
  OccupancyGrid a = OccupancyGrid(320, 480);

  ASSERT_EQ(a.rows, 320);
  ASSERT_EQ(a.cols, 480);
  ASSERT_THAT(a, Each(0));

  OccupancyGrid b = OccupancyGrid(320, 480, 100);

  ASSERT_EQ(b.rows, 320);
  ASSERT_EQ(b.cols, 480);
  ASSERT_THAT(b, Each(100));
}

TEST(OccupancyGridTests, Merge)
{
  OccupancyGrid a = OccupancyGrid(320, 480);
  OccupancyGrid b = OccupancyGrid(320, 480, 100);
  OccupancyGrid c = OccupancyGrid(320, 480, 20);
  OccupancyGrid::max(a, b, c);

  ASSERT_EQ(c.rows, 320);
  ASSERT_EQ(c.cols, 480);
  ASSERT_THAT(c, Each(100));

  OccupancyGrid e = OccupancyGrid(320, 480, 50);
  OccupancyGrid f = OccupancyGrid(320, 480);
  OccupancyGrid g = OccupancyGrid(320, 480, 10);
  OccupancyGrid::max(e, f, g);
  ASSERT_EQ(g.rows, 320);
  ASSERT_EQ(g.cols, 480);
  ASSERT_THAT(g, Each(50));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}