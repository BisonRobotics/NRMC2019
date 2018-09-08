#include <gtest/gtest.h>
#include <pc2_processor/pc2_processor.h>
#include <gmock/gmock.h>


// TEST(pc2_processor_tests, oneAndDone)
// {
//     pc2cmProcessor pcp(1);

//     EXPECT_TRUE(pcp.getOne());
// }

TEST(pc2_processor_points, oneAndDone)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Fill in the cloud data
    cloud.width  = 5;
    cloud.height = 1;
    cloud.points.resize (cloud.width * cloud.height);
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud.points.size (); ++i)
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    pc2cmProcessor pcp(.25, 3.0, 3.0);

    pcp.addPoints(cloud);
    EXPECT_TRUE(pcp.getOne());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}