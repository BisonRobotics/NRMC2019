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
    cloud.width  = 3;
    cloud.height = 3;
    cloud.points.resize(cloud.width * cloud.height);
    // for (size_t i = 0; i < cloud.points.size (); ++i)
    // {
    //     cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
    //     cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
    //     cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
    // }

    cloud.points[0].x = 5;
    cloud.points[0].y = 1;
    cloud.points[0].z = 1;

    cloud.points[1].x = 3;
    cloud.points[1].y = 1;
    cloud.points[1].z = 0;

    cloud.points[2].x = 4;
    cloud.points[2].y = 0;
    cloud.points[2].z = 1;


    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud.points.size (); ++i)
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    pc2cmProcessor pcp(.25, 3.0, 3.0);

    pcp.addPoints(cloud);

    std::cerr << pcp.get_Height(1, 1) << std::endl;

    EXPECT_TRUE(pcp.getOne());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}