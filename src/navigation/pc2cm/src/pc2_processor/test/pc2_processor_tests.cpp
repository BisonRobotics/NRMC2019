#include <gtest/gtest.h>
#include <pc2_processor/pc2_processor.h>
#include <gmock/gmock.h>
#include "opencv2/core/mat.hpp"
#include "opencv2/opencv.hpp"

TEST(pc2_processor_points, oneAndDone)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Fill in the cloud data
    cloud.width  =8;
    cloud.height = 8;
    cloud.points.resize(cloud.width * cloud.height);
    // for (size_t i = 0; i < cloud.points.size (); ++i)
    // {
    //     cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
    //     cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
    //     cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
    // }

    // cloud.points[0].x = 5;
    // cloud.points[0].y = 1;
    // cloud.points[0].z = 3;

    // cloud.points[1].x = 3;
    // cloud.points[1].y = 1;
    // cloud.points[1].z = 8;

    // cloud.points[2].x = 4;
    // cloud.points[2].y = 3;
    // cloud.points[2].z = 4;

    // cloud.points[3].x = 7;
    // cloud.points[3].y = 4;
    // cloud.points[3].z = 3;

    // std::cerr << "Cloud before filtering: " << std::endl;
    // for (size_t i = 0; i < cloud.points.size (); ++i)
    //     std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    pc2cmProcessor pcp(.25, 4, 4);

    pcp.addPoints(cloud);

    cv::Mat dog = pcp.takeDoG(9, .2, .8);

    std::cout << "DoG= = "<< std::endl << " "  << dog << std::endl << std::endl;
    pcp.print_grid();
    float avg = 1 ; //(cloud.points[3].x + cloud.points[0].x)/2;
    EXPECT_TRUE((abs(pcp.get_Height(0,0) - avg) < .001));
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}