/**
 * Helpful link on point clouds see slide 48: http://ais.inforcv::Matik.uni-freiburg.de/teaching/ws10/robotics2/pdfs/rob2-12-ros-pcl.pdf
 *
 * */

#include <pc2_processor/pc2_processor.h>

#include <opencv2/imgproc/imgproc.hpp>

#define CLAMP(A, B, C) ((A < B) ? B : ((A>C) ? C : A))


cv::Mat map; // a grid of heights defined by Cell_width, Grid_Width, GRID_HEIGHT
cv::Mat pointCount;
cv::Mat DoG; // the current DoG cv::Matrix to convulote throught the map


double GRID_WIDTH;
double GRID_HEIGHT;
double CELL_WIDTH;

pc2cmProcessor::pc2cmProcessor()
{
    ;
}


pc2cmProcessor::pc2cmProcessor(double cell_width, double grid_width, double grid_height)
{
    CELL_WIDTH = cell_width;
    GRID_HEIGHT = grid_height;
    GRID_WIDTH = grid_width;


    map = cv::Mat((int)(GRID_HEIGHT/CELL_WIDTH)+1, (int)(GRID_WIDTH/CELL_WIDTH), CV_64F );
    pointCount = cv::Mat((int)(GRID_HEIGHT/CELL_WIDTH)+1, (int)(GRID_WIDTH/CELL_WIDTH), CV_64F );

    // std::cout << "settings it to a thing" << std::endl;
    map.setTo(0);
    pointCount.setTo(0);
    takeDoG(9, .8, .2);

}

bool pc2cmProcessor::addPoints(pcl::PointCloud<pcl::PointXYZ> cloud_msg){

    for (size_t i = 0; i < cloud_msg.points.size(); ++i)
    {
        pcl::PointXYZ pt = cloud_msg.points[i];
        if (!(pt.x ==0 && pt.y ==0 && pt.z ==0))
        {
            if(! addPoint(pt)){
                std::cerr << "Error adding point" << std::endl;
                return false;
            }
        }
    }
    return true;
}

bool pc2cmProcessor::addPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg){

    for (const pcl::PointXYZ& pt : cloud_msg->points)
    {
        if (!(pt.x ==0 && pt.y ==0 && pt.z ==0))
        {
            if(! addPoint(pt)){
                std::cerr << "Error adding point" << std::endl;
                return false;
            }
        }
    }
    return true;
}

bool pc2cmProcessor::addPoint(pcl::PointXYZ point){
    //point (0,0) should map to index[0][.5*GRID_WIDTH]
    //point (1,0) should map to index[1/CELL_WIDTH][.5*GRID_WIDTH/CELL_WIDTH]
    // point (1,1) should map to index(1/CELL_WIDTH][1/CELL_WIDTH + .5*GRID_WIDTH/CELL_WIDTH

    std::cout << "adding point" << std::endl;

    int x_index = (int)((GRID_HEIGHT - point.z)/CELL_WIDTH );
    x_index = CLAMP(x_index, 0, GRID_HEIGHT/CELL_WIDTH);

    int y_index = (int)((-point.y)/CELL_WIDTH + (.5*GRID_WIDTH/CELL_WIDTH));
    y_index = CLAMP(y_index, 0, GRID_WIDTH/CELL_WIDTH);

    // take the avaerage of all the points in that grid cell
    map.at<double>(x_index, y_index) = map.at<double>(x_index, y_index) +
                                        (point.x - map.at<double>(x_index, y_index))
                                        / (++pointCount.at<int>(x_index, y_index));
    return true;
}

// https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#getgaussiankernel
cv::Mat pc2cmProcessor::takeDoG(int kernel_size, double sigma1, double sigma2){
    cv::Mat first;
    // multiply the first Guassian kernal by the transpose of it's self and store in first
    cv::mulTransposed(cv::getGaussianKernel(kernel_size, sigma1, CV_64F), first, false);
    cv::Mat second;
    cv::mulTransposed(cv::getGaussianKernel(kernel_size, sigma2, CV_64F), second,  false);
    DoG = cv::Mat(first.size(), first.type());

    subtract(first, second, DoG);

    return DoG ;
}

double pc2cmProcessor::get_Height(int xindex, int yindex){
    return map.at<double>(xindex, yindex);
}

void pc2cmProcessor::print_grid(void){
    std::cout << "map = "<< std::endl << " "  << map << std::endl << std::endl;
    std::cout << "pointcount = "<< std::endl << " "  << pointCount << std::endl << std::endl;
}


void pc2cmProcessor::computeOccupancyGrid(nav_msgs::OccupancyGrid *costMap){

    if ( DoG.empty() )
    {
        std::cerr << "DOG is not initilized!" << std::endl;
    }
    cv::Mat_<double> out(GRID_WIDTH, GRID_HEIGHT, CV_64F);
    cv::filter2D(map, out, -1, DoG);
    // std::cout << "out = "<< std::endl << " "  << out << std::endl << std::endl;

    out = out.reshape(1,1);
    imwrite( "output.jpg", out );

    imwrite( "map1.jpg", map );
    //convert cv::Mat to costmap_2d
    costMap->data.insert(costMap->data.begin(), out.begin(), out.end());
}
