/**
 * Helpful link on point clouds see slide 48: http://ais.inforcv::Matik.uni-freiburg.de/teaching/ws10/robotics2/pdfs/rob2-12-ros-pcl.pdf
 *
 * */

#include <pc2_processor/pc2_processor.h>
#include <opencv2/imgproc/imgproc.hpp>

// Don't use macros like this, create a function to do it - Jacob
#define CLAMP(A, B, C) ((A < B) ? B : ((A>C) ? C : A))

// There were a bunch of global variables here, don't do that,
// make them members of the class - Jacob


pc2cmProcessor::pc2cmProcessor(double cell_width, double grid_width, double grid_height)
{
    this->cell_width = cell_width;
    this->grid_height = grid_height;
    this->grid_width = grid_width;

    map = cv::Mat((int)(grid_height/cell_width)+1, (int)(grid_width/cell_width), CV_64F );
    point_count = cv::Mat((int)(grid_height/cell_width)+1, (int)(grid_width/cell_width), CV_64F );

    // std::cout << "settings it to a thing" << std::endl;
    map.setTo(0);
    point_count.setTo(0);
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
    // point (0,0) should map to index[0][.5*grid_width]
    // point (1,0) should map to index[1/cell_width][.5*grid_width/cell_width]
    // point (1,1) should map to index(1/cell_width][1/cell_width + .5*grid_width/cell_width

    //std::cout << "adding point" << std::endl;

    int x_index = (int)((grid_height - point.z)/cell_width );
    x_index = CLAMP(x_index, 0, grid_height/cell_width);

    int y_index = (int)((-point.y)/cell_width + (.5*grid_width/cell_width));
    y_index = CLAMP(y_index, 0, grid_width/cell_width);
    //std::cout << "x: " << x_index << " y: " << y_index << std::endl;

    // take the average of all the points in that grid cell
    // You can't do an average in real time, just sum the points,
    // you can take the average once you're done adding points - Jacob
    map.at<double>(x_index, y_index) = map.at<double>(x_index, y_index) + point.x;
    point_count.at<double>(x_index, y_index)++;
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
    std::cout << "pointcount = "<< std::endl << " "  << point_count << std::endl << std::endl;
}


void pc2cmProcessor::computeOccupancyGrid(nav_msgs::OccupancyGrid *costMap){

    if ( DoG.empty() )
    {
        std::cerr << "DOG is not initilized!" << std::endl;
    }

    // The width and height of a matrix should be integer values, not doubles
    // Do something like (int) (grid_height / cell_width) - Jacob
    cv::Mat_<double> out(grid_width, grid_height, CV_64F);
    cv::filter2D(map, out, -1, DoG);
    // std::cout << "out = "<< std::endl << " "  << out << std::endl << std::endl;

    out = out.reshape(1,1);
    imwrite( "output.jpg", out );

    imwrite( "map1.jpg", map );
    //convert cv::Mat to costmap_2d
    // costmap_2d is a ros package, we're using occupancy grid message - Jacob
    costMap->data.insert(costMap->data.begin(), out.begin(), out.end());
}

void pc2cmProcessor::copyMap(cv::Mat *map)
{
  this->map.copyTo(*map);
}
