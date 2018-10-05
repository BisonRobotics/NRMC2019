/**
 * Helpful link on point clouds see slide 48: http://ais.informatik.uni-freiburg.de/teaching/ws10/robotics2/pdfs/rob2-12-ros-pcl.pdf
 *
 * */

#include <pc2_processor/pc2_processor.h>


#define CLAMP(A, B, C) ((A < B) ? B : ((A>C) ? C : A))


Mat map; // a grid of heights defined by Cell_width, Grid_Width, GRID_HEIGHT
Mat pointCount;
Mat DoG; // the current DoG matrix to convulote throught the map


double GRID_WIDTH;
double GRID_HEIGHT;
double CELL_WIDTH;

pc2cmProcessor::pc2cmProcessor()
{
    ;
}


pc2cmProcessor::pc2cmProcessor(double cell_width, double grid_width, double GRID_HEIGHT)
{
    CELL_WIDTH = cell_width;
    GRID_HEIGHT = GRID_HEIGHT;
    GRID_WIDTH = grid_width;

    map = Mat((int)(GRID_HEIGHT/CELL_WIDTH)+1, (int)(GRID_WIDTH/CELL_WIDTH),CV_64F );
    pointCount = Mat((int)(GRID_HEIGHT/CELL_WIDTH)+1, (int)(GRID_WIDTH/CELL_WIDTH),CV_16U );

    map.setTo(0);
    pointCount.setTo(0);
    takeDoG(9, .8, .2);

    // std::cerr << "created pc2mProccessor" << std::endl;

}

bool pc2cmProcessor::addPoints(pcl::PointCloud<pcl::PointXYZ> cloud_msg){

    std::cerr << " adding points. Size of cloud = "<< cloud_msg.points.size() << std::endl;

    // for (const pcl::PointXYZ& pt : cloud_msg->points)
    for (size_t i = 0; i < cloud_msg.points.size(); ++i)
    {
        pcl::PointXYZ pt = cloud_msg.points[i];
        if (!(pt.x ==0 && pt.y ==0 && pt.z ==0))
        {
            //point (0,0) should map to index[0][.5*GRID_WIDTH]
            //point (1,0) should map to index[1/CELL_WIDTH][.5*GRID_WIDTH/CELL_WIDTH]
            // point (1,1) should map to index(1/CELL_WIDTH][1/CELL_WIDTH + .5*GRID_WIDTH/CELL_WIDTH
            int x_index = (int)((GRID_HEIGHT - pt.z)/CELL_WIDTH );
            x_index = CLAMP(x_index, 0, GRID_HEIGHT/CELL_WIDTH);

            int y_index = (int)((-pt.y)/CELL_WIDTH + (.5*GRID_WIDTH/CELL_WIDTH));
            y_index = CLAMP(y_index, 0, GRID_WIDTH/CELL_WIDTH);

            map.at<double>(x_index, y_index) = map.at<double>(x_index, y_index) +
                                                (pt.x - map.at<double>(x_index, y_index))
                                                / (++pointCount.at<int>(x_index, y_index));
        }
    }
    return true;
}

bool pc2cmProcessor::addPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg){

    for (const pcl::PointXYZ& pt : cloud_msg->points)
    {
        if (!(pt.x ==0 && pt.y ==0 && pt.z ==0))
        {
            //point (0,0) should map to index[0][.5*GRID_WIDTH]
            //point (1,0) should map to index[1/CELL_WIDTH][.5*GRID_WIDTH/CELL_WIDTH]
            // point (1,1) should map to index(1/CELL_WIDTH][1/CELL_WIDTH + .5*GRID_WIDTH/CELL_WIDTH
            int x_index = (int)((GRID_HEIGHT - pt.z)/CELL_WIDTH );
            x_index = CLAMP(x_index, 0, GRID_HEIGHT/CELL_WIDTH);

            int y_index = (int)((-pt.y)/CELL_WIDTH + (.5*GRID_WIDTH/CELL_WIDTH));
            y_index = CLAMP(y_index, 0, GRID_WIDTH/CELL_WIDTH);

            map.at<double>(x_index, y_index) = map.at<double>(x_index, y_index) +
                                                (pt.x - map.at<double>(x_index, y_index))
                                                / (++pointCount.at<int>(x_index, y_index));
        }
    }
    return true;
}

cv::Mat pc2cmProcessor::takeDoG(int kernel_size, double sigma1, double sigma2){ // https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#getgaussiankernel
    Mat first;
    mulTransposed(getGaussianKernel(kernel_size, sigma1, CV_64F), first, false); // multiply the first Guassian kernal by the transpose of it's self and store in first

    Mat second;
    mulTransposed(getGaussianKernel(kernel_size, sigma2, CV_64F), second,  false);
    DoG = Mat(first.size(), first.type());

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



costmap_2d::Costmap2DROS pc2cmProcessor::computeCostmap(){
    // costmap_2d costmap ;
    Mat out;
    filter2D(map, out, -1, DoG);
    std::cout << "out = "<< std::endl << " "  << out << std::endl << std::endl;

    //convert Mat to costmap_2d
    //https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#filter2d

}
