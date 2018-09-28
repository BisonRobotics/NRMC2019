/**
 * Helpful link on point clouds see slide 48: http://ais.informatik.uni-freiburg.de/teaching/ws10/robotics2/pdfs/rob2-12-ros-pcl.pdf
 *
 * */

#include <pc2_processor/pc2_processor.h>


#define CLAMP(A, B, C) ((A < B) ? B : ((A>C) ? C : A))


std::vector<std::vector<mapPoint>> map; // a grid of heights defined by Cell_width, Grid_Width, GRID_HEIGHT

double GRID_WIDTH;
double GRID_HEIGHT;
double CELL_WIDTH;

pc2cmProcessor::pc2cmProcessor(double cell_width, double grid_width, double GRID_HEIGHT)
{
    CELL_WIDTH = cell_width;
    GRID_HEIGHT = GRID_HEIGHT;
    GRID_WIDTH = grid_width;

    //grid is from +/- 2m on y and +3m on X
    for(int i =0; i <(int)(GRID_HEIGHT/CELL_WIDTH)+1; i++){
        std::vector<mapPoint> col;
        for(int j =0; j <(int)(GRID_WIDTH/CELL_WIDTH); j++){
            mapPoint mp;
            col.push_back(mp);
        }
        map.push_back(col);
    }

    std::cerr << "created pc2mProccessor!!" << std::endl;

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

            map.at(x_index).at(y_index).height = map.at(x_index).at(y_index).height +
                                                (pt.x - map.at(x_index).at(y_index).height)
                                                / (++map.at(x_index).at(y_index).totalPoints);
        }
    }
    return true;
}

bool pc2cmProcessor::addPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg){

    // std::cerr << " adding points. Size of cloud = "<< cloud_msg.points.size() << std::endl;

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

            map.at(x_index).at(y_index).height = map.at(x_index).at(y_index).height +
                                                (pt.x - map.at(x_index).at(y_index).height)
                                                / (++map.at(x_index).at(y_index).totalPoints);
        }
    }
    return true;
}

// double[][] pc2cmProcessor::takeDoG(double radius_1, double radius_2, double sigma){ // https://www.geeksforgeeks.org/gaussian-filter-generation-c/

//     // intialising standard deviation to 1.0

//     // sum is for normalization
//     double sum = 0.0;

//     // generating 5x5 kernel
//     for (int x = -2; x <= 2; x++) {
//         for (int y = -2; y <= 2; y++) {
//             // r = sqrt(x * x + y * y);
//             GKernel[x + 2][y + 2] = (exp(-(radius_1 * radius_1) / sigma)) / (M_PI * sigma);
//             sum += GKernel[x + 2][y + 2];
//         }
//     }

//     // normalising the Kernel
//     for (int i = 0; i < 5; ++i){
//         for (int j = 0; j < 5; ++j){
//             GKernel[i][j] /= sum;
//         }
//     }
// }

double pc2cmProcessor::get_Height(int xindex, int yindex){
    return map.at(xindex).at(yindex).height;
}

void pc2cmProcessor::print_grid(void){
    std::cout << "printing grid:\n x, y, height, total points" << std::endl;
    for ( std::vector<int>::size_type i = 0; i != map.size(); i++) {
        for (std::vector<int>::size_type j = 0; j != map[i].size(); j++) {
            std::cerr << " "<< i << "  "<< j << "     "<< map[i][j].height << "     "<<  map[i][j].totalPoints <<std::endl;
        }
    }

}


costmap_2d::Costmap2DROS pc2cmProcessor::computeCostmap(){
//1. Take the FFT, which will yield another 512x512 matrix


}
