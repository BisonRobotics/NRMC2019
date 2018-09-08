

#include <pc2_processor/pc2_processor.h>


#define CLAMP(A, B, C) ((A < B) ? B : ((A>C) ? C : A))
// #define Z_LOWER -8.0f
// #define Z_UPPER 8.0f

// #define GRID_WIDTH 3.0f
// #define GRID_LENGTH 3.0f
// #define CLAMP(A, B, C) ((A < B) ? B : ((A>C) ? C : A))

std::vector<std::vector<float>> masterGrid; // a grid of heights defined by Cell_width, Grid_Width, Grid_length

bool isOne;
double GRID_WIDTH;
double GRID_LENGTH;
double CELL_WIDTH;

pc2cmProcessor::pc2cmProcessor(double cell_width, double grid_width, double grid_length)
{
    CELL_WIDTH = cell_width;
    GRID_LENGTH = grid_length;
    GRID_WIDTH = grid_width;

    //grid is from +/- 2m on y and +3m on X
    for(int i =0; i <(int)(GRID_LENGTH/CELL_WIDTH); i++){
        std::vector<float> col;
        for(int i =0; i <(int)(GRID_WIDTH/CELL_WIDTH); i++){
            col.push_back(0.0);
        }
        masterGrid.push_back(col);
    }

    std::cerr << "created pc2mProccessor!!" << std::endl;

    // isOne = true;
}

// bool addPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg){
bool pc2cmProcessor::addPoints(pcl::PointCloud<pcl::PointXYZ> cloud_msg){

    // for (const pcl::PointXYZ& pt : cloud_msg->points)
    for (size_t i = 0; i < cloud_msg.points.size(); ++i)
    {
        pcl::PointXYZ pt = cloud_msg.points[i];
        // ROS_INFO("%f, %f, %f", pt.x, pt.y, pt.z);
        if (!(pt.x ==0 && pt.y ==0 && pt.z ==0))
        {
            //point (0,0) should map to index[0][.5*GRID_WIDTH]
            //point (1,0) should map to index[1/CELL_WIDTH][.5*GRID_WIDTH/CELL_WIDTH]
            // point (1,1) should map to index(1/CELL_WIDTH][1/CELL_WIDTH + .5*GRID_WIDTH/CELL_WIDTH
            int x_index = (int)((GRID_LENGTH - pt.z)/CELL_WIDTH );
            x_index = CLAMP(x_index, 0, GRID_LENGTH/CELL_WIDTH);

            int y_index = (int)((-pt.y)/CELL_WIDTH + (.5*GRID_WIDTH/CELL_WIDTH));
            y_index = CLAMP(y_index, 0, GRID_WIDTH/CELL_WIDTH);

            masterGrid.at(x_index).at(y_index) = (.90)*masterGrid.at(x_index).at(y_index)
                                            + .10*(-pt.x - masterGrid.at(x_index).at(y_index)); // simple filter
        }
    }
    return true;
}


bool pc2cmProcessor::getOne()
{
    return true;
}