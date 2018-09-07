

#include <pc2_processor/pc2_processor.h>


#define CLAMP(A, B, C) ((A < B) ? B : ((A>C) ? C : A))


pc2cmProcessor::pc2cmProcessor(double grid_width)
{
    //grid is from +/- 2m on y and +3m on X
    float gridTotal[(int)(MAP_LENGTH/CELL_WIDTH)][(int) (MAP_WIDTH/CELL_WIDTH)] = {0};
    float gridNew[(int)(MAP_LENGTH/CELL_WIDTH)][(int) (MAP_WIDTH/CELL_WIDTH)] = {0};


    isOne = one == 1;
}

bool addPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg){
    //point (0,0) should map to index[0][.5*MAP_WIDTH]
    //point (1,0) should map to index[1/CELL_WIDTH][.5*MAP_WIDTH/CELL_WIDTH]
    // point (1,1) should map to index(1/CELL_WIDTH][1/CELL_WIDTH + .5*MAP_WIDTH/CELL_WIDTH
    int x_index = (int)((MAP_LENGTH - cloud_msg.at(i).z)/CELL_WIDTH );
    x_index = CLAMP(x_index, 0, MAP_LENGTH/CELL_WIDTH);

    int y_index = (int)((-cloud_msg.at(i).y)/CELL_WIDTH + (.5*MAP_WIDTH/CELL_WIDTH));
    y_index = CLAMP(y_index, 0, MAP_WIDTH/CELL_WIDTH);

        gridTotal[x_index][y_index] = (.90)*gridTotal[x_index][y_index]
                                        + .1*(-cloud_msg.at(i).x - gridTotal[x_index][y_index]);
}


bool pc2cmProcessor::getOne()
{
  return this->isOne;
}