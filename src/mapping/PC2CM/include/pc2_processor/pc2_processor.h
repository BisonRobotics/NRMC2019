#ifndef PC2_PROCESSOR
#define PC2_PROCESSOR


#define _USE_MATH_DEFINES
#include <cmath>

//struct for input parameters like
//grid_width, min number of elements for a grid cell


//

class pc2cmProcessor
{
public:
    pc2cmProcessor(int one /*double grid_width, double grid_min_elements*/); //struct
    bool getOne();
/*
    bool addPoints(uint8_t* points_3d, uint32_t height, uint32_t row_step,
                   uint32_t point_step); 
    //adds points to internal grid (with smooting), returns true if good

    bool takeDoG(uint8_t* grid_2d, uint32_t height, uint32_t row_step,
                 uint32_t point_step, uint8_t* out_grid, float big, float small);
    //takes difference of gaussian on given data

*/
    
     //add points to grid
     //calculate new costmap
     //->median filter on grid?
     //->do DoG on height map
     //->project height map onto global costmap
private:
    bool isOne;
    //array containing average heights projected onto ground plane estimate
      //ground plane shoud be known from tf of camera to base_link 
        //(might get revised with time and data)
}; 

#endif