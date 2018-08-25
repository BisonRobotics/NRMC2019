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
     pc2cmProcessor(double grid_width, double grid_min_elements); //struct
     bool getOne();
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