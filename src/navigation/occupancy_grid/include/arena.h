#include <math.h>

#ifndef OCCUPANCY_GRID_ARENA_DIMENSIONS_H
#define OCCUPANCY_GRID_ARENA_DIMENSIONS_H

namespace occupancy_grid
{
  struct Arena
  {
    static constexpr double start_height        = 1.5;
    static constexpr double obstacle_height     = 2.94;
    static constexpr double obstacle_end_height = 4.44;
    static constexpr double mining_height       = 2.94;
    static constexpr double height              = 7.38;

    static constexpr double width               = 3.78;

    static constexpr double average_rock_width  = 0.2;
    static constexpr double max_rock_width      = 0.3;
    static constexpr double min_rock_width      = 0.1;

    static constexpr double max_crater_width    = 0.3;
    static constexpr double min_crater_width    = 0.1;

    static constexpr int number_of_craters      = 2;
    static constexpr int number_of_rocks        = 3;

    static constexpr int max_cost               = 255;

    static constexpr int crater_cost            = 255;
    static constexpr int rock_cost              = 255;

    static constexpr double resolution          = 0.01;

    static constexpr int height_cm              = 738;
    static constexpr int width_cm               = 378;
  };
}


#endif //OCCUPANCY_GRID_ARENA_DIMENSIONS_H
