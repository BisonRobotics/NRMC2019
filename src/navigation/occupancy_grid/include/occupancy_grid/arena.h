#include <vector>


#include <occupancy_grid/occupancy_grid.h>
#include <occupancy_grid/arena_dimensions.h>
#include <occupancy_grid/draw.h>

#ifndef OCCUPANCY_GRID_ARENA_H
#define OCCUPANCY_GRID_ARENA_H

namespace occupancy_grid
{

static const cv::Scalar_<double> occupied(100);

class Arena
{
  private:
    using AD = ArenaDimensions;

  public:
    OccupancyGrid rocks, walls, obstacles, inflated_rocks, inflated_walls, inflated_obstacles;

    Arena(std::vector<Circle> rock_locations, double cutoff=0.7) :
          rocks             (AD::height_cm, AD::width_cm),
          walls             (AD::height_cm, AD::width_cm),
          obstacles         (AD::height_cm, AD::width_cm),
          inflated_rocks    (AD::height_cm, AD::width_cm),
          inflated_walls    (AD::height_cm, AD::width_cm),
          inflated_obstacles(AD::height_cm, AD::width_cm)
    {
      // Populate rocks layer
      for (auto it = rock_locations.begin(); it != rock_locations.end(); it++)
      {
        draw(&rocks, (*it), occupied);
      }
      OccupancyGrid::inflate(&inflated_rocks, rocks, cutoff, 100, 2);

      // Populate wall layer
      double w = AD::width, h = AD::height;
      int thickness = 1;
      draw(&walls, Line(     0.0,       -w/2,      0.0, w/2 - 0.01), occupied, thickness);
      draw(&walls, Line(h - 0.01,       -w/2, h - 0.01, w/2 - 0.01), occupied, thickness);
      draw(&walls, Line(     0.0,       -w/2, h - 0.01,       -w/2), occupied, thickness);
      draw(&walls, Line(     0.0, w/2 - 0.01, h - 0.01, w/2 - 0.01), occupied, thickness);
      thickness = 75;
      draw(&inflated_walls, Line(     0.0,       -w/2,      0.0, w/2 - 0.01), occupied, thickness);
      draw(&inflated_walls, Line(h - 0.01,       -w/2, h - 0.01, w/2 - 0.01), occupied, thickness);
      draw(&inflated_walls, Line(     0.0,       -w/2, h - 0.01,       -w/2), occupied, thickness);
      draw(&inflated_walls, Line(     0.0, w/2 - 0.01, h - 0.01, w/2 - 0.01), occupied, thickness);
      OccupancyGrid::inflate(&inflated_walls, inflated_walls, 0.9, 20, 1);

      // Merge wall and rock layers
      OccupancyGrid::max(&obstacles, rocks, walls);
      OccupancyGrid::max(&inflated_obstacles, inflated_rocks, inflated_walls);
    }
};

}

#endif //OCCUPANCY_GRID_ARENA_H
