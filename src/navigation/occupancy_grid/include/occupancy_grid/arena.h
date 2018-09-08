#include <vector>

#include <occupancy_grid/occupancy_grid.h>
#include <occupancy_grid/arena_dimensions.h>

#ifndef OCCUPANCY_GRID_ARENA_H
#define OCCUPANCY_GRID_ARENA_H

namespace occupancy_grid
{

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
        rocks.draw((*it));
      }
      OccupancyGrid::inflate(rocks, &inflated_rocks, cutoff);

      // Populate wall layer
      double w = AD::width, h = AD::height;
      int thickness = 1;
      walls.draw(Line(     0.0,       -w/2,      0.0, w/2 - 0.01), thickness);
      walls.draw(Line(h - 0.01,       -w/2, h - 0.01, w/2 - 0.01), thickness);
      walls.draw(Line(     0.0,       -w/2, h - 0.01,       -w/2), thickness);
      walls.draw(Line(     0.0, w/2 - 0.01, h - 0.01, w/2 - 0.01), thickness);
      thickness = 75;
      inflated_walls.draw(Line(     0.0,       -w/2,      0.0, w/2 - 0.01), thickness);
      inflated_walls.draw(Line(h - 0.01,       -w/2, h - 0.01, w/2 - 0.01), thickness);
      inflated_walls.draw(Line(     0.0,       -w/2, h - 0.01,       -w/2), thickness);
      inflated_walls.draw(Line(     0.0, w/2 - 0.01, h - 0.01, w/2 - 0.01), thickness);
      OccupancyGrid::inflate(inflated_walls, &inflated_walls, 0.9, 20, 1);


      // Merge wall and rock layers
      OccupancyGrid::max(rocks, walls, &obstacles);
      OccupancyGrid::max(inflated_rocks, inflated_walls, &inflated_obstacles);
    }
};

}

#endif //OCCUPANCY_GRID_ARENA_H
