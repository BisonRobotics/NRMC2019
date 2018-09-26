#ifndef PROJECT_OBSTACLE_FIELD_H_H
#define PROJECT_OBSTACLE_FIELD_H_H


#include <string>
#include <cmath>
#include <vector>
#include <vrep_lib/v_repLib.h>
#include <nav_msgs/OccupancyGrid.h>


namespace vrep_interface
{
// In centimeters
#define FIELD_SIZE_X 294
#define FIELD_SIZE_Y 378
#define ARENA_SIZE_X 738
#define ARENA_SIZE_Y 378

const simFloat resolution = 1e-2; // 1 cm resolution
const simFloat field_size_f[2]   = {FIELD_SIZE_X * resolution, FIELD_SIZE_Y * resolution};
const simInt   field_size_i[2]   = {FIELD_SIZE_X, FIELD_SIZE_Y};
const simFloat field_position[2] = {2.97f, 0.00f};
const simFloat field_offset[2]   = {field_position[0] - field_size_f[0] / 2,
                                    field_position[1] - field_size_f[1] / 2};


class Crater
{
public:
  Crater();
  Crater(simFloat position[3]);
  Crater(simFloat dimensions[3], simFloat position[3]);

  // Getters and setters
  void setDimensions(simFloat *dimensions);
  void getDimensions(simFloat *dimensions);
  void setPosition(simFloat *position);
  void getPosition(simFloat *position);

  void generateCrater(const simFloat *obstacle_field_ptr);

private:
  simFloat dimensions[3];
  simInt position[2];
  simFloat resolution;
};


class Rock
{
public:
  Rock();
  Rock(simInt rock_number, simFloat *position, std::string models_path);
  Rock(simInt rock_number, simFloat *position, std::string models_path, simFloat *dimensions);

  void initialize(simInt rock_number, std::string models_path);
  void getDimensions(simFloat *dimensions);
  void setPosition(simFloat *position);
  void getPosition(simFloat *position);
  void place();
  simInt getNumber();
  simInt getHandle();
  std::string toString();


private:
  simFloat dimensions[3];
  simFloat position[3];
  simInt number;
  simInt handle;
};


/*
 * Info on obstacle field
 *
 * Surface features will consist of two craters on each side of the arena with three, randomly placed obstacles.
 *
 * There will be three obstacles placed on top of the compressed BP-1 surface within the obstacle
 * area before each competition attempt is made. The placement of the obstacles will be randomly
 * selected before the start of the competition. Each obstacle will have a diameter of approximately
 * 10 to 30 cm and an approximate mass of 3 to 10 kg. There will be two craters of varying depth and
 * width, being no wider or deeper than 30 cm. No obstacles will be intentionally buried in the BP-1
 * by NASA, however, BP-1 includes naturally occurring rocks.
 */
class ObstacleField
{
public:
  ObstacleField();

  void initialize(std::string models_path);
  void generateFieldRandom();
  void generateField(simFloat crater_positions[3][2], simFloat rock_positions[3][2]);

  void initialize_occupancy_grid();
  void get_occupancy_grid(nav_msgs::OccupancyGrid *map);


private:
  simFloat obstacle_field[FIELD_SIZE_Y][FIELD_SIZE_X];
  const simFloat *obstacle_field_ptr;
  simInt handle;
  Crater craters[2];
  Rock rocks[3];
  nav_msgs::OccupancyGrid occupancy_grid;

  void zeroField();
  void setHeightField();
  void setProperties();
};
}

#endif //PROJECT_OBSTACLE_FIELD_H_H
