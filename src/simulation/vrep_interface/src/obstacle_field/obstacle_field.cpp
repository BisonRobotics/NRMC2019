#include <obstacle_field/obstacle_field.h>
#include <tf/LinearMath/Quaternion.h>


using namespace vrep_interface;


Crater::Crater()
{
  dimensions[0] = 0.3f;
  dimensions[1] = 0.3f;
  dimensions[2] = 0.3f;

  position[0] = 0;
  position[1] = 0;

  resolution = 1e-2; // 1 cm resolution
}


Crater::Crater(simFloat *position) : Crater()
{
  this->resolution = resolution;
  setPosition(position);
}


Crater::Crater(simFloat *dimensions, simFloat *position) : Crater(position)
{
  for (int i = 0; i < 3; i++)
  {
    this->dimensions[i] = dimensions[i];
  }
}


void Crater::setDimensions(simFloat *dimensions)
{

  for (int i = 0; i < 3; i++)
  {
    this->dimensions[i] = dimensions[i];
  }
}


void Crater::getDimensions(simFloat *dimensions)
{
  for (int i = 0; i < 3; i++)
  {
    dimensions[i] = this->dimensions[i];
  }
}


void Crater::setPosition(simFloat *position)
{
  this->position[0] = (simInt) std::round(position[0] / resolution);
  this->position[1] = (simInt) std::round((position[1] + field_size_f[1] / 2) / resolution);
}


void Crater::getPosition(simFloat *position)
{
  for (int i = 0; i < 3; i++)
  {
    position[i] = ((simFloat)this->position[i]) * resolution;
  }
}


void Crater::generateCrater(const simFloat *obstacle_field_ptr)
{
  // Build crater
  simInt crater_x = (simInt)std::round(dimensions[0] / resolution);
  simInt crater_y = (simInt)std::round(dimensions[1] / resolution);

  simFloat crater[crater_y][crater_x];

  for (int i = 0; i < crater_y; i++) {
    for (int j = 0; j < crater_x; j++) {
      crater[i][j] = -dimensions[0]
                     * std::sin(((float)i) / ((float)crater_y) * 3.14f)
                     * std::sin(((float)j) / ((float)crater_x) * 3.14f);
    }
  }

  // Make the crater look less weird
  float lowest_edge = 0.0;
  for (int i = 0; i < crater_y; i++) {
    if (crater[i][0] < lowest_edge)
    {
      lowest_edge = crater[i][0];
    }
    if (crater[i][crater_x - 1] < lowest_edge)
    {
      lowest_edge = crater[i][crater_x - 1];
    }
  }
  for (int i = 0; i < crater_x; i++) {
    if (crater[0][i] < lowest_edge)
    {
      lowest_edge = crater[0][i];
    }
    if (crater[crater_y - 1][i] < lowest_edge)
    {
      lowest_edge = crater[crater_y - 1][i];
    }
  }

  // Add crater to height map
  simFloat(*obstacle_field)[field_size_i[0]] = (simFloat(*)[field_size_i[0]])obstacle_field_ptr;
  for (int i = 0; i < crater_y; i++) {
    for (int j = 0; j < crater_x; j++) {
      obstacle_field[i + position[1]][j + position[0]] =
          obstacle_field[i + position[1]][j + position[0]]
          + crater[i][j] - lowest_edge;
      if (obstacle_field[i + position[1]][j + position[0]] > 0.0f)
      {
        obstacle_field[i + position[1]][j + position[0]] = 0.0f;
      }
    }
  }
}


Rock::Rock()
{
  number = -1;

  dimensions[0] = 0.3f;
  dimensions[1] = 0.3f;
  dimensions[2] = 0.3f;

  position[0] = 0.0f;
  position[1] = 0.0f;
}


Rock::Rock(simInt rock_number, simFloat *position, std::string models_path) : Rock()
{
  initialize(rock_number, models_path);
  for (int i = 0; i < 2; i++)
  {
    this->position[i] = position[i];
  }
}


Rock::Rock(simInt rock_number, simFloat *position, std::string models_path,
           simFloat *dimensions) : Rock(rock_number, position, models_path)
{
  for (int i = 0; i < 3; i++)
  {
    this->dimensions[i] = dimensions[i];
  }
}


void Rock::initialize(simInt rock_number, std::string models_path)
{
  this->number = rock_number;

  handle = simLoadModel((models_path + this->toString()).c_str());
  if (handle == -1)
  {
    simAddStatusbarMessage(("Unable to load " + this->toString()).c_str());
  }

  // Calculate scaling factors
  simFloat scaling_factors[3];
  for (int i = 0; i < 3; i++)
  {
    scaling_factors[i] = dimensions[i] / 0.1f;
  }

  // Scale object
  simScaleObject(handle, scaling_factors[0], scaling_factors[1], scaling_factors[2], 0);

  // Scale mesh
  simInt mesh_handle = simGetObjectChild(handle, 0);
  simScaleObject(mesh_handle, scaling_factors[0], scaling_factors[1], scaling_factors[2], 0);

  // Scale current_position and move
  simFloat current_position[3];
  if(simGetObjectPosition(handle, -1, current_position) == -1)
  {
    simAddStatusbarMessage(("Unable to get current_position for " + this->toString()).c_str());
  }
  position[0] = this->position[0];
  position[1] = this->position[1];
  position[2] = current_position[2] * scaling_factors[2];

  place();
}


void Rock::getDimensions(simFloat dimensions[3])
{
  for (int i = 0; i < 3; i++)
  {
    dimensions[i] = this->dimensions[i];
  }
}


void Rock::setPosition(simFloat *position)
{
  this->position[0] = position[0] + field_offset[0];
  this->position[1] = position[1] + field_offset[1];
}


void Rock::getPosition(simFloat position[2])
{
  for (int i = 0; i < 3; i++)
  {
    position[i] = this->position[i];
  }
}


simInt Rock::getNumber()
{
  return number;
}


simInt Rock::getHandle()
{
  return handle;
}


std::string Rock::toString()
{
  std::string rock_name;

  if (number < 10){
     rock_name = "rock_0" + std::to_string(number) + ".ttm";
  }
  else
  {
    rock_name = "rock_" + std::to_string(number) + ".ttm";
  }

  return rock_name;
}


void Rock::place()
{
  simInt result = simSetObjectPosition(handle, -1, position);
  if(result == -1)
  {
    simAddStatusbarMessage(("Unable to set current_position for " + this->toString()).c_str());
  }
}


ObstacleField::ObstacleField()
{
  obstacle_field_ptr = (simFloat *)&obstacle_field;
}

void ObstacleField::initialize_occupancy_grid()
{
  occupancy_grid.header.seq = 0;
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.header.stamp = ros::Time::now();

  occupancy_grid.info.map_load_time = ros::Time::now();
  occupancy_grid.info.resolution = resolution;
  occupancy_grid.info.height = ARENA_SIZE_Y + 200; // TODO seems backwards to me, but works in rviz
  occupancy_grid.info.width  = ARENA_SIZE_X + 200;

  tf::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, 0.0);
  occupancy_grid.info.origin.orientation.x = orientation.x();
  occupancy_grid.info.origin.orientation.y = orientation.y();
  occupancy_grid.info.origin.orientation.z = orientation.z();
  occupancy_grid.info.origin.orientation.w = orientation.w();
  occupancy_grid.info.origin.position.x = -1.0;
  occupancy_grid.info.origin.position.y = -field_size_f[1] / 2.0 - 1;
  occupancy_grid.info.origin.position.z = 0.05;

  occupancy_grid.data.resize((ARENA_SIZE_X+200) * (ARENA_SIZE_Y+200));
}


void ObstacleField::get_occupancy_grid(nav_msgs::OccupancyGrid *map)
{
  occupancy_grid.header.seq++;
  occupancy_grid.header.stamp = ros::Time::now();
  *map = occupancy_grid;
}


void ObstacleField::initialize(std::string models_path)
{
  for(int i = 0; i < 3; i++)
  {
    rocks[i].initialize(i+1, models_path);
  }
}


void ObstacleField::zeroField()
{
  for (int i = 0; i < field_size_i[1]; i++)
  {
    for (int j = 0; j < field_size_i[0]; j++)
    {
      obstacle_field[i][j] = 0.0;
    }
  }

  for (int i = 0; i < occupancy_grid.data.size(); i++)
  {
    occupancy_grid.data[i] = 0;
  }
}


void ObstacleField::generateFieldRandom()
{
  // Position craters
  simFloat crater_positions[2][2];
  crater_positions[0][0] =  (rand() % (field_size_i[0] - 40)) * resolution;
  crater_positions[1][0] =  (rand() % (field_size_i[0] - 40)) * resolution;
  crater_positions[0][1] =  (rand() % (simInt)std::round(field_size_i[1] / 2) - 40) * resolution;
  crater_positions[1][1] = -(rand() % (simInt)std::round(field_size_i[1] / 2) - 40) * resolution;

  simFloat rock_positions[3][2];
  for (int i = 0; i < 3; i++)
  {
    rock_positions[i][0] = (rand() % (field_size_i[0] - (int) std::round(0.4f / resolution))) * resolution;
    rock_positions[i][1] = (rand() % (field_size_i[1] - (int) std::round(0.4f / resolution))) * resolution;
  }

  generateField(crater_positions, rock_positions);
}


void ObstacleField::generateField(simFloat crater_positions[3][2], simFloat rock_positions[3][2])
{
  initialize_occupancy_grid();
  zeroField();

  // Set crater positions
  for (int i = 0; i < 2; i++)
  {
    craters[i].setPosition(crater_positions[i]);
    craters[i].generateCrater(obstacle_field_ptr);
  }

  // Rotate crater array
  uint8_t field[FIELD_SIZE_X][FIELD_SIZE_Y];
  for (int i = 0; i < FIELD_SIZE_Y; i++)
  {
    for (int j = 0; j < FIELD_SIZE_X; j++)
    {
      if (fabs(obstacle_field[FIELD_SIZE_Y - 1 - i][j]) > 0.02)
      {
        field[j][i] = 100;
      }
      else
      {
        field[j][i] = 0;
      }
    }
  }

  // Add craters
  for (int i = 0; i < FIELD_SIZE_X; i++)
  {
    for (int j = 0; j < FIELD_SIZE_Y; j++)
    {
      occupancy_grid.data[(ARENA_SIZE_X + 200) * (j+100) + i + 150 + 100] = field[i][j];
    }
  }

  // Place and configure the height field in VREP
  this->setHeightField();
  this->setProperties();

  // Add the rocks
  for (int i = 0; i < 3; i++)
  {
    rocks[i].setPosition(rock_positions[i]);
    rocks[i].place();
  }

  // Add rocks
  for (int i = 0; i < ARENA_SIZE_X; i++)
  {
    for (int j = 0; j < FIELD_SIZE_Y; j++)
    {
      for (int k = 0; k < 3; k++)
      {
        simFloat pos[3];
        rocks[k].getPosition(pos);
        int x = (int)(pos[0]/resolution);
        int y = (int)(pos[1]/resolution + 378/2);
        if ((((x-12) < i) && ((x+12) > i)) && (((y-12) < j) && ((y+12) > j)))
        {
          occupancy_grid.data[(ARENA_SIZE_X + 200) * (j+100) + i + 100] = 100;
        }
      }
    }
  }

  // Add border
  for (int i = 0; i < ARENA_SIZE_X+200; i++)
  {
    for (int j = 0; j < ARENA_SIZE_Y+200; j++)
    {
      if ((i < 100) || (i > (ARENA_SIZE_X + 100)) || (j < 100) || (j > (ARENA_SIZE_Y + 100)))
      {
        occupancy_grid.data[(ARENA_SIZE_X + 200) * j + i] = 100;
      }
    }
  }
}


void ObstacleField::setProperties()
{
  simFloat color[3] = {1.00, 0.87, 0.87};
  simSetShapeColor(handle, NULL, sim_colorcomponent_ambient_diffuse, (simFloat *)&color);
  simSetObjectInt32Parameter(handle, sim_shapeintparam_respondable, 1);
  simSetObjectInt32Parameter(handle, sim_shapeintparam_static, 1);
  simSetObjectName(handle, "obstacle_field");
}


void ObstacleField::setHeightField()
{
  // Create a height field from the obstacle field
  handle = simGetObjectHandle("obstacle_field");
  if (handle != -1){
    simRemoveObject(handle);
  }
  handle = simCreateHeightfieldShape(0, 10, field_size_i[0], field_size_i[1], field_size_f[0], obstacle_field_ptr);

  // Place the height field
  simFloat pos[3] = {0.0, 0.0, 0.0};
  const simFloat *pos_ptr = (simFloat *)&pos;
  simGetObjectPosition(handle, -1, (simFloat *)&pos);
  pos[0] = field_position[0];
  pos[1] = field_position[1];
  simSetObjectPosition(handle, -1, pos_ptr);
}