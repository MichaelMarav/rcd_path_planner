#pragma once

#include "utilities.hpp"
/*
  Stores the Occupancy Grid and other parameters for the map. It gets updated
  as the RCD runs
*/
template<typename T>
using OccupancyGrid = std::vector<std::vector<T>>;

class MapHandler
{
  public:
    struct Cell
    {
      bool isOccupied;
      bool robotPass;
      bool targetPass;
    };

    OccupancyGrid<Cell> grid;  // 2D occupancy grid
    
    unsigned int width;
    unsigned int height;

    Point robot_pos;
    Point target_pos;
    float grid_size_x;
    float grid_size_y;
    float grid_resolution;



    MapHandler(const std::string & map_file);

  private:
    std::string yaml_filename;
    std::string ppm_filename;
    void loadOccupancyGrid(const std::string & map_file);
    void loadOccupancyParams(const std::string & yaml_file);
    void initializeGrid(unsigned int width, unsigned int height);
};