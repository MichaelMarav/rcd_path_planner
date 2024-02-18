#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

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
    unsigned int robot_pos;
    unsigned int target_pos;

    MapHandler(const std::string & ppm_filemame);

  private:
    void loadOccupancyGrid(const std::string & ppm_filemame);
    void initializeGrid(unsigned int width, unsigned int height);
};