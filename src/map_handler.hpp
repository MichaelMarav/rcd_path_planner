#pragma once

#include "utilities.hpp"  
#include "rcd_graph.hpp"


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
      bool isOccupied;        // True for occupied space 
      bool robotPass;         // True if robot-produced ray has passed through that cell
      bool targetPass;        // True if target-produced ray has passed through that cell
      RCD::RGraph::EdgeDescriptor edge_id; // Stores the edge descriptor that connects the nodes that pass through
    };

    OccupancyGrid<Cell> grid;  // 2D occupancy grid --> This occupancy grid will be inflated for the casting
    OccupancyGrid<Cell> deflated_grid; // This grid will be deflated for the optimization step
    OccupancyGrid<Cell> inflated_grid; // This grid will be deflated for the optimization step

    float grid_resolution; // in m
    int map_coverage;
    // Geometry of environment (m)
    unsigned int width;
    unsigned int height;

    Point robot_pos;
    Point target_pos;

    MapHandler(const std::string & map_file); // Constructor

  private:
    std::string yaml_filename;
    std::string ppm_filename;
    void loadOccupancyGrid(const std::string & map_file);
    void loadOccupancyParams(const std::string & yaml_file);
    void initializeGrid(unsigned int width, unsigned int height);
};