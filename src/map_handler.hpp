#pragma once

#include "utilities.hpp"  
#include "rcd_graph.hpp"
/*
  Stores the Occupancy Grid and other parameters for the map. It gets updated
  as the RCD runs
*/
template<typename T>
using OccupancyGrid = std::vector<std::vector<T>>;
using EdgeDescriptor = boost::graph_traits<RCD::RGraph::BoostGraph>::edge_descriptor;

class MapHandler
{
  public:
    struct Cell
    {
      bool isOccupied;        // True for occupied space
      bool robotPass;         // True if robot-produced ray has passed through that cell
      bool targetPass;        // True if target-produced ray has passed through that cell
      EdgeDescriptor edge_id; // Stores the edge descriptor that connects the nodes that pass through
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