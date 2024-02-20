#pragma once
#include <iostream>

#include "rcd_graph.hpp"
#include "map_handler.hpp"
#include "utilities.hpp"
namespace RCD
{
  class Core
  {
    private:
      const int N_rays{6}; // Number of rays to be casted
      const float PI{static_cast<float>(M_PI)}; 
      // const float angle_increment{2.*PI/static_cast<float>(N_rays)};
      const float angle_increment = static_cast<float>(2.0 * static_cast<float>(PI) / static_cast<float>(N_rays));
      
      float ray_dis; // Casted ray travelling distance
      float casting_dir;
      bool isRobot;
      // bool stopBeam = false;
      float cos;
      float sin;
      Point beam;
      RCD::RGraph::Node node2cast; // Node structure to be casted next
      RCD::RGraph::Node node2add;  // Node structure to be added at the graph
      RCD::RGraph::Edge edge2add;  // Edge structure to connect two nodes 

      RCD::RGraph G;
      
      RGraph::Node CastDecision();
      std::vector<unsigned int> casting_angles;
    public:
      Core(bool robot_flag,MapHandler *map_); // Constructor

      bool pathFound{false}; // Flag set to true if the path is found
      void CastRays();
      void Update();

      MapHandler* map; // pointer to the map object

  };
}
