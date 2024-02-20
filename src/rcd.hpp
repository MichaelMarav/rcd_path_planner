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
      const int angle_increment{360/N_rays};
      unsigned int casting_dir;

      bool isRobot;
      RCD::RGraph::Node node_; // Node structure to be added at the graph
      RCD::RGraph::Edge edge_; // Edge structure to connect two nodes 

      RCD::RGraph G;
      
      RGraph::Node CastDecision();
      std::vector<unsigned int> casting_angles;
    public:
      Core(bool robot_flag,MapHandler *map_);

      bool pathFound{false}; // Flag set to true if the path is found
      void CastRays(MapHandler & handler);
      void Update();

      MapHandler* map;

  };
}
