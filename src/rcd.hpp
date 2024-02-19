#pragma once
#include <iostream>

#include "rcd_graph.hpp"
#include "map_handler.hpp"

namespace RCD
{
  class Core
  {
    private:
      const int N_rays{6}; // Number of rays to be casted
      bool isRobot;
      RGraph::Node node_;
      RGraph::Edge edge_;

      RGraph graph;

    public:
      bool pathFound{false}; // Flag set to true if the path is found
      Core(bool robot_flag);
      // void cast(MapHandler & grid);
  };
}
