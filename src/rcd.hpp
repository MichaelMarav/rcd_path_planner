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
      // Number of rays to be casted
      const int N_rays{6}; 
    
      // Float PI
      const float PI{static_cast<float>(M_PI)}; 
    
      // Symmetric angle increment dependent on number of rays (e.g. 6 rays = 60 deg., 3 rays 120 deg.) 
      const float angle_increment = static_cast<float>(2.0 * static_cast<float>(PI) / static_cast<float>(N_rays));
    
      // Casted ray travelling distance
      float ray_dis; 

      // Random initiali casting direction    
      float casting_dir;
      
      // List with the casting angles -> casting_dir + angle_increment * i
      std::vector<float> casting_angles;  

      // Flag for describing the rcd object, Either robot casting or target 
      bool isRobot;

      float cos_cast; // cos of casting angle
      float sin_cast; // sin of casting angle

      // ray's position during casting
      Point ray_pos; 
      
      RCD::RGraph::Node node2cast; // Node structure to be casted next
      RCD::RGraph::Node node2add;  // Node structure to be added to the graph
      RCD::RGraph::Edge edge2add;  // Edge structure to connect two nodes 

      RCD::RGraph::BoostGraph::vertex_descriptor father;
      RCD::RGraph::BoostGraph::vertex_descriptor child;
      RCD::RGraph G; // Graph structure for this Core object
      
      // Returns which node to cast next based on the weights 
      RGraph::Node& CastDecision();
      //TODO: Add heap or set or queue for stroring the pointers to the new nodes and update the weights
    public:
      Core(bool robot_flag,MapHandler *map_); // Constructor

      bool pathFound{false}; // Flag set to true if the path is found
      void CastRays();
      void Update();

      MapHandler* map; // pointer to the map object

  };
}
