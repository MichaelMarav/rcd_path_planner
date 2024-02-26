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
      const int NUM_RAYS{6}; 
    
      // Symmetric angle increment dependent on number of rays (e.g. 6 rays = 60 deg., 3 rays 120 deg.) 
      const float angle_increment = static_cast<float>(2.0 * PI / static_cast<float>(NUM_RAYS));
    
      // Casted ray travelling distance
      float ray_dis; 

      // Random initial casting direction    
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

      std::vector<RCD::RGraph::Node> addNodeList; // Temp Node list that will be added to the graph at each iteration

      
      // Returns which node to cast next based on the weights 
      RGraph::Node& CastDecision();
      //TODO: Add heap or set or queue for stroring the pointers to the new nodes and update the weights
    

    public:
      RCD::RGraph G; // Graph structure for this Core object

      static bool pathFound; // Flag set to true if the path is found
      static bool pathFoundByRobot;
      static RCD::RGraph::Node intersectionNode; // The node tha connects the two graphs
      static RCD::RGraph::EdgeDescriptor intersectionEdge_id; // The edge_id of the intersection

      Core(bool robot_flag,MapHandler *map_); // Constructor

      void CastRays();
      void PrepareCasting();
      void UpdateGrid();

      std::vector<Point>  ShortestPath(RCD::RGraph::Node end_node);
      MapHandler* map; // pointer to the map object

  };
  // RCD::RGraph::Node Core::intersectionNode;
  // RCD::RGraph::Node Core::intersectionNode; // Definition of static member variable
}
