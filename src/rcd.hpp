#pragma once
#include <iostream>

#include "rcd_graph.hpp"
#include "map_handler.hpp"
#include "utilities.hpp"
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

namespace RCD
{
  class Core
  {
    private:
      struct CompareWeight {
          bool operator()(const RCD::RGraph::Node& a, const RCD::RGraph::Node& b) const {
              // Compare based on cast_w in descending order
              return a.cast_w < b.cast_w; // Change to > for descending order
          }
      };

      std::priority_queue<RCD::RGraph::Node, std::vector<RCD::RGraph::Node>, CompareWeight> weight_priority_queue;
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

      const float width_bias = 50.; // rectangle width bias for constraining search
      // ray's position during casting
      iPoint ray_pos; 
      
      RCD::RGraph::Node node2cast; // Node structure to be casted next
      RCD::RGraph::Node node2add;  // Node structure to be added to the graph
      RCD::RGraph::Edge edge2add;  // Edge structure to connect two nodes 

      std::vector<RCD::RGraph::Node> addNodeList; // Temp Node list that will be added to the graph at each iteration

      std::pair<bool,iPoint>  CheckIntersection(const iPoint & p);

      void PrepareCast();
      std::vector<iPoint> BresenhamLine(const iPoint & A, const iPoint & B); 
      void ConstraintSearchArea(float scale_rectangle);


    public:
      RCD::RGraph G; // Graph structure for this Core object

      static bool pathFound; // Flag set to true if the path is found
      static bool pathFoundByRobot;
      static RCD::RGraph::Node intersectionNode; // The node tha connects the two graphs
      static RCD::RGraph::EdgeDescriptor intersectionEdge_id; // The edge_id of the intersection

      Core(bool robot_flag,MapHandler *map_); // Constructor

      void CastRays();
      // void UpdateGrid();
      void UpdateGrid(const RCD::RGraph::Node &  source,const RCD::RGraph::Node & target);

      RCD::RGraph::Node AddIntersectionNode();
      std::vector<iPoint>  ShortestPath(RCD::RGraph::Node end_node);
      
      MapHandler* map; // pointer to the map object



  };
}
