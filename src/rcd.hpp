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

      const int NUM_RAYS{6};  // Number of rays to be casted
    
      // Symmetric angle increment dependent on number of rays (e.g. 6 rays = 60 deg., 3 rays 120 deg.) 
      const float angle_increment = static_cast<float>(2.0 * PI / static_cast<float>(NUM_RAYS));
    
      const float width_bias = 50.; // rectangle width bias for constraining search

      const float ray_start_dis = 5.0;
      float scaleRectangle_;
      float ray_dis;   // Casted ray travelling distance


      // Random initial casting direction    
      float casting_dir;
      
      // List with the casting angles -> casting_dir + angle_increment * i
      std::vector<float> casting_angles;  


      // Flag for describing the rcd object, Either robot casting or target 
      bool isRobot;

      // ray's position during casting
      iPoint ray_pos; 
      
      RCD::RGraph::Node node2cast; // Node structure to be casted next
      RCD::RGraph::Node node2add;  // Node structure to be added to the graph
      RCD::RGraph::Edge edge2add;  // Edge structure to connect two nodes 

      std::vector<RCD::RGraph::Node> addNodeList; // Temp Node list that will be added to the graph at each iteration

      std::pair<bool,iPoint>  CheckIntersection(const iPoint & p);

      void CastDecision();
      
      std::vector<iPoint> BresenhamLine(const iPoint & A, const iPoint & B); 

      void ConstraintSearchArea();

      void ImposeRectangle(const std::vector<iPoint> & line);

    public:
      Core(bool robot_flag,MapHandler *map_, float scaleRectangle); // Constructor

      RCD::RGraph G; // Graph structure for this Core object

      bool pathFound = false;
      RCD::RGraph::Node intersectionNode; // The node tha connects the two graphs
      RCD::RGraph::EdgeDescriptor intersectionEdge_id; // The edge_id of the intersection


      void CastRays();
      // void UpdateGrid();
      void UpdateGrid(const RCD::RGraph::Node &  source,const RCD::RGraph::Node & target);

      RCD::RGraph::Node AddIntersectionNode(RGraph::Node intersection,RGraph::EdgeDescriptor intersectionEdgeID);
      std::vector<iPoint>  ShortestPath(RCD::RGraph::Node end_node);
      
      MapHandler* map; // pointer to the map object



  };
}
