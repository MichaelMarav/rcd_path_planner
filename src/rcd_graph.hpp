#pragma once 
#include <boost/graph/adjacency_list.hpp>

#include "utilities.hpp"
namespace RCD
{
  class RGraph
  {
  public:
    // Forward declaration
    struct Node;
    struct Edge;

    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, RGraph::Node, RGraph::Edge> BoostGraph;
  
    BoostGraph G;

    struct Node
    {
      Point pos;
      // Father?
      float cast_w;   // Cast weight   -> cast_w = f(p,q,e) TODO: figure out this function
      float p;        // Proximity     -> How close this Node is to target
      float e;        // Explorability -> Distance between this node and its father
      unsigned int o; // Occurence     -> How many times this node has already been casted
      BoostGraph::vertex_descriptor descriptor;
    
    };

    struct Edge
    {
      float d; // Eucledean distance between the two connecting nodes (for finding shortest path)
      // float ??
    };



    RGraph();
    void AddNode(RGraph::Node & node, BoostGraph & G);
    void AddEdge(const BoostGraph::vertex_descriptor & father, const BoostGraph::vertex_descriptor & child, BoostGraph & G, float weight);
    };
 }
