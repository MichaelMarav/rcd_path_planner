#pragma once 
#include <boost/graph/adjacency_list.hpp>


namespace RCD
{
  class RGraph
  {
  public:
    struct Point
    {
      unsigned int x;
      unsigned int y;
    };
    // Use namespace to avoid errors
    struct Node
    {
      Point pos;
      // Father?
      float cast_w;   // Cast weight   -> cast_w = f(p,q,e) TODO: figure out this function
      float p;        // Proximity     -> How close this Node is to target
      float e;        // Explorability -> Distance between this node and its father
      unsigned int o; // Occurence     -> How many times this node has already been casted

    };

    struct Edge
    {
      float d; // Eucledean distance between the two connecting nodes
      // float ??
    };

    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Node, Edge> G;


    RGraph(){};
    void AddNode(const RGraph::Node & node);
    void AddEdge(const G::vertex_descriptor & father, const G::vertex_descriptor & child, float weight);
    };
 }
