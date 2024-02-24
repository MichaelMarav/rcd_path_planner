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
    typedef boost::graph_traits<RCD::RGraph::BoostGraph>::edge_descriptor    EdgeDescriptor;
    typedef boost::graph_traits<RCD::RGraph::BoostGraph>::vertex_descriptor  NodeDescriptor;

    BoostGraph G;

    struct Node
    {
      Point pos;
      // Father?
      float cast_w;   // Cast weight   -> cast_w = f(p,q,e) TODO: figure out this function
      float p;        // Proximity     -> How close this Node is to target
      float e;        // Explorability -> Distance between this node and its father
      float n;        // Frequency     -> How many times this node has already been casted
      NodeDescriptor node_descriptor; // Index for finding the node in the graph
      std::pair<EdgeDescriptor,bool> edge_descriptor;
      // Maybe use this as a pointer so when the edge is deleted the grid should automatically be updated
      // Note this breaks the "one function-changes-grid " rule
    };

    struct Edge
    {
      float d; // Eucledean distance between the two connecting nodes (for finding shortest path)
    };

    NodeDescriptor root_descriptor;

    // These twwo are used for saving the source and target of an edge given its edge_id
    NodeDescriptor source_vertex;
    NodeDescriptor target_vertex;


    RGraph(){}
   
    /* <AddEdge>
     * Adds a weighted edge between father and child 
     * Returns a std::pair with the edge descriptor and a flag if the edge was added succesfully
     */
    inline std::pair<EdgeDescriptor,bool> AddEdge(const BoostGraph::vertex_descriptor & father, const BoostGraph::vertex_descriptor & child, BoostGraph & G, float weight)    {
      return boost::add_edge(father, child, Edge{weight}, G);
    }
    
    
    
    /*<AddNode>
     *Initialize and insert node to the graph (inline for fast insertion)
     *Parameters
     *  -node: the node to be initialized and added
     *  -G: the graph the node should be added to
     *  -position: Node's position
     *  -distance: Node's distance from its father 
     */
    inline void AddNode(RGraph::Node & node, BoostGraph & G, const Point & node_pos, const Point & target_pos, const float & explorability)
    {
      node.pos.x  = node_pos.x;
      node.pos.y  = node_pos.y;
      node.p   = std::sqrt( std::pow(node_pos.x - target_pos.x,2.) + std::pow(node_pos.y - target_pos.y,2.));
      node.e   = explorability;
      node.n   = 0.;
      
      node.cast_w = (node.e*node.p +node.n + 1.)/( node.p*(node.n + 1.)); // cast_w = f(p,e,n) // Placeholder experiment with the structure of the graph and the times

      node.node_descriptor = boost::add_vertex(node, G); // Add the node to the graph and return the vertex descriptor
      return;
    }
    /*
     * Breaks an edge connecting nodes A and B then inserts C into the graph and creates two new edges connecting A to C and C to B
     * Parameters
     */

    inline void Connect(RGraph::Node & node, BoostGraph & G)
    {
      // boost::remove_edge(vertex_A_descriptor, vertex_B_descriptor, G);
    }

    /*
     * Given a graph g it find if vertex vd is in the graph
    */
    bool IsVertexInGraph(const BoostGraph& g, NodeDescriptor vd) {
    // Iterate over all vertices in the graph
    for (auto iter = boost::vertices(g); iter.first != iter.second; ++iter.first) {
        // If the current vertex matches the given vertex_descriptor, return true
        if (*iter.first == vd)
            return true;
    }
    // If the given vertex_descriptor is not found in the graph, return false
    return false;
}
  };
 }
