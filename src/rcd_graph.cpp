#include "rcd_graph.hpp"

using namespace RCD;
// Initializes graph
RGraph::RGraph()
{

}

RGraph::BoostGraph::vertex_descriptor RGraph::AddNode(const Node& node)
{
  return boost::add_vertex(node, G); // Add the node to the graph and return the vertex descriptor
}

void RGraph::AddEdge(const BoostGraph::vertex_descriptor& father, const BoostGraph::vertex_descriptor& child, float weight)
{
  boost::add_edge(father, child, Edge{weight}, G);
}

