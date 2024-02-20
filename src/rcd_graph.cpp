#include "rcd_graph.hpp"


using namespace RCD;
// Initializes graph
RGraph::RGraph()
{

}

void RGraph::AddNode(const Node& node) {
  boost::add_vertex(node, G);
}

void RGraph::AddEdge(const BoostGraph::vertex_descriptor& father, const BoostGraph::vertex_descriptor& child, float weight) {
  boost::add_edge(father, child, Edge{weight}, G);
}


// using namespace RCD;

/*
Adds "node" to RCD::Graph g 
*/
// void RGraph::AddNode(const RGraph::Node & node)
// {
//   boost::add_vertex(RGraph::Node{node.pos, node.cast_w, node.p,node.e,node.o}, RGraph::G);
//   return;
// }


// /*
// Adds an edge that connects father and child. "weight" is the distance between them 
// */
// void RGraph::AddEdge(const G::vertex_descriptor & father, const G::vertex_descriptor & child, float weight)
// {
//   boost::add_edge(father, child, RGraph::Edge{weight}, G);
//   return;
// }

// void RCD::RGraph::AddNode(const RCD::RGraph::Node & node) {
//     auto mike = boost::add_vertex(node, G);
// }

// void RCD::RGraph::AddEdge(const RCD::G::vertex_descriptor &father, const RCD::G::vertex_descriptor &child, float weight) {
//     boost::add_edge(father, child, RGraph::Edge{weight}, G);
// }