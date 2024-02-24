#include "rcd.hpp"
#include <boost/graph/breadth_first_search.hpp>

using namespace RCD;

RCD::RGraph::Node Core::intersectionNode; // Definition of static member variable
RCD::RGraph::EdgeDescriptor Core::intersectionEdge_id; // The edge_id of the intersection



Core::Core(bool robot_flag, MapHandler *map_)
:isRobot(robot_flag), casting_angles(NUM_RAYS), map{map_}
{

  isRobot ? G.AddNode(node2add, G.G, map->robot_pos, map->target_pos, 10.01) : G.AddNode(node2add, G.G, map->target_pos,map->robot_pos, 10.01);
  
  G.root_descriptor = node2add.node_descriptor; // Save the root de

  srand(static_cast<unsigned int>(time(nullptr))); //Seed time for different sequence of pseudo-random numbers
}


/*
NOW: Searches the graph for finding the max weight
TODO:Performs low-variance resampling  and decides which node to cast next
*/
RGraph::Node& Core::CastDecision()
{
    if (boost::num_vertices(G.G) == 0) {
        throw std::runtime_error("Graph is empty");
    }

    // Initialize variables to keep track of the maximum weight and corresponding node
    float maxWeight = std::numeric_limits<float>::lowest();
    size_t maxNodeDescriptor;

    // Iterate over all vertices in the graph
    for (auto vd : boost::make_iterator_range(boost::vertices(G.G)))
    {
        // Access the node
        RGraph::Node& node = G.G[vd];

        // Check if the weight of this node is greater than the current maximum
        if (node.cast_w > maxWeight)
        {
            // Update the maximum weight and corresponding node descriptor
            maxWeight = node.cast_w;
            maxNodeDescriptor = vd;
        }
    }
    G.G[maxNodeDescriptor].node_descriptor = maxNodeDescriptor ;
    // Return the node with the maximum weight
    return G.G[maxNodeDescriptor];
}


/*
  Updated Graph and compute properties. (Gets object ready before cast)
*/
void Core::PrepareCasting()
{
  // Find in which directions RCD is going to cast // BUG: rand() gives the same initial at every run
  casting_dir = (static_cast<float>(rand() % (360/NUM_RAYS + 1)) )*PI/180.;  // Random casting direction
  for (int i = 0 ; i < NUM_RAYS ; ++i){
    casting_angles[i] = casting_dir + i*angle_increment; // Casting angles
  }

  node2cast = CastDecision(); // Extract the node to be casted
  G.G[node2cast.node_descriptor].n += 1;
  G.G[node2cast.node_descriptor].cast_w /= (G.G[node2cast.node_descriptor].n +1); 

}


/*
Casts NUM_RAYS into map->grid
*/
void Core::CastRays()
{
  // std::cout <<"Cast this node: "<< node2cast.pos.x << "  " << node2cast.pos.y << '\n';
  // For every casting direction
  for (const auto& angle : casting_angles)
  {
    cos_cast = std::cos(angle);
    sin_cast = std::sin(angle);
    ray_dis = 5.0; // Starting distance from node
    
    while (!pathFound)
    {
      ray_pos.x = std::ceil(node2cast.pos.x + ray_dis*cos_cast);
      ray_pos.y = std::ceil(node2cast.pos.y + ray_dis*sin_cast);


      // Case #1: Path found
      if ( (!isRobot && map->grid[ray_pos.y][ray_pos.x].robotPass) ||
           (isRobot && map->grid[ray_pos.y][ray_pos.x].targetPass))
      {
        // The intersection node will be added later to both graphs
        isRobot ? G.AddNode(node2add,G.G,ray_pos, map->target_pos, ray_dis) : G.AddNode(node2add,G.G,ray_pos, map->robot_pos, ray_dis) ;
        G.AddEdge(node2cast.node_descriptor,node2add.node_descriptor,G.G,ray_dis);
        addNodeList.push_back(node2add);
        intersectionNode = node2add;
        intersectionEdge_id = map->grid[ray_pos.y][ray_pos.x].edge_id;
        
        pathFound = true;
        return; 
      }
    
      // Case #2: Hit wall after exploring
      if (map->grid[ray_pos.y][ray_pos.x].isOccupied)
      {
        if (ray_dis >10.){ //  Don't add node that immedietly hits the wall
          // Initalize node and fill its properties
          isRobot ? G.AddNode(node2add, G.G, ray_pos, map->target_pos, ray_dis) : G.AddNode(node2add,G.G,ray_pos, map->robot_pos, ray_dis) ;
          // Extract the edge descriptor
          node2add.edge_descriptor = G.AddEdge(node2cast.node_descriptor, node2add.node_descriptor, G.G, ray_dis);
          // Save node to list
          addNodeList.push_back(node2add);  
        }
        break;
      }
   
      // Case #3: Same kind of ray intersection (create new node)
      if (map->grid[ray_pos.y][ray_pos.x].robotPass || map->grid[ray_pos.y][ray_pos.x].targetPass)
      {   
        if (ray_dis >10.){ //  Do I really need this condition? Maybe it sacrifices probabilistic completeness
          isRobot ? G.AddNode(node2add, G.G, ray_pos, map->target_pos, ray_dis) : G.AddNode(node2add, G.G, ray_pos, map->robot_pos, ray_dis) ;

          // Find the source and target of the already implemented edge
          G.source_vertex = boost::source(map->grid[ray_pos.y][ray_pos.x].edge_id, G.G);
          G.target_vertex = boost::target(map->grid[ray_pos.y][ray_pos.x].edge_id, G.G);

          node2add.edge_descriptor = G.AddEdge(node2cast.node_descriptor, node2add.node_descriptor, G.G, ray_dis);
          // TODO: Ideally I should break the edge and update the grid with the new two edges
          // Find distance between new node and source/target of the already existing edge
          // A----> B ----> C , where B is node2add, and A & C are the already existing nodes connected by edge_id
          auto disAB = std::sqrt(std::pow(G.G[G.source_vertex].pos.x - node2add.pos.x,2) + std::pow(G.G[G.source_vertex].pos.y - node2add.pos.y,2));
          auto disBC = std::sqrt(std::pow(G.G[G.target_vertex].pos.x - node2add.pos.x,2) + std::pow(G.G[G.target_vertex].pos.y - node2add.pos.y,2)); 
          // Create two extra connections (like breaking the edge in two) 
          G.AddEdge(node2add.node_descriptor, G.source_vertex,G.G,disAB);
          G.AddEdge(node2add.node_descriptor, G.source_vertex,G.G,disBC);
          
          addNodeList.push_back(node2add); 
        }
        break;     
      }
      ray_dis += 1.0;
    }      
  }
  return;
}


/*
 Bresenham's line algorithm: Given two points finds where the line has passed through in 2D discrete grids
*/
void Core::UpdateGrid()
{
  // TODO: Optimize this
  // fill the grid with line path (max lines = NUM_RAYS)
  for (const auto& node: addNodeList) 
  {
    std::vector<Point> points;
    int x1 = node2cast.pos.x;
    int y1 = node2cast.pos.y;
    int x2 = node.pos.x;
    int y2 = node.pos.y;
    
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    
    int sx = x1 < x2 ? 1 : -1;
    int sy = y1 < y2 ? 1 : -1;
    
    int err = dx - dy;
    int e2;
    
    while (true) {
        points.push_back(Point(x1, y1));
        
        if (x1 == x2 && y1 == y2) break;
        
        e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    if (isRobot){
      for (const auto& point : points) {
        map->grid[point.y][point.x].robotPass = true; 
        map->grid[point.y][point.x].edge_id = node.edge_descriptor.first; 
      }
    }else{
      for (const auto& point : points) {
        map->grid[point.y][point.x].targetPass = true; 
        map->grid[point.y][point.x].edge_id = node.edge_descriptor.first; 

      }
    }
  }
 
  addNodeList.clear(); // To maintain the dynamic size (max_size = NUM_RAYS)
}

/* 
 * Adds the intersection node to both graphs and finds the shortest path on the graph with breadth_first search
 */
std::vector<Point> Core::ShortestPath()
{

  // Adds the intersection_node to the other graph (not the one that found the path)
  if (!G.IsVertexInGraph(G.G,intersectionNode.node_descriptor)){
    std::cout << " Is robot? -->" << isRobot << '\n';

    G.source_vertex = boost::source(intersectionEdge_id, G.G);
    G.target_vertex = boost::target(intersectionEdge_id, G.G);
    // G.AddNode(node2add, G.G, ray_pos, map->target_pos, ray_dis

    // A----> B ----> C , where B is node2add, and A & C are the already existing nodes connected by edge_id
    auto disAB = std::sqrt(std::pow(G.G[G.source_vertex].pos.x - intersectionNode.pos.x,2) + std::pow(G.G[G.source_vertex].pos.y - intersectionNode.pos.y,2));
    auto disBC = std::sqrt(std::pow(G.G[G.target_vertex].pos.x - intersectionNode.pos.x,2) + std::pow(G.G[G.target_vertex].pos.y - intersectionNode.pos.y,2)); 
    
    if (disAB > disBC){
      G.AddNode(intersectionNode, G.G, intersectionNode.pos, map->target_pos, disBC);
    }else{
      G.AddNode(intersectionNode, G.G, intersectionNode.pos, map->target_pos, disAB);
    }

    G.AddEdge(intersectionNode.node_descriptor, G.source_vertex,G.G,disAB);
    G.AddEdge(intersectionNode.node_descriptor, G.source_vertex,G.G,disBC);
  }


  std::vector<RGraph::NodeDescriptor> predecessors(boost::num_vertices(G.G));
  std::vector<Point> path;
  // Run breadth-first search algorithm
  breadth_first_search(G.G, G.root_descriptor,boost::visitor(boost::make_bfs_visitor(boost::record_predecessors(predecessors.data(), boost::on_tree_edge()))));

  // Reconstruct the shortest path
  std::vector<RGraph::NodeDescriptor> shortest_path;
  for (RGraph::NodeDescriptor v = intersectionNode.node_descriptor; v != G.root_descriptor; v = predecessors[v]) {
      shortest_path.push_back(v);
      path.push_back(G.G[v].pos);
  }

  shortest_path.push_back(G.root_descriptor);
  path.push_back(G.G[G.root_descriptor].pos);

  return path;
}
