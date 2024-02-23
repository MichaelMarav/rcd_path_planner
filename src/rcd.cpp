#include "rcd.hpp"

using namespace RCD;


Core::Core(bool robot_flag, MapHandler *map_)
:isRobot(robot_flag), casting_angles(NUM_RAYS), map{map_}
{
  if (isRobot){
    printInfo("Initialized Robot Graph");
    G.AddNode(node2add,G.G,map->robot_pos,1.0); // Initial Weight = 1
  }else{
    printInfo("Initialized Target Graph");
    G.AddNode(node2add,G.G,map->target_pos,1.0);
  }
}


/*
Performs low-variance resampling  and decides which node to cast next
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
    G.G[maxNodeDescriptor].descriptor = maxNodeDescriptor ;
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
  G.G[node2cast.descriptor].o += 1;
  G.G[node2cast.descriptor].cast_w /= (G.G[node2cast.descriptor].o +1);

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


      // Case #1
      if (map->grid[ray_pos.y][ray_pos.x].isOccupied)
      {
        if (ray_dis >10.){ //  Don't add node that immedietly hits the wall
          G.AddNode(node2add,G.G,ray_pos, ray_dis);
          G.AddEdge(node2cast.descriptor,node2add.descriptor,G.G,ray_dis);
          addNodeList.push_back(node2add); // 
        }
        break;
      }
   
      // Case #2
      if (isRobot && map->grid[ray_pos.y][ray_pos.x].robotPass)
      {        
          G.AddNode(node2add,G.G,ray_pos, ray_dis);
          G.AddEdge(node2cast.descriptor,node2add.descriptor,G.G,ray_dis);
          addNodeList.push_back(node2add); // 
          break;     
      }

      // Case #3
      if (!isRobot && map->grid[ray_pos.y][ray_pos.x].targetPass)
      {
        
          G.AddNode(node2add,G.G,ray_pos, ray_dis);
          G.AddEdge(node2cast.descriptor,node2add.descriptor,G.G,ray_dis);
          addNodeList.push_back(node2add); // 
          break;
      }

      // Case #4
      if ( (!isRobot && map->grid[ray_pos.y][ray_pos.x].robotPass) ||
           (isRobot && map->grid[ray_pos.y][ray_pos.x].targetPass))
      {
        // intersectionNode = node2add;
        // G.AddNode(intersectionNode,G.G, ray_pos, ray_dis);
        // G.AddEdge(node2cast.descriptor,node2add.descriptor,G.G,ray_dis);
        // addNodeList.push_back(node2add);
        pathFound = true;
        return; // THis is not plotted
        //TODO: break the edge
      }
    
      ray_dis += 1.0;
    }      
  }
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
      }
    }else{
      for (const auto& point : points) {
        map->grid[point.y][point.x].targetPass = true; 
      }
    }
  }
 
  addNodeList.clear(); // To maintain the dynamic size (max_size = NUM_RAYS)
}