#include "rcd.hpp"

using namespace RCD;


// Initialize static variables
RGraph::Node Core::intersectionNode; 
RGraph::EdgeDescriptor Core::intersectionEdge_id;
bool Core::pathFound = false;  
bool Core::pathFoundByRobot = false;

/**
 * @brief Construct a new Core:: Core object. 
 * 1. Initializes the two graphs
 * 2. Initilizes the weights of the root nodes. 
 * 3. Initilizes random seed for primary casting direction
 * 4. Super imposes a rectangle on the occupancy grid to constraint search area
 * @param robot_flag true if the robot caster is constructed, false if the target caster
 * @param map_ pointer to the occupancy grid data structure
 */
Core::Core(bool robot_flag, MapHandler *map_)
:isRobot(robot_flag), casting_angles(NUM_RAYS), map{map_}
{

  // Initialize robot and target root weights
  if (isRobot){
    node2add.pos.x  = map->robot_pos.x;
    node2add.pos.y  = map->robot_pos.y;
    node2add.p   = CalculateDistance(node2add.pos,map->target_pos);
    node2add.e   = 0;
    node2add.n   = 0.;
    node2add.cast_w = node2add.p;
  }else{
    node2add.pos.x  = map->target_pos.x;
    node2add.pos.y  = map->target_pos.y;
    node2add.p   = CalculateDistance(node2add.pos,map->robot_pos);
    node2add.e   = 0;
    node2add.n   = 0.;
    node2add.cast_w = node2add.p;  
  }

  G.AddNode(node2add, G.G); // Besides adding the node to the graph it also fills the node_descriptor
  weight_priority_queue.push(node2add);

  srand(static_cast<unsigned int>(time(nullptr))); //Seed time for different sequence of pseudo-random numbers  
  
  ConstraintSearchArea(4.0); // Add rectangle to limit search area
}


/**
 * @brief Imposes a rectangle into the grid in order to constraint the search area (adds an occupied rectangle that includes the pos of robot and target)
 * 
 * @param scale_rectangle A float used for scaling the size of the rectangle
 */
void Core::ConstraintSearchArea(float scale_rectangle)
{
  fPoint B(map->target_pos.x,map->target_pos.y);
  fPoint A(map->robot_pos.x,map->robot_pos.y);


  float w = scale_rectangle*(CalculateDistance(map->target_pos,map->robot_pos)  + width_bias); // Width of the constraint-search rectangle
  float h = scale_rectangle*50.;
  fPoint AB = B - A;

  // Normalize the direction vector of AB
  auto norm = std::sqrt(AB.x * AB.x + AB.y * AB.y);

  fPoint AB_normalized = fPoint(AB.x/norm,AB.y/norm);

  // Calculate the perpendicular vector to AB
  fPoint perp_vector(AB_normalized.y, -AB_normalized.x);

  // Calculate the coordinates of the mid-fPoint of AB
  fPoint mid_fPoint = A + AB*0.5;

  // Calculate the coordinates of the vertices of the rectangle
  iPoint C = {static_cast<int>(std::round(mid_fPoint.x + perp_vector.x * h + AB_normalized.x * (w / 2)) ),static_cast<int>(std::round(mid_fPoint.y + perp_vector.y * h + AB_normalized.y * (w / 2))) };
  iPoint D = {static_cast<int>(std::round(mid_fPoint.x + perp_vector.x * h - AB_normalized.x * (w / 2)) ),static_cast<int>(std::round(mid_fPoint.y + perp_vector.y * h - AB_normalized.y * (w / 2))) };
  iPoint E = {static_cast<int>(std::round(mid_fPoint.x - perp_vector.x * h - AB_normalized.x * (w / 2)) ),static_cast<int>(std::round(mid_fPoint.y - perp_vector.y * h - AB_normalized.y * (w / 2))) };
  iPoint F = {static_cast<int>(std::round(mid_fPoint.x - perp_vector.x * h + AB_normalized.x * (w / 2)) ),static_cast<int>(std::round(mid_fPoint.y - perp_vector.y * h + AB_normalized.y * (w / 2))) };

  // Extract the points of each side of the rectangle
  ImposeRectangle(BresenhamLine(C,D));
  ImposeRectangle(BresenhamLine(D,E));
  ImposeRectangle(BresenhamLine(E,F));
  ImposeRectangle(BresenhamLine(F,C));      
}
/**
 * @brief Given a line with points, it makes the map occupied while also inflating the line
 * 
 * @param line a vector of points that define a line
 */
void Core::ImposeRectangle(const std::vector<iPoint> & line)
{
  for (const auto& point : line) {
    if (point.y < map->height && point.x < map->width){
      for (int i = -ray_start_dis; i <= ray_start_dis; ++i) {
        for (int j = -ray_start_dis; j <= ray_start_dis; ++j) {
          int new_y = point.y + i;
          int new_x = point.x + j;
          if (new_y >= 0 && new_y < map->height && new_x >= 0 && new_x < map->width)
          {
              map->grid[new_y][new_x].isOccupied  = true; // For stopping casting besides the rectangle
              map->grid[new_y][new_x].onRectangle = true; // For visualization purposes
          }
        }
      }
    }
  }
}

/* <PrepareCast> 
 * Updated Graph and compute properties. (Gets object ready before cast)
 */
void Core::PrepareCast()
{
  // Find in which directions RCD is going to cast 
  casting_dir =  (static_cast<float>(rand() % (360/NUM_RAYS + 1)) )*PI/180.;  // Random casting direction
  for (int i = 0 ; i < NUM_RAYS ; ++i){
    casting_angles[i] = casting_dir + i*angle_increment; // Casting angles
  }
 
  node2cast = weight_priority_queue.top();// Extract the node to be casted
  weight_priority_queue.pop();
  // Update weight
  G.G[node2cast.node_descriptor].n += 1;
  G.G[node2cast.node_descriptor].cast_w /= G.G[node2cast.node_descriptor].n;//log10(G.G[node2cast.node_descriptor].e+1.)/(G.G[node2cast.node_descriptor].p*exp((G.G[node2cast.node_descriptor].n+1.)));
  weight_priority_queue.push(G.G[node2cast.node_descriptor]);
}



/* < CastRays > 
 * Casts NUM_RAYS into map->grid
 */
void Core::CastRays()
{
  if (RCD::Core::pathFound){
    return;
  }

  PrepareCast();


  // For every casting direction
  for (const auto& angle : casting_angles)
  {
    cos_cast = std::cos(angle);
    sin_cast = std::sin(angle);
    ray_dis = ray_start_dis; // Starting distance from node
    
    while (!pathFound)
    {
      ray_pos.x = std::ceil(node2cast.pos.x + ray_dis*cos_cast);
      ray_pos.y = std::ceil(node2cast.pos.y + ray_dis*sin_cast);

      // Case #1: Path found
      if ( (!isRobot && map->grid[ray_pos.y][ray_pos.x].robotPass) ||
           (isRobot  && map->grid[ray_pos.y][ray_pos.x].targetPass))
      {
        pathFound = true;

        // The intersection node will be added later to both graphs

        if (isRobot){
          G.UpdateWeight(node2add,ray_dis,node2cast.p, ray_pos, map->target_pos, map->robot_pos);
          pathFoundByRobot = true;
        }else{
          G.UpdateWeight(node2add,ray_dis,node2cast.p,ray_pos, map->robot_pos, map->target_pos);
          pathFoundByRobot = false;
        }

        G.AddNode(node2add, G.G);

        node2add.edge_descriptor = G.AddEdge(node2cast.node_descriptor, node2add.node_descriptor, G.G, ray_dis);
        // UpdateGrid(node2cast,node2add);

        intersectionNode = node2add; // This node corresponds to the current graph
        intersectionEdge_id = map->grid[ray_pos.y][ray_pos.x].edge_id; // This edge ID corresponds to the other graph
        

        return; // Return to stop further casting
      }


      // Case #2: Hit wall after exploring
      if (map->grid[ray_pos.y][ray_pos.x].isOccupied)
      {
        if (ray_dis < ray_start_dis + 1.)  //  Don't add node that immedietly hits the wall
        {
          break;
        }
        // Initalize node and fill its properties          
        
        isRobot ? G.UpdateWeight(node2add,ray_dis,node2cast.p, ray_pos, map->target_pos, map->robot_pos) : G.UpdateWeight(node2add,ray_dis,node2cast.p, ray_pos, map->robot_pos, map->target_pos);  
        G.AddNode(node2add, G.G);

        node2add.edge_descriptor = G.AddEdge(node2cast.node_descriptor, node2add.node_descriptor, G.G, ray_dis);

        weight_priority_queue.push(node2add);

        // Save node to list
        // addNodeList.push_back(node2add);  
        UpdateGrid(node2cast,node2add);

      
        break;
      }
   
      // Case #3: Same kind of ray intersection (create new node)
      std::pair<bool,iPoint>  intersection = CheckIntersection(ray_pos);

      if (intersection.first) // If there is an intersection
      {   

        if (ray_dis < ray_start_dis + 10.)  //  Don't add node that immedietly hits the wall
        {
          break;
        }
        ray_pos = intersection.second;

        isRobot ? G.UpdateWeight(node2add,ray_dis,node2cast.p , ray_pos, map->target_pos, map->robot_pos) : G.UpdateWeight(node2add,ray_dis, node2cast.p,ray_pos, map->robot_pos, map->target_pos);  

        G.AddNode(node2add, G.G);

        node2add.edge_descriptor = G.AddEdge(node2cast.node_descriptor, node2add.node_descriptor, G.G, ray_dis);

        weight_priority_queue.push(node2add);

        // Find the source and target of the already implemented edge
        G.source_vertex = boost::source(map->grid[ray_pos.y][ray_pos.x].edge_id, G.G);
        G.target_vertex = boost::target(map->grid[ray_pos.y][ray_pos.x].edge_id, G.G);

        // // Remove the edge between source and target
        // boost::remove_edge(G.source_vertex, G.target_vertex, G.G);

        // Add edge that connect node2Cast with node2add
        // Find distance between new node and source/target of the already existing edge
        // A----> B ----> C , where B is node2add, and A & C are the already existing nodes connected by edge_id
        auto disAB = CalculateDistance(G.G[G.source_vertex].pos, node2add.pos);
        auto disBC = CalculateDistance(G.G[G.target_vertex].pos, node2add.pos);
        
        // // G.G[G.target_vertex].e = disBC;
        // // G.G[G.target_vertex].cast_w =  (node.e*node.p +node.n + 1.)/( node.p*(node.n + 1.));
        auto new_node1 = G.G[G.source_vertex];
        auto new_node2 = G.G[G.target_vertex];

        // // Create two extra connections (breaking the edge in two segments) 
        new_node1.edge_descriptor = G.AddEdge(node2add.node_descriptor, G.source_vertex,G.G,disAB);
        new_node2.edge_descriptor = G.AddEdge(node2add.node_descriptor, G.target_vertex,G.G,disBC);
        
        isRobot ? G.UpdateWeight(G.G[G.target_vertex], disBC, node2add.p, G.G[G.target_vertex].pos, map-> target_pos, map->robot_pos) : G.UpdateWeight(G.G[G.target_vertex], disBC, node2add.p, G.G[G.target_vertex].pos, map->robot_pos, map-> target_pos);
        UpdateGrid(node2cast,node2add);
        UpdateGrid(node2add,new_node1);
        UpdateGrid(node2add,new_node2);

        // addNodeList.push_back(node2add); 
        
        break;     
      }

      ray_dis += 1.0;
    }      
  }
  return;
}

/**
 * @brief Performs the Bresenham's line algorithm and extract the points that define a line from A to B
 * 
 * @param A point in the occupancy grid (x,y)
 * @param B point in the occupancy grid (x,y)
 * @return std::vector<Point> the points that are on the line from A to B in grid resolution
 */

std::vector<iPoint> Core::BresenhamLine(const iPoint & A, const iPoint & B) 
{
  std::vector<iPoint> points;
  int x1 = A.x;
  int y1 = A.y;
  int x2 = B.x;
  int y2 = B.y;
  
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  
  int sx = x1 < x2 ? 1 : -1;
  int sy = y1 < y2 ? 1 : -1;
  
  int err = dx - dy;
  int e2;
  
  while (true) 
  {
    points.push_back(iPoint(x1, y1));
    
    if (x1 == x2 && y1 == y2) break;
    
    e2 = 2 * err;
    if (e2 > -dy)
    {
        err -= dy;
        x1 += sx;
    }
    if (e2 < dx) 
    {
        err += dx;
        y1 += sy;
    }
  }
  return points;
}


/**
 * @brief Uses the DefineLine to find the points that connect the source and target nodes and updates the occupancy grid based on which ray has passed
 *  Updates the *map global pointer
 * @param source The source node
 * @param target The target node
 */
void Core::UpdateGrid(const RCD::RGraph::Node&  source,const  RCD::RGraph::Node & target)
{
  std::vector<iPoint> points = BresenhamLine(source.pos, target.pos);

  if (isRobot){
    for (const auto& point : points) {
      map->grid[point.y][point.x].robotPass = true; 
      map->grid[point.y][point.x].edge_id = target.edge_descriptor; 
    }
  }else{
    for (const auto& point : points) {
      map->grid[point.y][point.x].targetPass = true; 
      map->grid[point.y][point.x].edge_id = target.edge_descriptor; 
    }
  }
}



/* <ShortestPath>
 * Adds the intersection node to both graphs and finds the shortest path on the graph with breadth_first search
 */
std::vector<iPoint> Core::ShortestPath(RCD::RGraph::Node end_node)
{
    // Define a priority queue to store vertices based on their distance from the end node
    std::priority_queue<std::pair<float, RCD::RGraph::NodeDescriptor>,
                        std::vector<std::pair<float, RCD::RGraph::NodeDescriptor>>,
                        std::greater<std::pair<float, RCD::RGraph::NodeDescriptor>>> pq;

    // Initialize distances to all vertices as infinity
    std::vector<float> dist(boost::num_vertices(G.G), std::numeric_limits<float>::infinity());

    // Initialize the distance of the end node as 0
    dist[end_node.node_descriptor] = 0;

    // Initialize the predecessor of all vertices as -1
    std::vector<RCD::RGraph::NodeDescriptor> predecessor(boost::num_vertices(G.G), - 1);

    // Push the end node into the priority queue
    pq.push(std::make_pair(0, end_node.node_descriptor));

    // Dijkstra's algorithm
    while (!pq.empty()) {
        // Extract the vertex with the smallest distance from the priority queue
        auto u = pq.top().second;
        pq.pop();

        // Iterate over all neighboring vertices of u
        for (auto&& edge : boost::make_iterator_range(boost::in_edges(u, G.G))) {
            auto v = boost::source(edge, G.G);
            auto weight = G.G[edge].d;  // Get the weight of the edge

            // Relaxation step
            if (dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                predecessor[v] = u;
                pq.push(std::make_pair(dist[v], v));
            }
        }
    }

    // Reconstruct the shortest path
    std::vector<iPoint> path;
    std::vector<RGraph::NodeDescriptor> path_ver;

    RCD::RGraph::NodeDescriptor current = 0; // Start from the root of the graph
    while (current != end_node.node_descriptor) {  // Stop when reaching the end node
        path_ver.push_back(current);
        path.push_back(G.G[current].pos);
        current = predecessor[current];
        if (current == -1) { // If the root node is unreachable from the end node
            path.clear();
            return path;
        }
    }

    // Add the end node's position to the path
    path.push_back(end_node.pos);
    path_ver.push_back(end_node.node_descriptor);
    
    return path;
}



/* <AddIntersectionNode>
 * otherGraph: Is the graph that found the intersection (either robot or target)
 * Returns the last node that was added to the graph
 */
RCD::RGraph::Node Core::AddIntersectionNode()
{
  auto tmp_node = intersectionNode;
  G.UpdateWeight(tmp_node,10,10, intersectionNode.pos, map->robot_pos, map->target_pos);
  G.AddNode(tmp_node, G.G); // Add node to the graph
  
  // Find the source and target of the already implemented edge
  G.source_vertex = boost::source(intersectionEdge_id, G.G);
  G.target_vertex = boost::target(intersectionEdge_id, G.G);

  auto disAB = std::sqrt(std::pow(G.G[G.source_vertex].pos.x - tmp_node.pos.x,2) + std::pow(G.G[G.source_vertex].pos.y - tmp_node.pos.y,2));
  auto disBC = std::sqrt(std::pow(G.G[G.target_vertex].pos.x - tmp_node.pos.x,2) + std::pow(G.G[G.target_vertex].pos.y - tmp_node.pos.y,2));



  G.AddEdge(tmp_node.node_descriptor, G.source_vertex,G.G,disAB);
  G.AddEdge(tmp_node.node_descriptor, G.target_vertex,G.G,disBC);
  
  return tmp_node;
}


/*<CheckIntersection>
 * Checks if there is an intersection around the ray. if it is, returns a flag and the edge descriptor 
 */
std::pair<bool,iPoint>  Core::CheckIntersection(const iPoint & p)
{
    for (int i = -1 ; i < 2 ; ++i){
      for (int j = -1 ; j < 2; ++j){
        if ( (isRobot && map->grid[p.y + i][p.x +j].robotPass) || (!isRobot && map->grid[p.y + i][p.x +j].targetPass)){
          return std::make_pair(true, iPoint(p.x +j, p.y +i));
        }
      }
    }
    return std::make_pair(false, iPoint(0,0));
}