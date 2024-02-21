#include "rcd.hpp"

using namespace RCD;


Core::Core(bool robot_flag, MapHandler *map_):isRobot(robot_flag), casting_angles(N_rays)
{
  this->map = map_;
  if (isRobot){
    std::cout << "Initialized Robot Graph \n";

    node2add.pos = map_->robot_pos;
    // Proximity
    node2add.p   = std::sqrt(std::pow(map_->robot_pos.x - map_->target_pos.x,2) + std::pow(map_->robot_pos.y - map_->target_pos.y,2) );
    // Explorability (distance from parent)
    node2add.e   = 0.;
    // Occurence 
    node2add.o   = 1;
    // Total weight
    node2add.cast_w   = 0.; // To do compute the weight with a function f(p,e,o)
  }else{
    std::cout << "Initialized Target Graph \n";

    node2add.pos = map_->target_pos;
    // Proximity
    node2add.p   = std::sqrt(std::pow(map_->robot_pos.x - map_->target_pos.x,2) + std::pow(map_->robot_pos.y -map_->target_pos.y,2) );
    // Explorability (distance from parent)
    node2add.e   = 0.;
    // Occurence 
    node2add.o   = 1;
    // Total weight
    node2add.cast_w   = node2add.o; // To do compute the weight with a function f(p,e,o)
  }
  father = G.AddNode(node2add); // Add root to the graph
}


/*
Performs low-variance resampling  and decides which node to cast next
*/
RGraph::Node Core::CastDecision()
{

  // TODO: add low variance resampling based on the weight of each node
  // Placeholder: for testing purposes, the next node to cast will be the one with more explorability
  RGraph::Node* maxWeightNode = nullptr;
  float maxWeight = std::numeric_limits<float>::lowest(); // Initialize to lowest possible value

  // Iterate through all nodes in the graph
  auto vertices = boost::vertices(G.G);
  for (auto it = vertices.first; it != vertices.second; ++it)
  {
    RGraph::Node& node = G.G[*it];
    if (node.cast_w > maxWeight)
    {
      maxWeight = node.cast_w;
      maxWeightNode = &node;
      father = *it; // The father will be the node to be casted
    }
  }

  return *maxWeightNode;
} 

/*
  Updated Graph and compute properties. (Gets object ready before cast)
*/
void Core::Update()
{
  // Find in which directions RCD is going to cast
  casting_dir = (static_cast<float>(rand() % (360/N_rays + 1)) )*PI/180.;  // Random casting direction
  for (int i = 0 ; i < N_rays ; ++i){
    casting_angles[i] = casting_dir + i*angle_increment; // Casting angles
  }

  node2cast = CastDecision();
}



/*
Casts N_Rays into map_->grid
-->!!! This is the only method that changes the values of the grid!!!<--
*/
void Core::CastRays()
{
  // For every casting direction
  for (const auto& angle : casting_angles)
  {
    cos_cast = std::cos(angle);
    sin_cast = std::sin(angle);
    
    ray_dis = 2.0;
    while (!pathFound)
    {
      beam.x = std::round(node2cast.pos.x + ray_dis*cos_cast);
      beam.y = std::round(node2cast.pos.y + ray_dis*sin_cast);




      // Don't do that here (store it in a vector and the nodes in update)
      if (map->grid[beam.x][beam.y].isOccupied && ray_dis > 3)// hit wall and it is not next to the node
      { 
        // Proximity
        node2add.p   = 0.;
        // Explorability (distance from parent)
        node2add.e   = 0.;
        // Occurence 
        node2add.o   = 0;
        // Total weight
        node2add.cast_w   = ray_dis; // To do compute the weight with a function f(p,e,o)
  
        child = G.AddNode(node2add);
        G.AddEdge(father,child,ray_dis);
        break;
      }

      if (isRobot && map->grid[beam.x][beam.y].robotPass)
      {
        // Proximity
        node2add.p   = 0.;
        // Explorability (distance from parent)
        node2add.e   = 0.;
        // Occurence 
        node2add.o   = 0;
        // Total weight
        node2add.cast_w   = ray_dis; // To do compute the weight with a function f(p,e,o)
  
        child = G.AddNode(node2add);
        G.AddEdge(father,child,ray_dis);
        break;
        //TODO: break the edge
      }


      if (!isRobot && map->grid[beam.x][beam.y].targetPass)
      {
        // Proximity
        node2add.p   = 0.;
        // Explorability (distance from parent)
        node2add.e   = 0.;
        // Occurence 
        node2add.o   = 0;
        // Total weight
        node2add.cast_w   = ray_dis; // To do compute the weight with a function f(p,e,o)
  
        child = G.AddNode(node2add);
        G.AddEdge(father,child,ray_dis);
        break;
        //TODO: break the edge
      }

      if (isRobot && map->grid[beam.x][beam.y].targetPass)
      {
        // Proximity
        node2add.p   = 0.;
        // Explorability (distance from parent)
        node2add.e   = 0.;
        // Occurence 
        node2add.o   = 0;
        // Total weight
        node2add.cast_w   = ray_dis; // To do compute the weight with a function f(p,e,o)
  
        child = G.AddNode(node2add);
        G.AddEdge(father,child,ray_dis);
        pathFound = true;

        break;

        //TODO: break the edge
      }


      if (!isRobot && map->grid[beam.x][beam.y].robotPass)
      {
        // Proximity
        node2add.p   = 0.;
        // Explorability (distance from parent)
        node2add.e   = 0.;
        // Occurence 
        node2add.o   = 0;
        // Total weight
        node2add.cast_w   = ray_dis; // To do compute the weight with a function f(p,e,o)
  
        child = G.AddNode(node2add);
        G.AddEdge(father,child,ray_dis);
        pathFound = true;
        break;
        
        //TODO: break the edge
      }

      // if (!isRobot && )
      ray_dis += 1.0;
    }      
  
  }
}
