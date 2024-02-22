#include "rcd.hpp"
#include <typeinfo>

using namespace RCD;


Core::Core(bool robot_flag, MapHandler *map_):isRobot(robot_flag), casting_angles(N_rays)
{
  map = map_;
  std::cout << "Robot --> " << map->robot_pos.x << "  " <<map->robot_pos.y << '\n';
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
    node2add.cast_w   = 3. ; // To do compute the weight with a function f(p,e,o)
  
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
    node2add.cast_w   = 3.; // To do compute the weight with a function f(p,e,o)
  }

  G.AddNode(node2add,G.G); // Add root to the graph
  father = node2add.descriptor;
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
void Core::Update()
{
  // Find in which directions RCD is going to cast // BUG: rand() gives the same initial at every run
  casting_dir = (static_cast<float>(rand() % (360/N_rays + 1)) )*PI/180.;  // Random casting direction
  for (int i = 0 ; i < N_rays ; ++i){
    casting_angles[i] = casting_dir + i*angle_increment; // Casting angles
  }

  node2cast = CastDecision();
  father = node2cast.descriptor;
}



/*
Casts N_Rays into map_->grid
-->!!! This is the only method that changes the values of the grid!!!<--
*/
void Core::CastRays()
{
  std::cout <<"Cast this node: "<< node2cast.pos.x << "  " << node2cast.pos.y << '\n';
  
  // For every casting direction
  for (const auto& angle : casting_angles)
  {
    cos_cast = std::cos(angle);
    sin_cast = std::sin(angle);
    ray_dis = 10.0;

    while (!pathFound)
    {
      ray_pos.x = std::ceil(node2cast.pos.x + ray_dis*cos_cast);
      ray_pos.y = std::ceil(node2cast.pos.y + ray_dis*sin_cast);

      // Don't do that here (store it in a vector and the nodes in update)

      if (map->grid[ray_pos.y][ray_pos.x].isOccupied)
      {
        if (ray_dis > 5.0){

          node2add.pos.x  = ray_pos.x;
          node2add.pos.y  = ray_pos.y;

          // Proximity
          node2add.p   = 0.;
          // Explorability (distance from parent)
          node2add.e   = 0.;
          // Occurence 
          node2add.o   = 0;
          // Total weight
          node2add.cast_w   = ray_dis; // To do compute the weight with a function f(p,e,o)

          G.AddNode(node2add,G.G);
          child = node2add.descriptor;
          G.AddEdge(father,child,G.G,ray_dis);
        
          break;
        }else{ // Don't add node
          break;
        }
      }
   

      if (isRobot && map->grid[ray_pos.y][ray_pos.x].robotPass)
      {        
        if (ray_dis > 5.0){
          node2add.pos.x  = ray_pos.x;
          node2add.pos.y  = ray_pos.y;

          // Proximity
          node2add.p   = 0.;
          // Explorability (distance from parent)
          node2add.e   = 0.;
          // Occurence 
          node2add.o   = 0;
          // Total weight
          node2add.cast_w  = ray_dis; // To do compute the weight with a function f(p,e,o)
    
          G.AddNode(node2add,G.G);
          child = node2add.descriptor;
          G.AddEdge(father,child,G.G,ray_dis);
          break;
          //TODO: break the edge
        }else{
          break;
        }
      
      }


      // if (!isRobot && map->grid[ray_pos.y][ray_pos.x].targetPass)
      // {
      //   if (ray_dis > 5.0){
      //     node2add.pos.x  = ray_pos.x;
      //     node2add.pos.y  = ray_pos.y;

      //     // Proximity
      //     node2add.p   = 0.;
      //     // Explorability (distance from parent)
      //     node2add.e   = 0.;
      //     // Occurence 
      //     node2add.o   = 0;
      //     // Total weight
      //     node2add.cast_w   = ray_dis; // To do compute the weight with a function f(p,e,o)
      //     G.AddNode(node2add,G.G);
      //     child = node2add.descriptor;
      //     G.AddEdge(father,child,G.G,ray_dis);
      //     break;
      //     //TODO: break the edge
      //   }else{
      //     break;
      //   }
      // }

      // if (isRobot && map->grid[ray_pos.y][ray_pos.x].targetPass)
      // {
      //   node2add.pos.x  = ray_pos.x;
      //   node2add.pos.y  = ray_pos.y;

      //   // Proximity
      //   node2add.p   = 0.;
      //   // Explorability (distance from parent)
      //   node2add.e   = 0.;
      //   // Occurence 
      //   node2add.o   = 0;
      //   // Total weight
      //   node2add.cast_w   = ray_dis; // To do compute the weight with a function f(p,e,o)
  
      //   G.AddNode(node2add,G.G);
      //   child = node2add.descriptor;
      //   G.AddEdge(father,child,G.G,ray_dis);
      //   pathFound = true;

      //   return;

      //   //TODO: break the edge
      // }


      // if (!isRobot && map->grid[ray_pos.y][ray_pos.x].robotPass)
      // {

      //   node2add.pos.x  = ray_pos.x;
      //   node2add.pos.y  = ray_pos.y;

      //   // Proximity
      //   node2add.p   = 0.;
      //   // Explorability (distance from parent)
      //   node2add.e   = 0.;
      //   // Occurence 
      //   node2add.o   = 0;
      //   // Total weight
      //   node2add.cast_w   = ray_dis; // To do compute the weight with a function f(p,e,o)
  
      //   G.AddNode(node2add,G.G);
      //   child = node2add.descriptor;
      //   G.AddEdge(father,child,G.G,ray_dis);
      //   pathFound = true;
      //   return;
        
      //   //TODO: break the edge
      // }
      
      if (isRobot){
        map->grid[ray_pos.y][ray_pos.x].robotPass = true;
      }else{
        map->grid[ray_pos.y][ray_pos.x].targetPass = true;
      }

      ray_dis += 1.0;
    }      
  
  }
}
