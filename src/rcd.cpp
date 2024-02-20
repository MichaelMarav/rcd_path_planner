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
    node2add.o   = 0;
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
    node2add.o   = 0;
    // Total weight
    node2add.cast_w   = 0.; // To do compute the weight with a function f(p,e,o)
  }
  G.AddNode(node2add);
}


/*
Performs low-variance resampling  and decides which node to cast next
*/
RGraph::Node Core::CastDecision()
{

  // TODO: add low variance resampling based on the weight of each node
  
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


  node2cast = node2add;
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
    cos = std::cos(angle);
    sin = std::sin(angle);
    
    ray_dis = 2.0;
    while (!pathFound)
    {
      beam.x = std::round(node2cast.pos.x + ray_dis*cos);
      beam.y = std::round(node2cast.pos.y + ray_dis*sin);
      std::cout << map->grid[beam.x][beam.y].isOccupied<< '\n';


      if (map->grid[beam.x][beam.y].isOccupied) break;
      if (isRobot && map->grid[beam.x][beam.y].robotPass) break;
      // if (!isRobot && )
      ray_dis += 1.0;
    }      
  
  }
}
