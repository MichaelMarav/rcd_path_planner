#include "rcd.hpp"

using namespace RCD;


Core::Core(bool robot_flag, MapHandler *map_):isRobot(robot_flag), casting_angles(N_rays)
{
  this->map = map_;
  if (isRobot){
    std::cout << "Initialized Robot Graph \n";

    node_.pos = map_->robot_pos;
    // Proximity
    node_.p   = std::sqrt(std::pow(map_->robot_pos.x - map_->target_pos.x,2) + std::pow(map_->robot_pos.y - map_->target_pos.y,2) );
    // Explorability (distance from parent)
    node_.e   = 0.;
    // Occurence 
    node_.o   = 0;
    // Total weight
    node_.cast_w   = 0.; // To do compute the weight with a function f(p,e,o)
  }else{
    std::cout << "Initialized Target Graph \n";

    node_.pos = map_->target_pos;
    // Proximity
    node_.p   = std::sqrt(std::pow(map_->robot_pos.x - map_->target_pos.x,2) + std::pow(map_->robot_pos.y -map_->target_pos.y,2) );
    // Explorability (distance from parent)
    node_.e   = 0.;
    // Occurence 
    node_.o   = 0;
    // Total weight
    node_.cast_w   = 0.; // To do compute the weight with a function f(p,e,o)
  }
  G.AddNode(node_);
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
  casting_dir = rand() % (360/N_rays + 1);  // Random  casting angle 
  for (int i = 0 ; i < N_rays ; ++i){
    casting_angles[i] = casting_dir + i*angle_increment;
  }
}

/*
Casts N_Rays into map_->grid
-->!!! This is the only method that changes the values of the grid!!!<--
*/
void Core::CastRays(MapHandler & handler)
{

}
