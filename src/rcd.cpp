#include "rcd.hpp"

using namespace RCD;


Core::Core(bool robot_flag):isRobot(robot_flag)
{
  std::cout << "Initialized RCD\n";

  // Initialize graph
  node_.pos.x = 10;
  node_.pos.y = 15;

  node_.cast_w = 10.;
  node_.p = 10.;
  node_.e = 10.;
  node_.o = 10.;
  graph.AddNode(node_);
}







/*
Updates the grid by performing the casting procedure
*/
// void RCD::cast(MapHandler & grid)
// {
  
// } 
