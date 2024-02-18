#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <fstream>

#include "rcd_graph.hpp"

class RCD
{
  private:
    const int N_rays{6}; // Number of rays to be casted
    bool isRobot;

  public:
    bool pathFound{false}; // Flag set to true if the path is found

    RCD(bool robot_flag);
    // void cast(MapHandler & grid);



};