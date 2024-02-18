#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <fstream>




class RCD
{
  private:
    const int N_rays{6}; // Number of rays to be casted

    const std::string mapFilename{"/home/michael/github/rcd_path_planner/maps/occ_2_30_0/occ_2_30_0.ppm"}; 
   
    // OccupancyGrid grid;
    // std::vector<int> robot_pos;
    // std::vector<int> target_pos;

    // void LoadOccupancyGrid(OccupancyGrid & grid);

  public:
    RCD();

};