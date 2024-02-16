#pragma once
#include <iostream>
// #include <Eigen/Dense>
#include <vector>
#include <fstream>

class RCD
{
  private:
    const std::string rcd_params{"../config/rcd_params.yaml"};
    std::vector<std::vector<bool>> grid;
    std::vector<int> robot_pos;
    std::vector<int> target_pos;

    
  public:
    RCD();

};