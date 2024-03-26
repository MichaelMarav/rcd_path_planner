#include <iostream>
#include "utilities.hpp"
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"
#include "rcd_graph.hpp"
#include "path_optimizer.hpp" 
#include <fstream>
#include <numeric>
#include <vector>
#include "path_planner.hpp"

int main()
{
  // for (int f = 0 ; f < 10 ; ++f){
  std::cout << "Ray Casting and Diffusion model for Path Planning \n";
  PathPlanner rcd_path_planner(5);
  rcd_path_planner.FindPath();
//   int N = 50;
//   std::string prefix = "/home/michael/github/rcd_path_planner/maps/mazes/";
//   // std::ofstream outfile("/home/michael/github/rcd_path_planner/maps/random_boxes1/result_boxes/rcd.csv");
//   // if (!outfile) {
//   //     std::cerr << "Error: Unable to open file: "  << std::endl;
//   // }

//   // outfile << "coverage,time_mean,time_std,length_mean,length_std" << std::endl;

  // outfile << primary_handler.map_coverage << ',' << time_mean << ',' << time_std << ',' << path_length_mean << ',' << path_length_std << std::endl;


  return 0;
}
