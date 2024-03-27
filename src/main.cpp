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
#include <thread>


int main()
{
  bool threading = true;
  std::cout << "Ray Casting and Diffusion model for Path Planning \n";
  PathPlanner rcd_path_planner(1);

  if (threading)
  {
    std::vector<std::thread> threads;
    for (int i = 1 ; i < 43 ; ++i){

    }
    std::vector<float> scales = {1, 2, 3, 4, 5, 6, 7, 8}; // Add as many values as needed

    for (int scale : scales) {
      threads.emplace_back([&rcd_path_planner, scale]() {
          rcd_path_planner.FindPath(scale);
      });
    }

    for (std::thread &thread : threads) {
      thread.join();
    }
  }else{
    rcd_path_planner.FindPath(10);
  }
 

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
