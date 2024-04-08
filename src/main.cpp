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
  bool threading = false;
  std::cout << "Ray Casting and Diffusion model for Path Planning \n";



  if (threading)
  {
    std::vector<float> scales = {3, 4, 5, 6, 10}; // Add as many values as needed
    std::vector<std::thread> threads;
    std::vector<std::pair<float,std::vector<iPoint>>> results;

    PathPlanner rcd_path_planner;

    for (int scale : scales) {
      threads.emplace_back([&results, rcd_path_planner, scale]() mutable{
        std::pair<float,std::vector<iPoint>> result = rcd_path_planner.FindPath(scale);
        results.push_back(result);
      });
    }

    for (std::thread &thread : threads) {
      thread.join();
    }
        // Find the pair with the smallest execution time
    std::pair<float,std::vector<iPoint>> smallest_result;
    smallest_result.first = std::numeric_limits<float>::max();
    for (auto& result : results) {
      if (result.first < smallest_result.first) {
          smallest_result = result;
      }
    }
    if (rcd_path_planner.visualizeFinalPath){
      auto plotter = Visualizer(rcd_path_planner.defaultMapHandler);
      plotter.VisualizePath(rcd_path_planner.defaultMapHandler, smallest_result.second,100);
    }
   
  }
  else{
    PathPlanner rcd_path_planner;

    auto result = rcd_path_planner.FindPath(100);  

    if (rcd_path_planner.visualizeFinalPath){
      auto plotter = Visualizer(rcd_path_planner.defaultMapHandler);
      plotter.VisualizePath(rcd_path_planner.defaultMapHandler, result.second,100);
    }
   
  }

  return 0;
}