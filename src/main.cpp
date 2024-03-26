#include <iostream>
#include "utilities.hpp"
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"
#include "rcd_graph.hpp"
#include "path_optimizer.hpp" 
#include <numeric>
#include <fstream>

int main()
{
  // for (int f = 0 ; f < 10 ; ++f){
  std::cout << "Ray Casting and Diffusion model for Path Plannig \n";

  int N = 50;
  std::string prefix = "/home/michael/github/rcd_path_planner/maps/occ_12_53_54/";
  // std::ofstream outfile("/home/michael/github/rcd_path_planner/maps/random_boxes1/result_boxes/rcd.csv");
  // if (!outfile) {
  //     std::cerr << "Error: Unable to open file: "  << std::endl;
  // }

  // outfile << "coverage,time_mean,time_std,length_mean,length_std" << std::endl;


// for (int f = 0 ; f < 100 ; ++f)
// {   
    // MapHandler primary_handler(prefix + std::to_string(f) + ".ppm");
    MapHandler primary_handler(prefix + "occ_12_53_54" + ".ppm");//"/home/michael/github/rcd_path_planner/maps/mazes/1.ppm");

    std::vector<float> mean_time_list;
    std::vector<float> time;
    std::vector<float> path_length;
  // auto path_to_map = prefix + std::to_string(f) + ".ppm";
for (int i = 0 ; i < N ; ++i)
{
  auto handler = primary_handler;
  
  // Initializes the map and the relevant parameters (Maybe do this from config file to avoid building it every time)0
  // MapHandler handler( "/home/michael/github/rcd_path_planner/maps/random_boxes/82.ppm");

  RCD::Core::pathFound = false;

  // Visualization of the imported grid
  Visualizer plotter(handler);
  
  
  // Robot Caster
  RCD::Core RobotCaster(true, &handler);

  // Target Caster
  RCD::Core TargetCaster(false, &handler);
  plotter.VisualizeRays(handler); // For real-time plotting
  std::vector<iPoint> robot_path  ;
  std::vector<iPoint> target_path ;

  auto start = std::chrono::system_clock::now();
  // Main loop
  while (!RCD::Core::pathFound){

    RobotCaster.CastRays();

    TargetCaster.CastRays();

    plotter.VisualizeNodes(RobotCaster, TargetCaster);
    plotter.VisualizeRays(handler); // For real-time plotting
  }
  plotter.VisualizeRays(handler); // For real-time plotting
    plotter.VisualizeNodes(RobotCaster, TargetCaster);

  // Add the intersection node to the graph that didn't find the path
  // Finds the shortest path
  RCD::RGraph::Node final_node;
  if (RCD::Core::pathFoundByRobot){

    final_node = TargetCaster.AddIntersectionNode();
    
    robot_path = RobotCaster.ShortestPath(RCD::Core::intersectionNode);
    
    target_path = TargetCaster.ShortestPath(final_node);

    // plotter.VisualizePath(handler,target_path, RCD::Core::intersectionNode); // Visualize the casting path (fully-unoptimized)

  }else{
    final_node = RobotCaster.AddIntersectionNode();

    robot_path = RobotCaster.ShortestPath(final_node);
    
    target_path = TargetCaster.ShortestPath(RCD::Core::intersectionNode);

    // plotter.VisualizePath(handler,robot_path, final_node); // Visualize the casting path (fully-unoptimized)
  }
  
  // Remove Duplicate intersection point (from target / robot graph)
  std::reverse(target_path.begin(), target_path.end());
  robot_path.pop_back();
  // Connect the two paths
  robot_path.insert(robot_path.end(), target_path.begin(), target_path.end());


  plotter.VisualizePath(robot_path); // Visualize the casting path (fully-unoptimized)

  PathOptimizer los_optimizer(robot_path, &handler);

  while (true){
    los_optimizer.OptimizePath();
    plotter.VisualizePath(los_optimizer.optimizedPath);
  }

  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  // printInfo("Elapsed Time = " + std::to_string(elapsed_seconds.count()) + " (s)");
  // printInfo("Path Length  = " + std::to_string(los_optimizer.PathDistance(los_optimizer.optimizedPath)));
  time.push_back(static_cast<float>(elapsed_seconds.count()));
  path_length.push_back(los_optimizer.PathDistance(los_optimizer.optimizedPath));
  
  
}
  // For one maze
  float time_mean = std::accumulate(time.begin(), time.end(), 0.0f) / time.size();
  float path_length_mean = std::accumulate(path_length.begin(), path_length.end(), 0.0f) / path_length.size();
 
  // Compute standard deviation of time vector
  float time_std = 0.0f;
  for (float t : time) {
      time_std += (t - time_mean) * (t - time_mean);
  }
  time_std = std::sqrt(time_std / time.size());

// Compute standard deviation of path_length vector
  float path_length_std = 0.0f;
  for (float pl : path_length) {
      path_length_std += (pl - path_length_mean) * (pl - path_length_mean);
  }
  path_length_std = std::sqrt(path_length_std / path_length.size());
  mean_time_list.push_back(time_mean);
  // std::cout << "Running maze: " << f +1 << "\n" ;
  std::cout << "Mean time: " << time_mean ;
  std::cout << "   std time : " << time_std << std::endl;
  std::cout << "Mean of path length: " << path_length_mean;
  std::cout << "   std of path length: " << path_length_std << std::endl;

  // outfile << primary_handler.map_coverage << ',' << time_mean << ',' << time_std << ',' << path_length_mean << ',' << path_length_std << std::endl;

// }
  // // Print results
  // std::cout << "Final Result of time--> "<<  std::accumulate(mean_time_list.begin(), mean_time_list.end(), 0.0f) / mean_time_list.size();


  return 0;
}
