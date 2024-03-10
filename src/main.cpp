#include <iostream>
#include "utilities.hpp"
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"
#include "rcd_graph.hpp"
#include "path_optimizer.hpp" 



int main()
{
  std::cout << "Ray Casting and Diffusion model for Path Plannig \n";

  // Initializes the map and the relevant parameters (Maybe do this from config file to avoid building it every time)0
  // MapHandler handler("/home/michael/github/rcd_path_planner/maps/occ_17_3_52/occ_17_3_52.ppm");

  MapHandler handler("/home/michael/github/rcd_path_planner/maps/occ_17_3_52/occ_17_3_52.ppm");

  
  // Visualization of the imported grid
  Visualizer plotter(handler);

  // Robot Caster
  RCD::Core RobotCaster(true, &handler);

  // Target Caster
  RCD::Core TargetCaster(false, &handler);

  std::vector<Point> robot_path  ;
  std::vector<Point> target_path ;

  auto start = std::chrono::system_clock::now();
  // Main loop
  while (!RCD::Core::pathFound){

    RobotCaster.PrepareCasting();
    RobotCaster.CastRays();

    if (!RCD::Core::pathFound){
      TargetCaster.PrepareCasting();
      TargetCaster.CastRays();
    }
  
    // plotter.VisualizeNodes(handler, RobotCaster, TargetCaster);
    // plotter.VisualizeRays(handler); // For real-time plotting
  }
 
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


  // plotter.VisualizePath(handler,robot_path, final_node); // Visualize the casting path (fully-unoptimized)

  PathOptimizer los_optimizer(robot_path, &handler);

  // while (true){
    los_optimizer.OptimizePath();
    // plotter.VisualizePath(handler,los_optimizer.optimizedPath, RCD::Core::intersectionNode);

  // }



  // PathOptimizer los_optimizer(robot_path, &handler);


  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  printInfo("Elapsed Time = " + std::to_string(elapsed_seconds.count()) + " (s)");
  printInfo("Path Length  = " + std::to_string(los_optimizer.PathDistance(los_optimizer.optimizedPath)));
  plotter.VisualizePath(handler,los_optimizer.optimizedPath, RCD::Core::intersectionNode);

    // plotter.VisualizePath(handler,robot_path, final_node); // Visualize the casting path (fully-unoptimized)

  // plotter.VisualizeRays(handler);
  // plotter.VisualzePath(handler,robot_path, RCD::Core::intersectionNode); // Visualize the casting path (fully-unoptimized)

  // plotter.VisualzePath(handler,los_optimizer.optimizedPath, RCD::Core::intersectionNode);


  return 0;
}
