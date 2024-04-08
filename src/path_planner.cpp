#include "path_planner.hpp"
#include <fstream>
PathPlanner::PathPlanner()
{
  LoadRCDparams();
  defaultMapHandler = MapHandler(mapFilename_);
}


std::pair<float,std::vector<iPoint>> PathPlanner::FindPath(float scale_value)
{

  auto handler_i = defaultMapHandler;
  plotter = Visualizer(handler_i);

  // Robot Caster
  RCD::Core RobotCaster(NumberOfRays, true, &handler_i,scale_value);

  // Target Caster
  RCD::Core TargetCaster(NumberOfRays,false, &handler_i,scale_value);
  // Visualization of the imported grid
  // plotter.VisualizeRays(handler_i, RobotCaster, TargetCaster); // For real-time plotting

  std::vector<iPoint> robot_path  ;
  std::vector<iPoint> target_path ;

  auto start = std::chrono::system_clock::now();
  int casting_times = 0;
  // Main loop
  if (real_time_plotting)
  {
    while (!RobotCaster.pathFound && !TargetCaster.pathFound && casting_times < 100000){

      RobotCaster.CastRays();

      TargetCaster.CastRays();

      plotter.VisualizeRays(handler_i, RobotCaster, TargetCaster); // For real-time plotting

      ++casting_times;
    }
    plotter.VisualizeRays(handler_i, RobotCaster, TargetCaster); // For real-time plotting

  }
  else
  {
    while (!RobotCaster.pathFound && !TargetCaster.pathFound && casting_times < 100000){

      RobotCaster.CastRays();

      TargetCaster.CastRays();
      ++casting_times;

    }
  }

  
  // Add the intersection node to the graph that didn't find the path
  // Finds the shortest path
  RCD::RGraph::Node final_node;
  if (RobotCaster.pathFound){
    final_node = TargetCaster.AddIntersectionNode(RobotCaster.intersectionNode,RobotCaster.intersectionEdge_id);
    
    robot_path = RobotCaster.ShortestPath(RobotCaster.intersectionNode);
    
    target_path = TargetCaster.ShortestPath(final_node);
  }
  else if (TargetCaster.pathFound)
  {
    final_node = RobotCaster.AddIntersectionNode(TargetCaster.intersectionNode,TargetCaster.intersectionEdge_id);

    target_path = TargetCaster.ShortestPath(TargetCaster.intersectionNode);

    robot_path = RobotCaster.ShortestPath(final_node);      
  }else{
    printInfo("No path found by thread --> " + std::to_string(scale_value));

    return {std::numeric_limits<float>::max(), robot_path};
  }


  // Remove Duplicate intersection point (from target / robot graph)
  std::reverse(target_path.begin(), target_path.end());
  robot_path.pop_back();
  // Connect the two paths
  robot_path.insert(robot_path.end(), target_path.begin(), target_path.end());


  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;

  PathOptimizer los_optimizer(robot_path, &handler_i);
  
  if (stepOptimization)
  {
    plotter.VisualizePath(handler_i,robot_path,scale_value); // Visualize the casting path (fully-unoptimized)

    while (true){
      los_optimizer.OptimizePath(stepOptimization);
      plotter.VisualizePath(handler_i, los_optimizer.optimizedPath,scale_value);
    }
  }else{
    los_optimizer.OptimizePath(stepOptimization);
  }
  



  printInfo("Elapsed Time = " + std::to_string(elapsed_seconds.count()) + " (s)");
  printInfo("Path Length  = " + std::to_string(los_optimizer.PathDistance(los_optimizer.optimizedPath)));
  std::cout << "---------------------\n";

  return {los_optimizer.PathDistance(los_optimizer.optimizedPath), los_optimizer.optimizedPath};
}

/**
 * @brief Loads the rcd_params.yaml and stores them in variables to be 
 * passed on the RCD objects
 */
void PathPlanner::LoadRCDparams()
{
  std::string current_directory = std::filesystem::current_path();
  std::string absolute_path = current_directory + "/../config/" + "rcd_params.yaml";

  YAML::Node config = YAML::LoadFile(absolute_path);
  this->NumberOfRays = config["num_beams"].as<unsigned int>();
  this->real_time_plotting = config["real_time_plotting"].as<bool>();
  this->mapFilename_ = config["map_folder"].as<std::string>() + config["filename"].as<std::string>();
  this->stepOptimization = config["real_time_path_optimization"].as<bool>();
  this->visualizeFinalPath = config["visualize_final_path"].as<bool>();
}