#include "path_planner.hpp"

PathPlanner::PathPlanner(int NumberOfRuns)
:N{NumberOfRuns} 
{
  LoadRCDparams();
  defaultMapHandler = MapHandler(mapFilename_);
  plotter = Visualizer(defaultMapHandler);

}


void PathPlanner::FindPath()
{
  for (int i = 0 ; i < N ; ++i)
  {
    auto handler_i = defaultMapHandler;

    RCD::Core::pathFound = false;
    
    // Robot Caster
    RCD::Core RobotCaster(true, &handler_i,10.0);

    // Target Caster
    RCD::Core TargetCaster(false, &handler_i,10.0);
    // Visualization of the imported grid

    std::vector<iPoint> robot_path  ;
    std::vector<iPoint> target_path ;

    auto start = std::chrono::system_clock::now();

    // Main loop
    if (real_time_plotting)
    {
      while (!RCD::Core::pathFound){

        RobotCaster.CastRays();

        TargetCaster.CastRays();

        plotter.VisualizeNodes(RobotCaster, TargetCaster);
        plotter.VisualizeRays(handler_i); // For real-time plotting
      }
      plotter.VisualizeRays(handler_i); // For real-time plotting
      plotter.VisualizeNodes(RobotCaster, TargetCaster);

    }
    else
    {
      while (!RCD::Core::pathFound){

        RobotCaster.CastRays();

        TargetCaster.CastRays();
      }
    }
  

  
    // Add the intersection node to the graph that didn't find the path
    // Finds the shortest path
    RCD::RGraph::Node final_node;
    if (RCD::Core::pathFoundByRobot){
      final_node = TargetCaster.AddIntersectionNode();
      
      robot_path = RobotCaster.ShortestPath(RCD::Core::intersectionNode);
      
      target_path = TargetCaster.ShortestPath(final_node);
    }else{
      final_node = RobotCaster.AddIntersectionNode();

      robot_path = RobotCaster.ShortestPath(final_node);
      
      target_path = TargetCaster.ShortestPath(RCD::Core::intersectionNode);
    }
    
    // Remove Duplicate intersection point (from target / robot graph)
    std::reverse(target_path.begin(), target_path.end());
    robot_path.pop_back();
    // Connect the two paths
    robot_path.insert(robot_path.end(), target_path.begin(), target_path.end());


    plotter.VisualizePath(robot_path); // Visualize the casting path (fully-unoptimized)

    PathOptimizer los_optimizer(robot_path, &handler_i);
    
    if (stepOptimization)
    {
      while (true){
        los_optimizer.OptimizePath(stepOptimization);
        plotter.VisualizePath(los_optimizer.optimizedPath);
      }
    }else{
      los_optimizer.OptimizePath(stepOptimization);
      plotter.VisualizePath(los_optimizer.optimizedPath);

    }
    

    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    // printInfo("Elapsed Time = " + std::to_string(elapsed_seconds.count()) + " (s)");
    // printInfo("Path Length  = " + std::to_string(los_optimizer.PathDistance(los_optimizer.optimizedPath)));
    time.push_back(static_cast<float>(elapsed_seconds.count()));
    path_length.push_back(los_optimizer.PathDistance(los_optimizer.optimizedPath));

  }
  
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
}