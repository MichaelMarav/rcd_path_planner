#include "path_planner.hpp"

PathPlanner::PathPlanner(int NumberOfRuns, std::string BoxMapFile)
:N{NumberOfRuns} 
{
  LoadRCDparams();
  defaultMapHandler = MapHandler(BoxMapFile);

}


void PathPlanner::FindPath(float scale_value)
{
  for (int i = 0 ; i < N ; ++i)
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

      return;
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
      // plotter.VisualizePath(handler_i,los_optimizer.optimizedPath,scale_value);
    }
    



    // printInfo("Found path by thread --> "  + std::to_string(scale_value));
    // printInfo("Elapsed Time = " + std::to_string(elapsed_seconds.count()) + " (s)");
    // printInfo("Path Length  = " + std::to_string(los_optimizer.PathDistance(los_optimizer.optimizedPath)));
    // std::cout << "---------------------\n";


    time.push_back(static_cast<float>(elapsed_seconds.count()));
    path_length.push_back(los_optimizer.PathDistance(los_optimizer.optimizedPath));
    plotter.VisualizePath(handler_i,los_optimizer.optimizedPath,scale_value);

  }

  float sum_time = std::accumulate(time.begin(), time.end(), 0.0f);
  float mean_time =  sum_time / time.size();

  float sum_path = std::accumulate(path_length.begin(), path_length.end(), 0.0f);
  float mean_path =  sum_path / time.size();
  printInfo("Mean elapsed Time = " + std::to_string(mean_time) + " (s)");
  printInfo("Mean path Length  = " + std::to_string(mean_path));

  if (bestThreadResult[2] < 0){
    bestThreadResult[0] = defaultMapHandler.map_coverage;
    bestThreadResult[1] = mean_time;
    bestThreadResult[2] = mean_path;
    return;
  }
  if (mean_time < bestThreadResult[1] ){
    std::cout << " BEST TIME ---> " << bestThreadResult[1] << "CUrrent time ---> " << mean_time <<'\n';
    bestThreadResult[0] = defaultMapHandler.map_coverage;
    bestThreadResult[1] = mean_time;
    bestThreadResult[2] = mean_path;
    return;
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