#include <iostream>
#include "utilities.hpp"
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"
#include "rcd_graph.hpp"
#include "path_optimizer.hpp" 

bool RCD::Core::pathFound = false;  // Initialize static variable 
bool RCD::Core::pathFoundByRobot = false;  // Initialize static variable 

int main()
{
    std::cout << "Ray Casting and Diffusion model for Path Plannig \n";

    // Initializes the map and the relevant parameters
    MapHandler handler("/home/michael/github/rcd_path_planner/maps/occ_17_3_52/occ_17_3_52.ppm");
    
    // Visualization of the imported grid
    Visualizer plotter(handler);

    // Robot Caster
    RCD::Core RobotCaster(true, &handler);

    // Target Caster
    RCD::Core TargetCaster(false, &handler);

    // Main loop
    auto start = std::chrono::system_clock::now();
    while (!RCD::Core::pathFound){

        RobotCaster.PrepareCasting();
        RobotCaster.CastRays();
        RobotCaster.UpdateGrid();

        if (!RCD::Core::pathFound){
            TargetCaster.PrepareCasting();
            TargetCaster.CastRays();
            TargetCaster.UpdateGrid();
        }
        // plotter.VisualzeRays(handler); // For real-time plotting
    }
    auto tmp_node =  RCD::Core::intersectionNode;
    std::vector<Point> robot_path  ;
    std::vector<Point> target_path ;

    if (RCD::Core::pathFoundByRobot){
      TargetCaster.G.AddNode(tmp_node,TargetCaster.G.G,RCD::Core::intersectionNode.pos,RCD::Core::intersectionNode.pos, 10.);
      
      // Find the source and target of the already implemented edge
      TargetCaster.G.source_vertex = boost::source(RCD::Core::intersectionEdge_id, TargetCaster.G.G);
      TargetCaster.G.target_vertex = boost::target(RCD::Core::intersectionEdge_id, TargetCaster.G.G);

      auto disAB = std::sqrt(std::pow(TargetCaster.G.G[TargetCaster.G.source_vertex].pos.x - tmp_node.pos.x,2) + std::pow(TargetCaster.G.G[TargetCaster.G.source_vertex].pos.y - tmp_node.pos.y,2));
      auto disBC = std::sqrt(std::pow(TargetCaster.G.G[TargetCaster.G.target_vertex].pos.x - tmp_node.pos.x,2) + std::pow(TargetCaster.G.G[TargetCaster.G.target_vertex].pos.y - tmp_node.pos.y,2)); 
    
      TargetCaster.G.AddEdge(tmp_node.node_descriptor, TargetCaster.G.source_vertex,TargetCaster.G.G,disAB);
      TargetCaster.G.AddEdge(tmp_node.node_descriptor, TargetCaster.G.target_vertex,TargetCaster.G.G,disBC);
      
      robot_path = RobotCaster.ShortestPath(RCD::Core::intersectionNode);
      
      target_path = TargetCaster.ShortestPath(tmp_node);

    }else{

      RobotCaster.G.AddNode(tmp_node,RobotCaster.G.G,RCD::Core::intersectionNode.pos,RCD::Core::intersectionNode.pos, 10.);
      
      // Find the source and target of the already implemented edge
      RobotCaster.G.source_vertex = boost::source(RCD::Core::intersectionEdge_id, RobotCaster.G.G);
      RobotCaster.G.target_vertex = boost::target(RCD::Core::intersectionEdge_id, RobotCaster.G.G);

      auto disAB = std::sqrt(std::pow(RobotCaster.G.G[RobotCaster.G.source_vertex].pos.x - tmp_node.pos.x,2) + std::pow(RobotCaster.G.G[RobotCaster.G.source_vertex].pos.y - tmp_node.pos.y,2));
      auto disBC = std::sqrt(std::pow(RobotCaster.G.G[RobotCaster.G.target_vertex].pos.x - tmp_node.pos.x,2) + std::pow(RobotCaster.G.G[RobotCaster.G.target_vertex].pos.y - tmp_node.pos.y,2)); 
    
      RobotCaster.G.AddEdge(tmp_node.node_descriptor, RobotCaster.G.source_vertex,RobotCaster.G.G,disAB);
      RobotCaster.G.AddEdge(tmp_node.node_descriptor, RobotCaster.G.target_vertex,RobotCaster.G.G,disBC);

      robot_path = RobotCaster.ShortestPath(tmp_node);
      
      target_path = TargetCaster.ShortestPath(RCD::Core::intersectionNode);


    }


    std::reverse(robot_path.begin(), robot_path.end());
    robot_path.insert(robot_path.end(), target_path.begin(), target_path.end());


    PathOptimizer los_optimizer(robot_path, &handler);

    // auto new_path = los_optimizer.GenerateSamples();
    auto end = std::chrono::system_clock::now();
    // for (int i = 0 ; i < robot_path.size() ; ++i){
    //     std::cout << robot_path[i].x << "  " <<robot_path[i].y <<'\n' ;
    // }

    std::chrono::duration<double> elapsed_seconds = end - start;
    printInfo("Elapsed Time = " + std::to_string(elapsed_seconds.count()) + " (s)");
    plotter.VisualzeRays(handler);

    // plotter.VisualzePath(handler,new_path, tmp_node);


    // plotter.VisualzePath(handler,target_path);

    // plotter.VisualzePath(handler,path2);



    
    // Use path optimizer to fix the path
    // Use visualizer to save the path for paper TODO: Implement it with color
    return 0;
}
