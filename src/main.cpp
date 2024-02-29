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

    // Initializes the map and the relevant parameters (Maybe do this from config file to avoid building it every time)0
    MapHandler handler("/home/michael/github/rcd_path_planner/maps/occ_17_3_52/occ_17_3_52.ppm");
    
    // Visualization of the imported grid
    Visualizer plotter(handler);

    // Robot Caster
    RCD::Core RobotCaster(true, &handler);

    // Target Caster
    RCD::Core TargetCaster(false, &handler);

    std::vector<Point> robot_path  ;
    std::vector<Point> target_path ;

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
        plotter.VisualizeRays(handler); // For real-time plotting
        plotter.VisualizeNodes(handler, RobotCaster, TargetCaster);
    }

    // Add the intersection node to the graph that didn't find the path
    // Finds the shortest path
    RCD::RGraph::Node final_node;
    if (RCD::Core::pathFoundByRobot){

      final_node = TargetCaster.AddIntersectionNode();
      
      robot_path = RobotCaster.ShortestPath(RCD::Core::intersectionNode);
      
      target_path = TargetCaster.ShortestPath(final_node);
      
      plotter.VisualizePath(handler,robot_path, RCD::Core::intersectionNode); // Visualize the casting path (fully-unoptimized)

    }else{

      RCD::RGraph::Node final_node = RobotCaster.AddIntersectionNode();

      robot_path = RobotCaster.ShortestPath(final_node);
      
      target_path = TargetCaster.ShortestPath(RCD::Core::intersectionNode);

      plotter.VisualizePath(handler,robot_path, final_node); // Visualize the casting path (fully-unoptimized)

    }

    // auto tmp_node =  RCD::Core::intersectionNode;

    // if (RCD::Core::pathFoundByRobot){
    //   printInfo("Path Found by ROBOT");
    //   TargetCaster.G.UpdateWeight(tmp_node, RCD::Core::intersectionNode.pos,RCD::Core::intersectionNode.pos, 10.);

    //   TargetCaster.G.AddNode(tmp_node,TargetCaster.G.G);
      
    //   // Find the source and target of the already implemented edge
    //   TargetCaster.G.source_vertex = boost::source(RCD::Core::intersectionEdge_id, TargetCaster.G.G);
    //   TargetCaster.G.target_vertex = boost::target(RCD::Core::intersectionEdge_id, TargetCaster.G.G);

    //   auto disAB = std::sqrt(std::pow(TargetCaster.G.G[TargetCaster.G.source_vertex].pos.x - tmp_node.pos.x,2) + std::pow(TargetCaster.G.G[TargetCaster.G.source_vertex].pos.y - tmp_node.pos.y,2));
    //   auto disBC = std::sqrt(std::pow(TargetCaster.G.G[TargetCaster.G.target_vertex].pos.x - tmp_node.pos.x,2) + std::pow(TargetCaster.G.G[TargetCaster.G.target_vertex].pos.y - tmp_node.pos.y,2)); 
    
    //   TargetCaster.G.AddEdge(tmp_node.node_descriptor, TargetCaster.G.source_vertex,TargetCaster.G.G,disAB);
    //   TargetCaster.G.AddEdge(tmp_node.node_descriptor, TargetCaster.G.target_vertex,TargetCaster.G.G,disBC);
      
    //   robot_path = RobotCaster.ShortestPath(RCD::Core::intersectionNode);
      
    //   target_path = TargetCaster.ShortestPath(tmp_node);

    // }else{
    //   printInfo("Path Found by TARGET");
    //   RobotCaster.G.UpdateWeight(tmp_node, RCD::Core::intersectionNode.pos,RCD::Core::intersectionNode.pos, 10.);

    //   RobotCaster.G.AddNode(tmp_node,RobotCaster.G.G);

    //   // Find the source and target of the already implemented edge
    //   RobotCaster.G.source_vertex = boost::source(RCD::Core::intersectionEdge_id, RobotCaster.G.G);
    //   RobotCaster.G.target_vertex = boost::target(RCD::Core::intersectionEdge_id, RobotCaster.G.G);

    //   auto disAB = std::sqrt(std::pow(RobotCaster.G.G[RobotCaster.G.source_vertex].pos.x - tmp_node.pos.x,2) + std::pow(RobotCaster.G.G[RobotCaster.G.source_vertex].pos.y - tmp_node.pos.y,2));
    //   auto disBC = std::sqrt(std::pow(RobotCaster.G.G[RobotCaster.G.target_vertex].pos.x - tmp_node.pos.x,2) + std::pow(RobotCaster.G.G[RobotCaster.G.target_vertex].pos.y - tmp_node.pos.y,2)); 
    
    //   RobotCaster.G.AddEdge(tmp_node.node_descriptor, RobotCaster.G.source_vertex,RobotCaster.G.G,disAB);
    //   RobotCaster.G.AddEdge(tmp_node.node_descriptor, RobotCaster.G.target_vertex,RobotCaster.G.G,disBC);

    //   robot_path = RobotCaster.ShortestPath(tmp_node);
      
    //   target_path = TargetCaster.ShortestPath(RCD::Core::intersectionNode);
    // }


    // std::reverse(robot_path.begin(), robot_path.end());
    // robot_path.insert(robot_path.end(), target_path.begin(), target_path.end());


    // PathOptimizer los_optimizer(robot_path, &handler);


    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    printInfo("Elapsed Time = " + std::to_string(elapsed_seconds.count()) + " (s)");
    plotter.VisualizeRays(handler);
    // plotter.VisualzePath(handler,robot_path, RCD::Core::intersectionNode); // Visualize the casting path (fully-unoptimized)

    // plotter.VisualzePath(handler,los_optimizer.optimizedPath, RCD::Core::intersectionNode);


    return 0;
}
