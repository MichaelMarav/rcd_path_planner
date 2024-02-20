#include <iostream>
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"
#include "rcd_graph.hpp"


int main()
{
    std::cout << "Ray Casting and Diffusion model for Path Plannig \n";

    // Initializes the map and the relevant parameters
    MapHandler handler("/home/michael/github/rcd_path_planner/maps/occ_2_30_0/occ_2_30_0.ppm");
    
    // Visualization of the imported grid
    Visualizer plotter(handler);

    // Robot Caster
    RCD::Core RobotCaster(true, &handler);

    // Target Caster
    RCD::Core TargetCaster(false, &handler);

    // RobotCaster.Update()
    // RobotCaster.Cast();

    // RCD::Graph my_graph;
    // Main loop
    // while (!RobotCaster.pathFound && !RobotCaster.pathFound){
    // RobotCaster.Update()
    // TargetCaster.Update()
    // TargetCaster.CastRays() 
    // TargetCaster.CastRays()
    // 
    //     RobotCaster.updateGraph();
    //     TargetCaster.cast(grid);
    //     RobotCaster.updateGraph()
    // }
    // Use path optimizer to fix the path
    // Use visualizer to save the path for paper TODO: Implement it with color
    return 0;
}