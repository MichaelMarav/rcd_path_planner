#include <iostream>
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"
#include "rcd_graph.hpp"


int main()
{
    std::cout << "Ray Casting and Diffusion model for Path Plannig \n";

    // Initializes the map and the relevant parameters
    MapHandler handler("/home/michael/github/rcd_path_planner/maps/occ_21_2_38/occ_21_2_38.ppm");
    
    // Visualization of the imported grid
    Visualizer plotter(handler);

    // Robot Caster
    RCD::Core RobotCaster(true, &handler);

    // Target Caster
    // RCD::Core TargetCaster(false, &handler);

    // RobotCaster.Update(); // Prepare for casting
    // RobotCaster.CastRays();

    // Main loop
    while (!RobotCaster.pathFound){//&& !TargetCaster.pathFound){
        RobotCaster.Update();
        RobotCaster.CastRays();
        plotter.VisualzeRays(handler);
        // TargetCaster.Update();
        // TargetCaster.CastRays();
    }
    // Use path optimizer to fix the path
    // Use visualizer to save the path for paper TODO: Implement it with color
    return 0;
}