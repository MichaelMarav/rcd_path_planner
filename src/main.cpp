#include <iostream>
#include "utilities.hpp"
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"
#include "rcd_graph.hpp"


bool RCD::Core::pathFound = false;  // Initialize static variable 

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
        
        TargetCaster.PrepareCasting();
        TargetCaster.CastRays();
        TargetCaster.UpdateGrid();
        // plotter.VisualzeRays(handler);
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    printInfo("Elapsed Time = " + std::to_string(elapsed_seconds.count()) + " (s)");
    plotter.VisualzeRays(handler);


    
    // Use path optimizer to fix the path
    // Use visualizer to save the path for paper TODO: Implement it with color
    return 0;
}