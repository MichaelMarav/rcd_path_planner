#include <iostream>
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"


int main()
{
    std::cout << "Ray Casting and Diffusion model for Path Plannig \n";
    // RCD planner;
    MapHandler grid("/home/michael/github/rcd_path_planner/maps/occ_2_30_0/occ_2_30_0.ppm");


    Visualizer plotter(grid);

    // Robot Caster
    RCD RobotCaster(true);

    // Target Caster
    RCD TargetCaster(false);

    // Main loop
    // while (!RobotCaster.pathFound && !RobotCaster.pathFound){
    //     RobotCaster.cast(grid); //use mutex
    //     RobotCaster.updateGraph();
    //     TargetCaster.cast(grid);
    //     RobotCaster.updateGraph()
    // }
    // Use path optimizer to fix the path
    // Use visualizer to save the path for paper TODO: Implement it with color
    return 0;
}