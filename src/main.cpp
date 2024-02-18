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
    // RCD RobotCaster(grid);
    // RCD TargetCaster(grid);

    return 0;
}