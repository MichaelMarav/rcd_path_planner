#pragma once


#include "utilities.hpp"
#include "rcd.hpp"
#include "map_handler.hpp"
#include "visualizer.hpp"
#include "rcd_graph.hpp"
#include "path_optimizer.hpp" 

#include <fstream>
#include <string>


class PathPlanner
{
  private:
    std::string mapFilename_;
    int N;
    int NumberOfRays;
    bool real_time_plotting;
    bool stepOptimization;
    std::vector<float> mean_time_list;
    std::vector<float> time;
    std::vector<float> path_length;

    void LoadRCDparams();
    MapHandler defaultMapHandler;
    Visualizer plotter;

  public:
    PathPlanner(int NumberOfRuns = 1);
    void WriteResults(const std::string & filename);
    void PrintResults();
    void FindPath(float scale_value);
};