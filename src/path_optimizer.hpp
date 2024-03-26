#pragma once
#include <vector>
#include "utilities.hpp"
#include "map_handler.hpp"

class PathOptimizer
{
  private:
    std::vector<iPoint> originalPath;
    std::vector<iPoint> infusedPath; // Path with extra generated samples
    std::vector<iPoint> interPath; // Path with extra generated samples
    int opt_point;
    iPoint curr_point;
    iPoint next_point;

    std::vector<iPoint> LoS(const std::vector<iPoint> & path, int opt);

    std::vector<iPoint> GenerateSamples(const std::vector<iPoint> & path, int start, int end);// Generates new samples on top of the originalPath and saves it in infusedPath
    bool HasLineOfSight(const iPoint& p1, const iPoint& p2);
    std::pair<iPoint,int> last_seen;

    int sampleIncrement; // Distance between two consecutive generated samples 
  public:

    std::vector<iPoint> optimizedPath; // Optimized path with LoS

    MapHandler* map; // pointer to the map object
    
    PathOptimizer(const std::vector<iPoint> & default_path, MapHandler *map_);
    void OptimizePath(bool stepOptimize); // Optimizes infusedPath with LoS (iterative)
    float PathDistance(const std::vector<iPoint> & path);


};