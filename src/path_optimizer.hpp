#pragma once
#include <vector>
#include "utilities.hpp"
#include "map_handler.hpp"

class PathOptimizer
{
  private:
    std::vector<Point> originalPath;
    std::vector<Point> infusedPath; // Path with extra generated samples
    Point curr_point;
    Point next_point;

    std::vector<Point> GenerateSamples(); // Generates new samples on top of the originalPath and saves it in infusedPath
    void OptimizePath(); // Optimizes infusedPath with LoS (iterative)

    const int sampleIncrement = 2; // Distance between two consecutive generated samples 
  public:
    PathOptimizer(const std::vector<Point> & path, MapHandler *map_);
    
    std::vector<Point> optimizedPath; // Optimized path with LoS

    MapHandler* map; // pointer to the map object

};