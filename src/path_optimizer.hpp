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

    double curr_path_length;
    double prev_path_length;

    void GenerateSamples(); // Generates new samples on top of the originalPath and saves it in infusedPath
    void OptimizePath(); // Optimizes infusedPath with LoS (iterative)
    bool HasLineOfSight(const Point& p1, const Point& p2);
    void PathDistance();
    double ComputeDistance(const Point & p1, const Point & p2);

    const int sampleIncrement = 10; // Distance between two consecutive generated samples 
  public:
    PathOptimizer(const std::vector<Point> & path, MapHandler *map_);
    
    std::vector<Point> optimizedPath; // Optimized path with LoS

    MapHandler* map; // pointer to the map object

};