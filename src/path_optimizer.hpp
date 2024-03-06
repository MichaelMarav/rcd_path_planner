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

   

    void GenerateSamples(const std::vector<Point> & path); // Generates new samples on top of the originalPath and saves it in infusedPath
    bool HasLineOfSight(const Point& p1, const Point& p2);
    float PathDistance(const std::vector<Point> & path);

    const int sampleIncrement = 1; // Distance between two consecutive generated samples 
  public:
    PathOptimizer(const std::vector<Point> & default_path, MapHandler *map_);
    void OptimizePath(); // Optimizes infusedPath with LoS (iterative)

    std::vector<Point> optimizedPath; // Optimized path with LoS

    MapHandler* map; // pointer to the map object

};