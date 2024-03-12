#pragma once
#include <vector>
#include "utilities.hpp"
#include "map_handler.hpp"

class PathOptimizer
{
  private:
    std::vector<Point> originalPath;
    std::vector<Point> infusedPath; // Path with extra generated samples
    std::vector<Point> interPath; // Path with extra generated samples
    int opt_point;
    Point curr_point;
    Point next_point;

    std::vector<Point> LoS(const std::vector<Point> & path, int opt);

    std::vector<Point> GenerateSamples(const std::vector<Point> & path, int start, int end);// Generates new samples on top of the originalPath and saves it in infusedPath
    bool HasLineOfSight(const Point& p1, const Point& p2);

    const int sampleIncrement = 10; // Distance between two consecutive generated samples 
  public:
    PathOptimizer(const std::vector<Point> & default_path, MapHandler *map_);
    void OptimizePath(); // Optimizes infusedPath with LoS (iterative)
    float PathDistance(const std::vector<Point> & path);

    std::vector<Point> optimizedPath; // Optimized path with LoS

    MapHandler* map; // pointer to the map object
};