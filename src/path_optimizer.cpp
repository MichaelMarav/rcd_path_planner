#include "path_optimizer.hpp"

PathOptimizer::PathOptimizer(const std::vector<Point> & path, MapHandler *map_)
:originalPath{path}, map{map_}
{

  printInfo("Initialized Path Optimizer");
}

std::vector<Point> PathOptimizer::GenerateSamples()
{
  int dis = 1;
  double distance_between_points = 0.0;
  double angle = 0.0;

  Point point2add; // point to add to the infused path
  for (int p = 0; p < originalPath.size() - 1; ++p) {
      dis = 1;
      infusedPath.push_back(originalPath[p]);

      curr_point = originalPath[p];
      next_point = originalPath[p + 1];
      
      distance_between_points = std::sqrt(std::pow((static_cast<double>(curr_point.x) - static_cast<double>(next_point.x)), 2) +
                                          std::pow((static_cast<double>(curr_point.y) - static_cast<double>(next_point.y)), 2));
      angle = std::atan2((next_point.y - curr_point.y), (next_point.x - curr_point.x));

      while (dis < distance_between_points) {
          point2add.x = static_cast<int>(std::ceil(curr_point.x + dis * std::cos(angle)));
          point2add.y = static_cast<int>(std::ceil(curr_point.y + dis * std::sin(angle)));
          dis += sampleIncrement;
          if (map->grid[point2add.y][point2add.x].isOccupied) {
              continue;
          }
          infusedPath.push_back(point2add);
      }
  }
  return infusedPath;
}


/*
 * Optimizes the path using iterative LoS
*/
void PathOptimizer::OptimizePath()
{

}