#include "path_optimizer.hpp"

PathOptimizer::PathOptimizer(const std::vector<Point> & default_path, MapHandler *map_)
:originalPath{default_path}, map{map_}
{

  printInfo("Initialized Path Optimizer");
  
  opt_point = 0;
  // First step optimization: one node should only be able to see one
  int c = 0 ;
  int p = c + 2;
  std::pair<Point,int> last_seen;
  
  interPath.push_back(originalPath[0]);

  while (p < originalPath.size()){

    if (HasLineOfSight(originalPath[c],originalPath[p])){
      ++p;
    }else{
      last_seen = {originalPath[p-1], p-1};
      if (p-1 == originalPath.size()-1){
        interPath.push_back(last_seen.first);
        originalPath = interPath; // So, original path holds the first layer of optimized initial path
        interPath.clear();
        return;
      }

      for (int i = last_seen.second + 1 ; i < originalPath.size() ; ++i){
        if (HasLineOfSight(originalPath[c],originalPath[i])){
          last_seen = {originalPath[i], i};
        }
      }
      c = last_seen.second;
      p = c + 2;
      interPath.push_back(last_seen.first);
    }
  }
  interPath.push_back(originalPath.back());
  originalPath = interPath; // So, original path holds the first layer of optimized initial path
  infusedPath = GenerateSamples(originalPath, 0, originalPath.size());
  infusedPath.push_back(originalPath.back());
  optimizedPath = infusedPath;

  // interPath.clear();
}

std::vector<Point> PathOptimizer::GenerateSamples(const std::vector<Point> & path, int start, int end)
{
  if (end > path.size()){
    printInfo("ERROR");
    exit(3); 
  }
  std::vector<Point> generatedPath;
  
  for (int i = 0 ; i < start + 1; ++i){
    generatedPath.push_back(path[i]);
  }

  int dis;
  double distance_between_points = 0.0;
  double angle = 0.0;

  Point point2add;


  for (int p = start; p < end-1; ++p) {
    dis = sampleIncrement;

    curr_point = path[p];
    next_point = path[p + 1];
    
    distance_between_points = CalculateDistance(curr_point, next_point);
    angle = std::atan2((next_point.y - curr_point.y), (next_point.x - curr_point.x));

    while (dis < distance_between_points) {
      point2add.x = static_cast<int>(std::ceil(curr_point.x + dis * std::cos(angle)));
      point2add.y = static_cast<int>(std::ceil(curr_point.y + dis * std::sin(angle)));
      dis += sampleIncrement;

      if (map->deflated_grid[point2add.y][point2add.x].isOccupied) {
          continue;
      }
      generatedPath.push_back(point2add);
    }
  }

  for (int i = end; i < path.size() ; ++i ){
    generatedPath.push_back(path[i]);
  }
  return generatedPath;
}

// This is the optimal line of sight for one poitn. It finds the truly last seen element for every node
std::vector<Point> PathOptimizer::LoS(const std::vector<Point> & path  , int opt)
{
  std::vector<Point> reducedPath;
  for (int i = 0 ; i < opt+1 ; ++i){
    reducedPath.push_back(path[i]);
  }

  std::pair<Point,int> last_seen;
  int p = opt+1 ;
  while (p < path.size()){
    if (HasLineOfSight(path[opt],path[p])){
      if (p == path.size()-1){
        reducedPath.push_back(path.back());
        return reducedPath;
      }
      ++p;
    }else{
      last_seen = {path[p-1],p-1};
      for (int i = p ; i < path.size(); ++i){
        if (HasLineOfSight(path[opt],path[i])){
          last_seen = {path[i],i};
        }
      }
      reducedPath.push_back(last_seen.first);
      // Fill in the rest as same
      for (int i = last_seen.second + 1 ; i < path.size()  ; ++i ){
        reducedPath.push_back(path[i]);
      }
      return reducedPath;
    }
  }
}




void PathOptimizer::OptimizePath()
{
  while (optimizedPath.back() == originalPath.back()){
    optimizedPath = LoS(optimizedPath,opt_point);
    optimizedPath = GenerateSamples(optimizedPath,opt_point, opt_point + 2);
    ++opt_point;  
  }
} 


/*
 * returns true if there is a direct line of sight between p1 and p2 else it returns false
*/
bool PathOptimizer::HasLineOfSight(const Point& p1, const Point& p2)
{
    // Calculate distance between points
    double distance_between_points = CalculateDistance(p1, p2);

    // Calculate angle
    double angle = std::atan2((p2.y - p1.y), (p2.x - p1.x));

    // Initialize Bresenham's algorithm parameters
    int x1 = static_cast<int>(p1.x);
    int y1 = static_cast<int>(p1.y);
    int x2 = static_cast<int>(p2.x);
    int y2 = static_cast<int>(p2.y);

    // Determine increments for x and y
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    // Iterate through points using Bresenham's algorithm
    while (true) {
        // Check if the current point is occupied
        for (int i = -1; i < 2; ++i) {
            for (int j = -1; j < 2; ++j) {
                if (map->deflated_grid[y1 + j][x1 + i].isOccupied) {
                    return false; // Line of sight blocked
                }
            }
        }

        // Check if reached the end point
        if (x1 == x2 && y1 == y2) {
            break;
        }

        // Calculate next point using Bresenham's algorithm
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }

    return true; // Line of sight not blocked
}


/*
 * Computes the path distance of the optimized path
 */
float PathOptimizer::PathDistance(const std::vector<Point> & path)
{
  float curr_path_length = 0;
    // double prev_path_length;
  for (int i = 0 ; i < path.size()-2 ; ++ i){
    curr_path_length += CalculateDistance(path[i], path[i+1]);
  }
  return curr_path_length;
}

