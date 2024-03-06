#include "path_optimizer.hpp"

PathOptimizer::PathOptimizer(const std::vector<Point> & default_path, MapHandler *map_)
:originalPath{default_path}, map{map_}
{

  printInfo("Initialized Path Optimizer");
  // GenerateSamples(originalPath);
  // OptimizePath(); // fills in the optimized path
  GenerateSamples(originalPath);            // Fills in the infused path

  // while (prev_path_length > curr_path_length)
  // {
  //   prev_path_length = curr_path_length;
  //   originalPath = optimizedPath;
  //   GenerateSamples(); // Fills in the infused path
  //   // optimizedPath = infusedPath;
  //   OptimizePath(); // fills in the optimized path
  //   PathDistance(); // updates curr_path_length
  // }
}

void PathOptimizer::GenerateSamples(const std::vector<Point> & path)
{
  infusedPath.clear();

  int dis;
  double distance_between_points = 0.0;
  double angle = 0.0;

  Point point2add; // point to add to the infused path
  for (int p = 0; p < path.size() - 1; ++p) {
    infusedPath.push_back(path[p]);
    dis = sampleIncrement;

    curr_point = path[p];
    next_point = path[p + 1];
    
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
}



/*
 * Optimizes the path using iterative LoS
 */
void PathOptimizer::OptimizePath()
{
  optimizedPath.push_back(originalPath[0]); // Add the starting node

  float prev_curr_length = PathDistance(infusedPath);
  float curr_path_length = prev_curr_length + 1;
  float epsilon = std::numeric_limits<float>::lowest();


  // while (std::abs(prev_curr_length - curr_path_length) > epsilon)
  // {
    prev_curr_length = curr_path_length;
    for (int i = 0 ; i < optimizedPath.size()  ; ++i){
      for (int j = i + 1 ; j < infusedPath.size() ; ++j){
        if (HasLineOfSight(optimizedPath[i], infusedPath[j])){
          continue;
        }else{
          // if (std::find(optimizedPath.begin(), optimizedPath.end(), infusedPath[j-1]) != optimizedPath.end()){
          //   break;
          // }else{
            optimizedPath.push_back(infusedPath[j-1]);
          // }
          std::cout << "Addded \n";

        }
      }
    }
    curr_path_length = PathDistance(optimizedPath);
    std::cout << "Lengths -> " << curr_path_length << "  " << prev_curr_length << '\n';
    GenerateSamples(optimizedPath); // Fills in the infused path based on the optimized path
  // }

}



// /*
//  * Optimizes the path using iterative LoS
// */
// void PathOptimizer::OptimizePath()
// {
//   optimizedPath.push_back(originalPath[0]);
//   int c = 0;
//   int p = 2;
//   while (p < infusedPath.size()){
//     // std::cout << "Optimized " << optimizedPath.back().x << "  " << optimizedPath.back().y << '\n'; 
//     // std::cout << "infused  " << originalPath.back().x << "  " << originalPath.back().y << '\n'; 

//     if (HasLineOfSight(infusedPath[c], infusedPath[p])){
//       ++p;
//     }else{
//       optimizedPath.push_back(infusedPath[p-1]);
//       c = p - 1;
//       ++p;
//     }
//   }
//   optimizedPath.push_back(originalPath.back());
// }



/*
 * returns true if there is a direct line of sight between p1 and p2 else it returns false
*/
bool PathOptimizer::HasLineOfSight(const Point& p1, const Point& p2)
{
  double dis = 1;
  
  double distance_between_points = CalculateDistance(p1,p2);
  
  double angle = std::atan2((p2.y - p1.y), (p2.x - p1.x));
  int x_i,y_i;
  while (dis < distance_between_points) {

    x_i = static_cast<int>(std::ceil(p1.x + dis * std::cos(angle)));
    y_i = static_cast<int>(std::ceil(p1.y + dis * std::sin(angle)));
    if (map->grid[y_i][x_i].isOccupied) {
        return false;
    }
    dis += 1;
  }

  return true;
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

