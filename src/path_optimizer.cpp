#include "path_optimizer.hpp"

PathOptimizer::PathOptimizer(const std::vector<Point> & default_path, MapHandler *map_)
:originalPath{default_path}, map{map_}
{

  printInfo("Initialized Path Optimizer");
  // GenerateSamples(originalPath);
  // OptimizePath(); // fills in the optimized path

  // while (prev_path_length > curr_path_length)
  // {
  //   prev_path_length = curr_path_length;
  //   originalPath = optimizedPath;
  //   GenerateSamples(); // Fills in the infused path
  //   // optimizedPath = infusedPath;
  //   OptimizePath(); // fills in the optimized path
  //   PathDistance(); // updates curr_path_length
  // }


  optimizedPath.clear();

  // First layer of optimization (make only 2 consecutive nodes have LoS between them)
  int c = 0; 
  int p = 1;
  optimizedPath.push_back(originalPath[0]); // Add source
  while (p < originalPath.size())
  {
    if (HasLineOfSight(originalPath[c],originalPath[p]))
    {
      ++p;
    }
    else
    {
      std::pair<Point,int> last_seen(originalPath[p-1],p-1);
      for (int i = p + 1 ; i < originalPath.size() ; ++i)
      {
        if (HasLineOfSight(originalPath[c],originalPath[i]))
        {
          last_seen = {originalPath[i], i};
        } 
      }
      optimizedPath.push_back(last_seen.first);
      c = last_seen.second;
      p = c + 1;
    }
  }
  optimizedPath.push_back(*(originalPath.end() -1));
  GenerateSamples(optimizedPath);


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
      if (map->deflated_grid[point2add.y][point2add.x].isOccupied) {
          continue;
      }
      infusedPath.push_back(point2add);
    }
  }
}

void PathOptimizer::OptimizePath()
{
  int opt_point = 0;
    std::cout << " Path size   "  << infusedPath.size() << '\n';
  int prev_p = 0;
  while(opt_point < infusedPath.size())
  {
    std::cout << " Path size   "  << infusedPath.size() << '\n';

    std::cout << "Opt --> " << opt_point << '\n';
    // Second Layer of optimization:
    GenerateSamples(optimizedPath);
    optimizedPath.clear();
    optimizedPath.push_back(originalPath[0]);
    int c = opt_point;
    int p = c + 1;

    while (p < infusedPath.size())
    {
      if (HasLineOfSight(infusedPath[c],infusedPath[p]))
      {
        ++p;
      }
      else
      {
        std::pair<Point,int> last_seen(infusedPath[p-1],p-1);
        for (int i = p + 1 ; i < infusedPath.size() ; ++i)
        {
          if (HasLineOfSight(infusedPath[c],infusedPath[i]))
          {
            last_seen = {infusedPath[i], i};
          } 
        }
        optimizedPath.push_back(last_seen.first);
        c = last_seen.second;
        p = c + 1;
      }
    }
    optimizedPath.push_back(*(originalPath.end() -1));
    ++opt_point;
    if (prev_p == p){
      break;
    }
    prev_p = p;

  }
  


} 


/* OLD
 * Optimizes the path using iterative LoS
 */
// void PathOptimizer::OptimizePath()
// {
//   if (optimizedPath.size() > 1 ){
//     GenerateSamples(optimizedPath);            // Fills in the infused path
//   }else{
//     GenerateSamples(originalPath);            // Fills in the infused path
//   }
//   optimizedPath.clear();
//   // optimizedPath.push_back(originalPath[0]); // Add the starting node

//   int c = 0;
//   int p = 1;
//   while (p < infusedPath.size()){
    
//     if (p == infusedPath.size()-1) break; // Done
 
//     if (HasLineOfSight(infusedPath[c], infusedPath[p]))
//     {
//       ++p;
//     }
//     else
//     {
//       if (std::count(prevPath.begin(), prevPath.end(), infusedPath[p-1]) > 0)
//       {
//         c++;
//         p = c + 1;
//         continue;
//       }
//       else
//       {
//         optimizedPath.push_back(infusedPath[c]);
//         optimizedPath.push_back(infusedPath[p-1]);
//         c = p - 1;
//         ++p;
//         // if (HasLineOfSight(optimizedPath.back(),infusedPath[p-1]))
//         // {
//         //   optimizedPath.push_back(infusedPath[p-1]);

//         // }
//         // else
//         // {
//         //   ++c;
//         //   p = c+1;
//         // }
        
//       }

//     }
//   }
//   optimizedPath.push_back(originalPath.back());
//   prevPath = optimizedPath;
//  std::cout << "Path Distance --> " <<  PathDistance(optimizedPath) << '\n';
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
    if (map->deflated_grid[y_i][x_i].isOccupied) {
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

