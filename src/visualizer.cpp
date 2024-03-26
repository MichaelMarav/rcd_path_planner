#include "visualizer.hpp"

/** 
 * @brief Constructor of visualizer object. Initializes the occupancy grid, the cv image for the visualizing the rays
 * and the path image used for visualizing the paths
 * @param input: The map object that contains all the infomation of the occupancy grid
 */

Visualizer::Visualizer(const MapHandler & map_object)
: map(map_object), path_image{cv::Mat(map_object.height,map_object.width,CV_8UC3)},cast_image{cv::Mat(map_object.height,map_object.width,CV_8UC3)}
{
  printInfo("Visualizing Occupancy Grid..");
  printInfo("Confirm that the dots are the initial and target positions");
  visualizeOccupancyGrid();

}

/** 
 * @brief Visualizes the occupancy grid with black for occupied and white for free space
 * Pops up an image with the grid
 */
void Visualizer::visualizeOccupancyGrid()
{
  // Fill the cast_image with appropriate colors based on the grid
  for (int i = 0; i < map.height; ++i) {
    for (int j = 0; j < map.width; ++j) {
      if (map.grid[i][j].isOccupied) {
        // Black for occupied cells
        cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      } else {
        // White for unoccupied cells
        cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }


  cv::Point robot(map.robot_pos.x, map.robot_pos.y); 
  cv::Point target(map.target_pos.x, map.target_pos.y); 
  int radius = 10; // Radius of the dot

  // Draw a blue filled circle (dot)
  cv::circle(cast_image, robot, radius, cv::Scalar(255, 0, 0 ), -1); // Color: Blue (BGR), -1 for filled circle
  cv::circle(cast_image, target, radius, cv::Scalar(0, 0, 255), -1); // Color: Blue (BGR), -1 for filled circle

  // Display the cast_image using OpenCV
  cv::imshow("Occupancy Grid", cast_image);
  cv::waitKey(0);

  cv::waitKey(0);
}



/**
 * @brief: Visualizes the occupancy grid's information about whether the robot has passed or the target
 * 
 * @param: updatedMap The map object that gets updated at each iteration with the new rays
 */
void Visualizer::VisualizeRays(const MapHandler & updatedMap)
{
  // Fill the cast_image with appropriate colors based on the grid
  for (int i = 0; i < updatedMap.height; ++i) {
    for (int j = 0; j < updatedMap.width; ++j) {
      if (updatedMap.grid[i][j].robotPass) {
        cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0); // Blue for robot rays
      } 

      if (updatedMap.grid[i][j].targetPass) {
        cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255); // Red for target rays
      }

      if (updatedMap.grid[i][j].onRectangle) {
        cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 120, 255); // Red for target rays
      }
    }
  }
  cv::Point robot(updatedMap.robot_pos.x, updatedMap.robot_pos.y); 
  cv::Point target(updatedMap.target_pos.x, updatedMap.target_pos.y); 

  
  int radius = 15; // Radius of the dot

  // Draw a blue filled circle (dot)
  cv::circle(cast_image, robot, radius, cv::Scalar(255, 0, 0), -1); // Color: Blue (BGR), -1 for filled circle
  cv::circle(cast_image, target, radius, cv::Scalar(0, 0, 255), -1); // Color: Blue (BGR), -1 for filled circle

  // cv::circle(cast_image, target, radius, cv::Scalar(0, 255, 0), -1); // Color: Blue (BGR), -1 for filled circle



  // Display the cast_image using OpenCV
  cv::imshow("Path", cast_image);
  cv::waitKey(0);

  cv::waitKey(0);
}



/**
 * @brief: Visualizes the path in the occupancy grid (not the rays)
 * 
 * @param path A vector with Point that construct the path in 2D.
 */
void Visualizer::VisualizePath(std::vector<iPoint> path)
{

  for (int i = 0; i < map.height; ++i) {
    for (int j = 0; j < map.width; ++j) {
      if (map.grid[i][j].isOccupied) {
        // Black for occupied cells
        cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      } else {
        // White for unoccupied cells
        cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }

  cv::Point robot(map.robot_pos.x, map.robot_pos.y); // robot coordinates
  cv::Point target(map.target_pos.x, map.target_pos.y); // target coordinates

  cv::circle(cast_image, robot, 10, cv::Scalar(255, 0, 0), -1);
  cv::circle(cast_image, target, 10, cv::Scalar(0, 0, 255), -1);

  int radius = 7; // Radius of the dot

  cv::Point prevPoint; // Store the previous point for line drawing
  int thickness = 4;  // Thickness of the lines (adjust as needed)

  // Loop through the path points
  for (const auto& point : path) {
    cv::Point currentPoint(point.x, point.y);
    // Draw the circle for the current point
    cv::circle(cast_image, currentPoint, radius, cv::Scalar(255, 0, 0), -1);

    // Draw a line between the current point and the previous point (if applicable)
    if (prevPoint.x != 0 &&  prevPoint.y != 0){
      cv::line(cast_image, prevPoint, currentPoint, cv::Scalar(0,255, 0), thickness);
    }

    prevPoint = currentPoint; // Update the previous point for the next iteration
  }
  // Display the cast_image using OpenCV
  cv::imshow("Path", cast_image);
  cv::waitKey(0);
  cv::waitKey(0);

}



/**
 * @brief: Adds the visualization of the node of each graph to the cast iamge. If called alone it does not pop a new image window
 * 
 * @param g1 Robot graph object
 * @param g2 Target graph object
 */  
void Visualizer::VisualizeNodes(RCD::Core g1, RCD::Core g2)
{

   for (auto vd : boost::make_iterator_range( boost::vertices(g1.G.G) )) 
    {
      // Access the node
      RCD::RGraph::Node& node = g1.G.G[vd];        
      cv::Point p(node.pos.x, node.pos.y); // robot coordinates
      cv::circle(cast_image, p,5, cv::Scalar(255, 0, 0), -1);
    }


    for (auto vd : boost::make_iterator_range(boost::vertices(g2.G.G)))
    {
      // Access the node
      RCD::RGraph::Node& node = g2.G.G[vd];        
      cv::Point p(node.pos.x, node.pos.y); // robot coordinates
      cv::circle(cast_image, p, 5, cv::Scalar(0, 0, 255), -1);
    }
}
