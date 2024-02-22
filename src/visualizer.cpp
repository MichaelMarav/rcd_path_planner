#include "visualizer.hpp"

Visualizer::Visualizer(const MapHandler & map): map(map), image{cv::Mat(map.height,map.width,CV_8UC3)}
{
  std::cout << "Visualizing Occupancy Grid \n";
  visualizeOccupancyGrid();
}


void Visualizer::visualizeOccupancyGrid()
{
  // Fill the image with appropriate colors based on the grid
  for (int i = 0; i < map.height; ++i) {
    for (int j = 0; j < map.width; ++j) {
      if (map.grid[i][j].isOccupied) {
        // Black for occupied cells
        image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      } else {
        // White for unoccupied cells
        image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }

  printInfo("Loaded Occupancy Grid");
  printInfo("Confirm that the dots are the initial and target positions");
  cv::Point robot(map.robot_pos.x, map.robot_pos.y); // Center coordinates
  cv::Point target(map.target_pos.x, map.target_pos.y); // Center coordinates

  int radius = 5; // Radius of the dot

  // Draw a blue filled circle (dot)
  cv::circle(image, robot, radius, cv::Scalar(0, 255, 0 ), -1); // Color: Blue (BGR), -1 for filled circle
  cv::circle(image, target, radius, cv::Scalar(0, 0, 255), -1); // Color: Blue (BGR), -1 for filled circle



  // Display the image using OpenCV
  cv::imshow("Occupancy Grid", image);
  cv::waitKey(0);
}


void Visualizer::printInfo(const std::string & message)
{
  std::cout << "[INFO]  " << message << '\n';
}

/*
Visualizes the casted rays
*/
void Visualizer::VisualzeRays(const MapHandler & updatedMap)
{
  // Fill the image with appropriate colors based on the grid
  for (int i = 0; i < map.height; ++i) {
    for (int j = 0; j < map.width; ++j) {
      if (map.grid[i][j].robotPass) {
        // Black for occupied cells
        image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
      } 
      
      if (map.grid[i][j].targetPass) {
        // White for unoccupied cells
        image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
      }
    }
  }

  printInfo("Loaded Occupancy Grid");
  printInfo("Confirm that the dots are the initial and target positions");
  cv::Point robot(map.robot_pos.x, map.robot_pos.y); // Center coordinates
  cv::Point target(map.target_pos.x, map.target_pos.y); // Center coordinates

  
  int radius = 5; // Radius of the dot

  // Draw a blue filled circle (dot)
  cv::circle(image, robot, radius, cv::Scalar(0, 255, 0), -1); // Color: Blue (BGR), -1 for filled circle
  cv::circle(image, target, radius, cv::Scalar(0, 0, 255), -1); // Color: Blue (BGR), -1 for filled circle

  // cv::circle(image, target, radius, cv::Scalar(0, 255, 0), -1); // Color: Blue (BGR), -1 for filled circle



  // Display the image using OpenCV
  cv::imshow("Occupancy Grid", image);
  cv::waitKey(0);
}
