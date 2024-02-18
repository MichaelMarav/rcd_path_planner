#include "visualizer.hpp"

Visualizer::Visualizer(const MapHandler & map): map(map)
{
  std::cout << "Visualizing Occupancy Grid \n";
  visualizeOccupancyGrid();

}


void Visualizer::visualizeOccupancyGrid()
{

  // Create an image to visualize the grid
  cv::Mat image(map.height, map.width, CV_8UC1);

  // Fill the image with appropriate colors based on the grid
  for (int i = 0; i < map.height; ++i) {
      for (int j = 0; j < map.width; ++j) {
          if (map.grid[i][j].isOccupied) {
              image.at<uchar>(i, j) = 0; // White for occupied cells
          } else {
              image.at<uchar>(i, j) = 255;   // Black for unoccupied cells
          }
      }
  }
  printInfo("Loaded Occupancy Grid");
  printInfo("Confirm tha the greed dot is the initial position the red one is the target");
  cv::Point center(38, 163); // Center coordinates
  int radius = 10; // Radius of the dot

  // Draw a blue filled circle (dot)
  cv::circle(image, center, radius, cv::Scalar(0, 0, 255), -1); // Color: Blue (BGR), -1 for filled circle



  // Display the image using OpenCV
  cv::imshow("Occupancy Grid", image);
  cv::waitKey(0);
}

void Visualizer::printInfo(const std::string & message)
{
  std::cout << "[INFO]  " << message << '\n';
}