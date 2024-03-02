#include "visualizer.hpp"

Visualizer::Visualizer(const MapHandler & input)
: map(input), path_image{cv::Mat(input.height,input.width,CV_8UC3)},cast_image{cv::Mat(input.height,input.width,CV_8UC3)}
{
  printInfo("Visualizing Occupancy Grid..");
  printInfo("Confirm that the dots are the initial and target positions");
  visualizeOccupancyGrid();
}


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
  int radius = 5; // Radius of the dot

  // Draw a blue filled circle (dot)
  cv::circle(cast_image, robot, radius, cv::Scalar(0, 255, 0 ), -1); // Color: Blue (BGR), -1 for filled circle
  cv::circle(cast_image, target, radius, cv::Scalar(0, 0, 255), -1); // Color: Blue (BGR), -1 for filled circle

  // Display the cast_image using OpenCV
  cv::imshow("Occupancy Grid", cast_image);
  cv::waitKey(0);
}



/*
Visualizes the casted rays
*/
void Visualizer::VisualizeRays(const MapHandler & updatedMap)
{
  // Fill the cast_image with appropriate colors based on the grid
  for (int i = 0; i < updatedMap.height; ++i) {
    for (int j = 0; j < updatedMap.width; ++j) {
      if (updatedMap.grid[i][j].robotPass) {
        // Black for occupied cells
        cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
      } 
      
      if (updatedMap.grid[i][j].targetPass) {
        // White for unoccupied cells
        cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
      }

        // cast_image.at<cv::Vec3b>(i, j) = cv::Vec3b( updatedMap.grid[i][j].edge_id.m_source, updatedMap.grid[i][j].edge_id.m_target, 0);
    }
  }
  printInfo("Press anything to visualize next cast");
  cv::Point robot(updatedMap.robot_pos.x, updatedMap.robot_pos.y); 
  cv::Point target(updatedMap.target_pos.x, updatedMap.target_pos.y); 

  
  int radius = 5; // Radius of the dot

  // Draw a blue filled circle (dot)
  cv::circle(cast_image, robot, radius, cv::Scalar(0, 255, 0), -1); // Color: Blue (BGR), -1 for filled circle
  cv::circle(cast_image, target, radius, cv::Scalar(0, 0, 255), -1); // Color: Blue (BGR), -1 for filled circle

  // cv::circle(cast_image, target, radius, cv::Scalar(0, 255, 0), -1); // Color: Blue (BGR), -1 for filled circle



  // Display the cast_image using OpenCV
  cv::imshow("Occupancy Grid", cast_image);
  cv::waitKey(0);
}

void Visualizer::VisualizePath(const MapHandler & updatedMap, std::vector<Point> path,RCD::RGraph::Node node)
{

  for (int i = 0; i < updatedMap.height; ++i) {
    for (int j = 0; j < updatedMap.width; ++j) {
      if (updatedMap.grid[i][j].isOccupied) {
        // Black for occupied cells
        path_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
      } else {
        // White for unoccupied cells
        path_image.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
    // Fill the cast_image with appropriate colors based on the grid
  for (int i = 0; i < updatedMap.height; ++i) {
    for (int j = 0; j < updatedMap.width; ++j) {
      if (updatedMap.grid[i][j].robotPass) {
        // Black for occupied cells
        path_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
      } 
      
      if (updatedMap.grid[i][j].targetPass) {
        // White for unoccupied cells
        path_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
      }
    }
  }
  cv::Point intersection(node.pos.x, node.pos.y); // robot coordinates

  cv::circle(path_image, intersection, 20, cv::Scalar(0, 255, 0), -1);

  cv::Point robot(map.robot_pos.x, map.robot_pos.y); // robot coordinates
  cv::Point target(map.target_pos.x, map.target_pos.y); // target coordinates

  cv::circle(path_image, robot, 10, cv::Scalar(0, 255, 0), -1);
  cv::circle(path_image, target, 10, cv::Scalar(0, 0, 255), -1);

  int radius = 5; // Radius of the dot

  cv::Point prevPoint; // Store the previous point for line drawing
  int thickness = 2;  // Thickness of the lines (adjust as needed)

  // Loop through the path points
  for (const auto& point : path) {
    cv::Point currentPoint(point.x, point.y);
    // Draw the circle for the current point
    cv::circle(path_image, currentPoint, radius, cv::Scalar(255, 0, 0), -1);

    // Draw a line between the current point and the previous point (if applicable)
    if (prevPoint.x != 0 &&  prevPoint.y != 0){
      cv::line(path_image, prevPoint, currentPoint, cv::Scalar(255, 0, 0), thickness);
    }

    prevPoint = currentPoint; // Update the previous point for the next iteration
  }
  // Display the cast_image using OpenCV
  cv::imshow("Path ", path_image);
  cv::waitKey(0);
}



  
void Visualizer::VisualizeNodes(const MapHandler & updatedMap, RCD::Core g1, RCD::Core g2)
{

   for (auto vd : boost::make_iterator_range( boost::vertices(g1.G.G) )) 
    {
      // Access the node
      RCD::RGraph::Node& node = g1.G.G[vd];        
      cv::Point p(node.pos.x, node.pos.y); // robot coordinates
      cv::circle(cast_image, p, 2, cv::Scalar(0, 255, 0), -1);
    }


    for (auto vd : boost::make_iterator_range(boost::vertices(g2.G.G)))
    {
      // Access the node
      RCD::RGraph::Node& node = g2.G.G[vd];        
      cv::Point p(node.pos.x, node.pos.y); // robot coordinates
      cv::circle(cast_image, p, 2, cv::Scalar(0, 0, 255), -1);
    }
}
