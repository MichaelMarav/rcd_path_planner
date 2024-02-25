#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "map_handler.hpp"
#include "utilities.hpp"
/*
Object for handling visualization of the produced path
*/

class Visualizer
{
  public:
    Visualizer(const MapHandler & map);
    
    void visualizeOccupancyGrid(); 

    void VisualzeRays(const MapHandler & updatedMap);

    void VisualzePath(const MapHandler & updatedMap, std::vector<Point> path, RCD::RGraph::Node node);

  private:
    MapHandler  map;    // The map with just the occupancy grid
    cv::Mat cast_image; // 3 channels for RGB
    cv::Mat path_image; // 3 channels for RGB

};