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

  private:
    const MapHandler & map;
    cv::Mat image; // 3 channels for RGB

};