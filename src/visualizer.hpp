#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "map_handler.hpp"
/*
Object for handling visualization of the produced path
*/

class Visualizer
{
  public:
    Visualizer(const MapHandler & map);
    void visualizeOccupancyGrid(); 
    void printInfo(const std::string & message);


  private:
    const MapHandler & map;

};