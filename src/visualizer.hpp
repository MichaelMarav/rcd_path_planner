#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "map_handler.hpp"
#include "utilities.hpp"
#include "rcd.hpp"
#include "rcd_graph.hpp"
#include <boost/graph/adjacency_list.hpp>

/*
Object for handling visualization of the produced path
*/

class Visualizer
{
  public:
    Visualizer(const MapHandler & map);
    Visualizer() = default; // Default constructor

    void visualizeOccupancyGrid(); 

    void VisualizeRays(const MapHandler & updatedMap);

    void VisualizePath(const MapHandler & thread_map, std::vector<iPoint> path,float thread_id);

    void VisualizeNodes(RCD::Core g1, RCD::Core g2);

  private:
    MapHandler map;    // The map with just the occupancy grid
    cv::Mat cast_image; // 3 channels for RGB
    cv::Mat path_image; // 3 channels for RGB

};