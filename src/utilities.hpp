/* 
Contains generic data structures and libraries
*/

#pragma once

#include <iostream>
#include <fstream>
#include <chrono>     //start = std::chrono::system_clock::now();
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <queue> // For fast storing of weight
#include <unordered_map>



inline void printInfo(const std::string & message)
{
  std::cout << "[INFO]  " << message << '\n';
}

struct Point
{
  int x;
  int y;

  Point(int x_, int y_):x{x_},y{y_}{}
  Point(){};

  // Overloaded == operator
  bool operator==(const Point& other) const {
      return (x == other.x) && (y == other.y);
  }

  // Overloaded != operator
  bool operator!=(const Point& other) const {
      return !(*this == other);
  }

};



constexpr float PI = 3.14159265359f;