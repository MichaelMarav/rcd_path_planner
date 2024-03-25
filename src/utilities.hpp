/* 
Contains generic data structures and libraries
*/

#pragma once

#include <iostream>
#include <fstream>
#include <filesystem> // For std::filesystem::current_path()

#include <chrono>     //start = std::chrono::system_clock::now();
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <queue> // For fast storing of weight
#include <unordered_map>


constexpr float PI = 3.14159265359f;


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

  Point operator+(const Point& other) const {
      return {x + other.x, y + other.y};
  }

  Point operator-(const Point& other) const {
      return {x - other.x, y - other.y};
  }

  Point operator*(double scalar) const {
    return {static_cast<int>(x * scalar), static_cast<int>(y * scalar)};
  }

};

struct fPoint
{
  float x;
  float y;

  fPoint(float x_, float y_):x{x_},y{y_}{}
  fPoint(){};

  // Overloaded == operator
  bool operator==(const fPoint& other) const {
      return (x == other.x) && (y == other.y);
  }

  // Overloaded != operator
  bool operator!=(const fPoint& other) const {
      return !(*this == other);
  }

  fPoint operator+(const fPoint& other) const {
      return {x + other.x, y + other.y};
  }

  fPoint operator-(const fPoint& other) const {
      return {x - other.x, y - other.y};
  }

  fPoint operator*(double scalar) const {
    return {x * scalar,y * scalar};
  }

};

inline void printInfo(const std::string & message)
{
  std::cout << "[INFO]  " << message << '\n';
}

inline double CalculateDistance(const Point & p1, const Point & p2)
{
  return std::sqrt( std::pow(static_cast<double>(p1.x) - static_cast<double>(p2.x),2.) +std::pow(static_cast<double>(p1.y) - static_cast<double>(p2.y),2.) );
}
