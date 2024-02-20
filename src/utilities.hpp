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

struct Point
{
  unsigned int x;
  unsigned int y;
};
