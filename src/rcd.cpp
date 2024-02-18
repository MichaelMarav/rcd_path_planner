#include "rcd.hpp"

RCD::RCD()
{
  std::cout << "Initialized RCD\n";
  // read rcd params.yaml
  // LoadOccupancyGrid(grid);

}

// void RCD::LoadOccupancyGrid(OccupancyGrid & grid)
// {
//   std::ifstream file(mapFilename, std::ios::binary);
//   if (!file) {
//       std::cerr << "Error: Unable to open file." << std::endl;
//       return;
//   }

//   std::string magicNumber;
//   int width, height, maxColor;
//   file >> magicNumber >> width >> height >> maxColor;

  // Initialize grid size
  
  // if (magicNumber != "P6") {
  //     std::cerr << "Error: Invalid PPM file format." << std::endl;
  //     return;
  // }

  // // Check if the maximum color value is within the valid range
  // if (maxColor != 255) {
  //   std::cerr << "Error: Maximum color value must be 255 for black and white images." << std::endl;
  //   return;
  // }

  // // Consume the newline character after the maxColor value
  // file.ignore(1);

  // for (int y = 0; y < height; ++y) {
  //     for (int x = 0; x < width; ++x) {
  //         unsigned char pixel;
  //         file.read(reinterpret_cast<char*>(&pixel), 1);

  //         // Assuming 0 is white and 255 is black
  //         bool isBlack = pixel == 255;

  //         grid[y][x].occupied = isBlack;
  //     }
  // }

  // file.close();
// }