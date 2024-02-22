#include "map_handler.hpp"

/*
  Loads map and initializes 
*/
MapHandler::MapHandler(const std::string & map_file): ppm_filename(map_file)
{
  std::cout << "Initializing Occupancy Grid..\n";
  std::cout << "Loading file: " << ppm_filename <<'\n';
  
  loadOccupancyGrid(ppm_filename);

  yaml_filename = ppm_filename.substr(0, ppm_filename.find_last_of('.')) + ".yaml";
  
  loadOccupancyParams(yaml_filename);

}

/*
  Loads Occupancy grid from .ppm file
*/
void MapHandler::loadOccupancyGrid(const std::string & ppm_filename)
{
  std::ifstream file(ppm_filename,std::ios::binary);
  if (!file.is_open()) {
      std::cerr << "Error: Unable to open the image file." << std::endl;
      exit(1);
  }

  std::string magic_number;

  file >> magic_number;

  if (magic_number != "P6") {
      std::cerr << "Error: Invalid PPM file format. Only P6 supported" << std::endl;
      exit(1);
  }

  int max_color;
  file >> this->width >> this->height >> max_color;
  if (max_color != 255) {
    std::cerr << "Error: Only PPM files with a maximum color value of 255 are supported." << std::endl;
    exit(1);
  }else{
    std::cout << "Loaded occupancy grid with dimensions:  " << this->width << " x " << this->height <<'\n'; 
  }
  // Resize the grid based on image dimensions
  initializeGrid(this->width, this->height);

  // Consume newline character after max_val
  file.get();


  for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
          unsigned char r, g, b;
          file.read(reinterpret_cast<char*>(&r), 1);
          file.read(reinterpret_cast<char*>(&g), 1);
          file.read(reinterpret_cast<char*>(&b), 1);
          
          // Assuming black is defined as RGB(0, 0, 0)
          // Convert to grayscale for simplicity
          // You may adjust this based on your definition of black
          bool isBlack = (r + g + b) / 3 < 128;

          grid[y][x].isOccupied = isBlack;
      }
  }
  file.close();
}


/*
Loads occupancy grid params
*/
void MapHandler::loadOccupancyParams(const std::string & yaml_file)
{
  YAML::Node config = YAML::LoadFile(yaml_file);

  this->robot_pos.x     = config["robot_position_x"].as<unsigned int>();
  this->robot_pos.y     = config["robot_position_y"].as<unsigned int>();
  this->target_pos.x    = config["target_position_x"].as<unsigned int>();
  this->target_pos.y    = config["target_position_y"].as<unsigned int>();
  this->grid_size_x     = config["workspace_dimension_x"].as<float>();
  this->grid_size_y     = config["workspace_dimension_y"].as<float>();
  this->grid_resolution = config["grid_resolution"].as<float>(); 
}

/*
Initilizes memory and attributes for the occupancy grid
*/
void MapHandler::initializeGrid(unsigned int width, unsigned int height) 
{
  grid.resize(height, std::vector<Cell>(width));

  for (int i = 0; i < this->height; ++i) {
    for (int j = 0; j < this->width; ++j) {
      grid[i][j].isOccupied = false;
      grid[i][j].robotPass  = false;
      this->grid[i][j].targetPass = false;
    }
  }
}
