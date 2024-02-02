cmake_minimum_required(VERSION 3.12)
project(rcd_path_planner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Add the source files
file(GLOB SOURCES "src/main.cpp" "src/data_structures/*.cpp")

# Add executable target
add_executable(rcd_path_planner ${SOURCES})


target_include_directories(rcd_path_planner PRIVATE "src")
