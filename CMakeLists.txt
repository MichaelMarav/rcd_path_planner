cmake_minimum_required(VERSION 3.12)
project(rcd)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(rcd_sources ${rcd_sources}
    src/path_optimizer.cpp
    src/rcd.cpp
)


find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost  REQUIRED NO_MODULE)

include_directories(${Boost_INCLUDE_DIR})
include_directories(${Eigen_INCLUDE_DIR} )

# Add executable target
add_executable(rcd "src/main.cpp" ${rcd_sources})
target_include_directories(rcd PRIVATE Eigen3::Eigen)
