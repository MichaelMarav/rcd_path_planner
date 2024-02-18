cmake_minimum_required(VERSION 3.12)
project(rcd)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(rcd_sources ${rcd_sources}
    src/path_optimizer.cpp
    src/rcd.cpp
    src/visualizer.cpp
    src/map_handler
)


find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
# find_package(Boost  REQUIRED NO_MODULE)

# include_directories(${Boost_INCLUDE_DIR})
# include_directories(${EIGEN_INCLUDE_DIR})

# Add executable target
add_executable(rcd "src/main.cpp" ${rcd_sources}) 

target_include_directories(rcd
 PRIVATE
  src/
)
target_link_libraries(rcd PRIVATE Eigen3::Eigen ${OpenCV_LIBS})
