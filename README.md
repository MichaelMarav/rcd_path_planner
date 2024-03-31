#  <span style="color:red;">R</span>ay <span style="color:red;">C</span>asting & <span style="color:red;">D</span>iffusion (<span style="color:red;">RCD</span>) model for global path planning 
# Tutorial Under Construction
## Build
```
mkdir build && cd build
```
```
cmake .. -DCMAKE_BUILD_TYPE=Release
```
``` 
make
```
## Overview
RCD is a global path planner that given a binary occupancy grid, the target's and the robot's position produces a collision free path.


