#  <span style="color:red;">R</span>ay <span style="color:red;">C</span>asting & <span style="color:red;">D</span>iffusion (<span style="color:red;">RCD</span>) model for global path planning 

# TODOs
* Fix max weight searching of the graph by filling up a list in real-time
* Fix shortest path from initial to final node (both target and robot)
* Fix merge robot and target graph for creating the first path
* Add the function f that decides how to update the weights
* Optimize code in CastRays
* Implement multiple runs on the same dataset and compute mean and std
* Refactor the random elements because they are pseudo-random every iteration as the same randomness
* Add multi-threading for robot and target update function. 
* Add guards for grid (avoid changing it simultaneously by two threads) 
* Refine the path with LoS?

# For future work
* The pointer to the map will be changed by three entities -> real-time occ. grid updated, robot , target

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


## Dependencies 

```
sudo apt-get install python3-tk
```
Install opencv
```
sudo apt install libopencv-dev python3-opencv
```
Get eigen3 and Boost


## Interactive occupancy grid path planning
In order to test RCD you simply clone the package and:
1. tkinter 
2. PIL 
```
$ python3 rcd/rcd.py
```
```
$ sudo apt-get install python3-pil python3-pil.imagetk
```
For now, only the interactive occupancy grid is released. This means that by pressing left mouse button, you can draw an occupancy grid and place the robot and target (by pressing any button) wherever you want. 
There is a detailed prompt that can guide you with shortcuts for producing and occupancy grid.

![Alt Text](.gif/RCD.gif)
