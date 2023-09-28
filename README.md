#  <span style="color:red;">R</span>ay <span style="color:red;">C</span>asting & <span style="color:red;">D</span>iffusion (<span style="color:red;">RCD</span>) model for global path planning 


## Overview
RCD is a global path planner that given a binary occupancy grid, the target's and the robot's position produces a collision free path.


## Interactive occupancy grid path planning
In order to test RCD you simply:
```
$ python3 main.py
```

For now, only the interactive occupancy grid is released. This means that by pressing left mouse button, you can draw an occupancy grid and place the robot and target (by pressing any button) wherever you want. 
There is a detailed prompt that can guide you with shortcuts for producing and occupancy grid.

![Alt Text](.gif/RCD.gif)