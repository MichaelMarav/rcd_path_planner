#  <span style="color:red;">R</span>ay <span style="color:red;">C</span>asting & <span style="color:red;">D</span>iffusion (<span style="color:red;">RCD</span>) model for global path planning 

## Overview
RCD is a fast and efficient sampling-based global path planner that given a binary occupancy grid, the target's and the robot's position produces a collision free path.


## Build
Get the repo:
``` 
git clone https://github.com/MichaelMarav/rcd_path_planner.git
```
Go to the rcd_path_planner/ folder
```
mkdir build && cd build
```
```
cmake .. -DCMAKE_BUILD_TYPE=Release
```
``` 
make -j8
```

## Interactive grid plotter 

In order to generate your own custom occupancy grid, install Pillow:

```
pip install Pillow
```
and then run the script "scripts/interactive_grid.py"
```
python3 interactive_grid.py
```
a set of instructions will guide you to easily design your experiment.

## Run RCD
Modify the config file ("config/rcd_params.yaml") to specify which file you want to run RCD on.

Then go to the build folder and run RCD:

```
./rcd
```