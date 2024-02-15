# ompl_benchmark

## Install ompl
`sudo apt-get install libompl-dev ompl-demos`

## Run with
`git clone https://github.com/mrsp/ompl_benchmark.git`

`cd ompl_benchmark && mkdir build && cd build && cmake .. && make -j4`

`./ompl_benchmark planner_runs planner_type path_to_map initial_position[0] initial_position[1] target_position[0] target_position[1]`
