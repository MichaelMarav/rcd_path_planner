# Path Planner Package
# Main Idea
Perform two-way ray casting in order to find a path. Cast some N rays from robot and target. Inflate occupancy grid by R_robot/2 and consider the robot as a point. Each intersection between same rays is considered a new source point. Construct two trees, one with root the robot and one with root the target. Perform ray casting until there is an intersection between the beams of the robot with the beams of the target. Find the shortest path between the robot root and the target root. These points are connected with linear lines. Perform spline smoothing in these points to obtain a path. Maybe add a metric in order to choose which of the children to cast first.


### Ideas for choosing next point to diffuse:
1. https://www.youtube.com/watch?v=2iF9PRriA7w
2. https://www.youtube.com/watch?v=uAB9WOQCSxw
3. https://www.google.com/search?channel=fs&client=ubuntu&q=Value+Iteration


* Paper for finding the shortest path on a directed non negative weighted graph


"Integer priority queues with decrease key in constant time and the single source shortest paths problem"

## ToDos:

*Name*: RAY CASTING & DIFFUSION MODEL-> RA CA DIM
## Ongoing

* Add condition, when the path is not found to start rotating the source angles
* Implement line of sight to reduce way points

* <del> Create a directed weighted graph from the intesections of the ray beams </del>
* <del> Find the shortest path between robot and goal (number of points) </del>
* Find path from these points: Spline Smoothing or smthing 
* Figure out a metric to decide which point to cast next*


## Done 
* <del>Make the ray beam have the size of the robot and find a way if one part of the beam is stack what happens (Change the cmap value in plt.imshow())</del>
* <del> Add diffusion </del> 
* <del> Add condition to stop(when robot_beam finds the goal beam) </del>
* Desp: <del>Add saving occupancy grid option  </del>
* <del> Check Voronoi diagram </del>


## Ideas
* Weight each point based on some distance. EG distance traveled?
* Pick best point based on probability which is analogous to the discovered area
* Value Iteration
* Markov Decision process
* Exploration exploitation
* Self Organizing maps 


* <del> Rotate the source rays randomly. </del>

* Do multi threading 



