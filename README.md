# Motion Planner Package

### Ideas for choosing next point to diffuse:
1. https://www.youtube.com/watch?v=2iF9PRriA7w
2. https://www.youtube.com/watch?v=uAB9WOQCSxw
3. https://www.google.com/search?channel=fs&client=ubuntu&q=Value+Iteration


* Paper for finding the shortest path on a directed non negative weighted graph


"Integer priority queues with decrease key in constant time and the single source shortest paths problem"

## ToDos:

*Name*: RAY CASTING & DIFFUSION MODEL-> RA CA DIM
## Ongoing
1. Create a directed weighted graph from the intesections of the ray beams
2. Find the shortest path between robot and goal (number of points)
3. Find path from these points: Spline Smoothing or smthing 
4. Figure out a metric to decide which point to cast next*


## Done 
1. <del>Make the ray beam have the size of the robot and find a way if one part of the beam is stack what happens (Change the cmap value in plt.imshow())</del>
2. <del> Add diffusion </del> 
3. <del> Add condition to stop(when robot_beam finds the goal beam) </del>
4. Desp: <del>Add saving occupancy grid option  </del>
5. <del> Check Voronoi diagram <del>


## Ideas
* Weight each point based on some distance. EG distance traveled?
* Pich best point based on probability which is analogous to the discovered area
* Value Iteration
* Markov Decision process
* Exploration exploitation
* Self Organizing maps 


* Rotate the source rays randomly.

* Do multi threading 



