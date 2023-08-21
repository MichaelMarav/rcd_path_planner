# Motion Planner Package



## RAY CASTING IDEA
*Name* ; RAY CASTING DIFFUSION MODEL-> RA CA DIM
0. Split the raycasting with multi threading. One function one input x,y
1. Change the scatter plot from the beams to be plotted as a grid values not dots in scatter
2. Make ray casting more generic. Add a list with points for diffusion. At first the list will contain only the one pose then the next one and the next one
3. Make the ray beam have the size of the robot and find a way if one part of the beam is stack what happens (Change the cmap value in plt.imshow())
4. Add diffusion 
5. Add condition to stop(when robot_beam finds the goal beam)
6. Add saving occupancy grid option  
7. Check Voronoi diagram



# CLUSTERING IDEAS
Goal: Develop a clustering algorithm for motion planning. The clusters will be non-convex so keep this in mind.
Maybe use the Mahalanobis distance. --> Distance between a point and a distribution D 
Maybe this as a metric: https://www.youtube.com/watch?v=LgeXhp-hQIw


# Literature
Spectral clustering for path planning:
 https://arxiv.org/abs/1712.06206

## Other methods for clustering 

### Hierachical clustering
https://www.google.com/search?channel=fs&client=ubuntu&q=hierachical+clustering


### Spectral Clustering
https://www.google.com/search?channel=fs&client=ubuntu&q=spectral+clustering

### Density based Clustering

https://pro.arcgis.com/en/pro-app/latest/tool-reference/spatial-statistics/how-density-based-clustering-works.htm

### DBSCAN clustering 

https://www.google.com/search?channel=fs&client=ubuntu&q=dbscan+clustering

### Agglomerative hierarchical clustering

https://www.google.com/search?channel=fs&client=ubuntu&q=Agglomerative+hierarchical+clustering+

### Grid-based methods

https://www.tutorialspoint.com/what-is-grid-based-methods


### Consensus clustering

https://www.google.com/search?channel=fs&client=ubuntu&q=Consensus+clustering

