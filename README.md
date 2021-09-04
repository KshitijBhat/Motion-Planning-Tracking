# Motion Planning and Tracking for Wheeled Robots
This repo is for all the path planning and tracking code.

```trajectory.py``` contains the following functions:

Function | What it does
---------|-------------
get_route |  command generation using set of path coordinates
generate_trajectory | cubic turn path generation using the commands 

```path_search.py``` contains the following functions:

Function | What it does
---------|-------------
initgraph |  generates the possible nodes in a binary occupancy grid
astar_path | returns a list of coordinates corresponding to the A* search algorithm
dijkstra_path | returns a list of coordinates corresponding to the Dijkstra algorithm
