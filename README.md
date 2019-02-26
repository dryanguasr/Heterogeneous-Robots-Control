# Heterogeneous Robots Control
This repository presents some of the simulations employed during the development of my masters thesis titled "Control of Heterogeneous Robot Networks for Assistance in Search and Rescue Tasks" the full document is available at https://www.researchgate.net/publication/331302550_Control_of_Heterogeneous_Robot_Networks_for_Assistance_in_Search_and_Rescue_Tasks
## Convex Exploration Simulations
The folder [Convex](Convex/) presents the Heterogeneous DisCoverage exploration algorithm in a very simple convex environment. The file [FullSim.m](Convex/FullSim.m) generates 4 interest videos that correspond to:
* The map perspective of the robot team (Map.avi)
* The Voronoi cells and connectivity perspective (VoronoiCenter.avi)
* The exploration frontier in time (Frontier.avi)
* The Minimum time required for any robot to arrive to each position on the map based on the Flooding distance (MinTime.avi)
The execution of the full simulation should take a couple of minutes depending on the PC specifications, however in the subfolder [Videos](Convex/Videos) contains preloaded samples.
