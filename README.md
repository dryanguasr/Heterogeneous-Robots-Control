# Heterogeneous Robots Control
This repository presents some of the simulations employed during the development of my masters thesis titled "Control of Heterogeneous Robot Networks for Assistance in Search and Rescue Tasks" the full document is available at [ResearchGate]( https://www.researchgate.net/publication/331302550_Control_of_Heterogeneous_Robot_Networks_for_Assistance_in_Search_and_Rescue_Tasks) and at [National University of Colombia repository](http://bdigital.unal.edu.co/69801/) 
## Convex Exploration Simulations
The folder [Convex](Convex/) presents the Heterogeneous DisCoverage exploration algorithm in a very simple convex environment. The file [FullSim.m](Convex/FullSim.m) generates 4 interest videos that correspond to:
* The map perspective of the robot team (Map.avi)
* The Voronoi cells and connectivity perspective (VoronoiCenter.avi)
* The exploration frontier in time (Frontier.avi)
* The Minimum time required for any robot to arrive to each position on the map based on the Flooding distance (MinTime.avi)
The execution of the full simulation should take a couple of minutes depending on the PC specifications, however in the subfolder [Videos](Convex/Videos) contains preloaded samples.
## Non-Convex Exploration Simulations
The folder [Non Convex](Non Convex/) presents the Heterogeneous DisCoverage exploration introduced in [Exploration with Heterogeneous Robots Networks for Search and Rescue](https://www.sciencedirect.com/science/article/pii/S240589631731217X) algorithm in a simple Non-convex environment. The file [FullSim.m](Non Convex/FullSim.m) generates the same videos (but the frontier perspective) as the Convex case and also contains preloaded samples.
## Full Mission Simulations
The folder [TaskAllocation](TaskAllocation/) presents the general workflow of the algoriths presented including the exploration of the environment, the automatic task allocation during the mission and the corresponding execution in a non-convex environment with certain amount of different victims of different types (true victim capable to move, true victim unable to move and false detection). The file [FullSim.m](TaskAllocation/FullSim.m) generates the same videos as the Non-Convex case and also contains preloaded samples. However in this case, in the Map perspective and in the Voronoi cells perspective show the current position of the victims and their type (Blue, green and yellow respectively).
