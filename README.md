<div align="center">

# Heterogeneous Robots Control
### Research archive · Exploration, coverage and emergent tasks in heterogeneous robot networks

**MATLAB · cooperative robotics · frontier exploration · task allocation**

[Master's thesis (UNAL)](https://repositorio.unal.edu.co/handle/unal/68687) · [Related IFAC paper](https://www.sciencedirect.com/science/article/pii/S240589631731217X) · [Modern Mars sandbox](https://github.com/dryanguasr/NASA-Sandbox) · [Original README](README_TECHNICAL.md)

</div>

## Research question

How can robots with different mobility, sensing and task capabilities **explore an initially unknown environment and assign new tasks as information appears**? This repository preserves MATLAB experiments developed around *Control of Heterogeneous Robot Networks for Assistance in Search and Rescue Tasks*.

This is an **archival research artifact**, not a newly maintained ROS/Gazebo package. The code, precomputed videos and illustrations remain accessible as a scientific record.

## Explore the experiments

| Scenario | MATLAB entry point | What it illustrates |
| --- | --- | --- |
| Convex exploration | [`Convex/FullSim.m`](Convex/FullSim.m) | Heterogeneous DisCoverage, frontier, Voronoi and flooding-based time maps |
| Non-convex exploration | [`Non Convex/FullSim.m`](Non%20Convex/FullSim.m) | Exploration with internal obstacles and robot-dependent travel costs |
| Search-and-rescue mission | [`TaskAllocation/AutoTaskSim.m`](TaskAllocation/AutoTaskSim.m) | Emerging detections and task reallocations |
| Allocation logic | [`TaskAllocation/HRT.m`](TaskAllocation/HRT.m) | Cost matrix, capabilities and assignment through Munkres |

[Historical map videos](TaskAllocation/Map.avi) · [Voronoi video](TaskAllocation/VoronoiCenter.avi) · [Travel-time video](TaskAllocation/MinTime.avi).

## Approach

1. Build an individual robot view of free/occupied space and accessible areas.
2. Compute travel-time or Voronoi-based partitions and candidate exploration targets.
3. Share observations and generate tasks when possible victims are detected.
4. Allocate feasible tasks based on robot capability and estimated completion cost.
5. Replan until missions are completed or agents return.

## Reproducibility notice

The scripts are preserved close to their historical state. **Do not assume a clean run on a modern MATLAB release.** The old README pointed to a nonexistent `TaskAllocation/FullSim.m`; the current entry points are `AutoTaskSim.m` and `ManualTaskSim.m`. The archived `AutoTaskSim.m` also contains a potential configuration inconsistency (five robots but only four capability rows) that needs review before execution.

This documentation modernization **does not change historical algorithms or invalidate published research**. Verify dependencies, inputs and paths before interpreting any animation or comparing runs.

## Related continuation

[NASA-Sandbox](https://github.com/dryanguasr/NASA-Sandbox) explores a *different, partially known* setting with orbital prior maps and heterogeneous human/robot teams. It is **not** a validated one-to-one reimplementation of the thesis algorithm.

[Original detailed README](README_TECHNICAL.md) · [Repository license](LICENSE).
