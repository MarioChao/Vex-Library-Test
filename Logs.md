# Changelog and Plans


## Plans

- [ ] Nonuniform splines like centripetal Catmull Rom
- [ ] Optimize the trajectory planner


## Optimization: Curve Sampling & Trajectory | 2025/04/04

Modified [curve sampler](./src/Pas1-Lib/Planning/Splines/curve-sampler.cpp):
- Optimization
	- It now integrates using midpoint sum, which reduces number of matrix multiplications by 2/3.

Modified [trajectory planner](./src/Pas1-Lib/Planning/Trajectories/trajectory-planner.cpp):
- Changed to accept **distanceStep** instead of **distanceResolution**.
- `setCurvatureFunction` can accept additional distances to sample at
- Not much optimization found yet.


## Splines Refactor | 2025/04/03

When creating new cubic spline segments, control points are converted and stored as cubic Hermite control points.
- Thus, only the Hermite characteristic matrix is used
