# Changelog and Plans


## Plans

- [ ] Nonuniform splines like centripetal Catmull Rom
- [ ] Optimize the trajectory planner


## Optimization: Curve Sampling & Trajectory | 2025/04/04

Optimized [curve sampler](./src/Pas1-Lib/Planning/Splines/curve-sampler.cpp) by reducing number of matrix multiplications by 2/3.<br>
It now integrates using midpoint sum.


## Splines Refactor | 2025/04/03

When creating new cubic spline segments, control points are converted and stored as cubic Hermite control points.
- Thus, only the Hermite characteristic matrix is used
