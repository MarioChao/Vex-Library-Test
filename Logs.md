# Changelog and Plans


## Plans

- [ ] Nonuniform splines like centripetal Catmull Rom
- [ ] Optimize the trajectory planner


## Giselle Geometry & Trajectory Constraints | 2025/04/10 - 2025/04/12

New [Giselle Geometry](./include/Aespa-Lib/Giselle-Geometry/):
- Polygon2D class
	- Define simple polygons using vertices specified in counter-clockwise order
	- Provides functions like `getArea()` and `containsPoint()`

New [trajectory constraints](./include/Pas1-Lib/Planning/Trajectories/trajectory-constraint.h),
inspired by that of [WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/constraints.html):
- Centripetal Acceleration Constraint
	- Limits the centripetal acceleration of the trajectory to **slow down around tight turns**.
- Polygon Region Constraint
	- Limits the velocity when the robot is inside a polygon region
- Use the trajectory planner's `addCenterTrajectoryConstraints()` function


## Optimization: Curve Sampling & Trajectory | 2025/04/04

Modified [curve sampler](./src/Pas1-Lib/Planning/Splines/curve-sampler.cpp):
- Optimization
	- It now integrates using midpoint sum, which reduces number of matrix multiplications by 2/3.

Modified [trajectory planner](./src/Pas1-Lib/Planning/Trajectories/trajectory-planner.cpp):
- Changed to accept **distanceStep** instead of **distanceResolution**.
- `setCurvatureFunction` can accept additional distances to sample at.
- Not much optimization found yet.


## Splines Refactor | 2025/04/03

When creating new cubic spline segments, control points are converted and stored as cubic Hermite control points.
- Thus, only the Hermite characteristic matrix is used
