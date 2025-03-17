#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"
#include <stdio.h>

namespace {
	using pas1_lib::planning::splines::SplineCurve;
	using pas1_lib::planning::splines::CurveSampler;
	using pas1_lib::planning::trajectories::ConstraintSequence;
	using pas1_lib::planning::trajectories::TrajectoryPlanner;

	double trackWidth = 0.503937008
}

void pushNewSpline(SplineCurve spline, bool reverse, double maxVel) {
	CurveSampler splineSampler = CurveSampler(spline)
		.calculateByResolution(spline.getTRange().second * 10);
	TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(splineSampler.getDistanceRange().second)
		.addConstraint_maxMotion({maxVel, maxAccel})
		.addConstraint_maxCombinedVelocity(splineSampler, maxVel, trackWidth, 0.3, 60)
		.calculateMotionProfile(100);
}

void testTrajectory() {
	SplineCurve = 
	
}

int main() {
	
}
