#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <stdio.h>
#include <cmath>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Plan Point ----------

PlanPoint::PlanPoint(double time_seconds, double distance, std::vector<double> motion_dV_dT)
	: time_seconds(time_seconds), distance(distance),
	motion_dV_dT(motion_dV_dT) {}

PlanPoint &PlanPoint::constrain(Constraint constraint) {
	for (int i = 0; i < (int) motion_dV_dT.size(); i++) {
		// Validate constraint
		if (i >= (int) constraint.maxMotion_dV_dT.size()) {
			break;
		}

		// Constrain
		double maxValue = constraint.maxMotion_dV_dT[i];
		double currentValue = motion_dV_dT[i];
		motion_dV_dT[i] = aespa_lib::genutil::clamp(currentValue, -maxValue, maxValue);
	}
	return *this;
}

double getTimeStepFromDistanceStep(PlanPoint node, double distanceStep) {
	double bs_l, bs_r, bs_m;
	int bs_trials = 0;
	double bs_error = 1e9;
	bs_l = 0;
	bs_r = 1;
	while (fabs(bs_error) >= 1e-5 && bs_trials <= 100) {
		bs_m = bs_l + (bs_r - bs_l) / 2;
		double bs_distanceStep = aespa_lib::genutil::integratePolynomial(node.motion_dV_dT, bs_m).first;
		bs_error = fabs(distanceStep) - fabs(bs_distanceStep);
		if (bs_error > 0) {
			bs_l = bs_m;
		} else {
			bs_r = bs_m;
		}
		bs_trials++;
	}
	return bs_m;
}


// ---------- Trajectory Planner ----------

TrajectoryPlanner::TrajectoryPlanner(
	double distance_inches,
	double trackWidth_inches,
	std::function<double(double)> distanceToCurvature_function,
	std::vector<double> startMotion_dInches_dSec,
	std::vector<double> endMotion_dInches_dSec
)
	: trackWidth(trackWidth_inches),
	startMotion(startMotion_dInches_dSec),
	endMotion(endMotion_dInches_dSec),
	distanceToCurvature_function(distanceToCurvature_function) {
	distance = fabs(distance_inches);
	distance_sign = aespa_lib::genutil::signum(distance_inches);
};

TrajectoryPlanner::TrajectoryPlanner(
	double distance_inches,
	double trackWidth_inches, std::function<double(double)> distanceToCurvature_function
)
	: TrajectoryPlanner(distance_inches, trackWidth_inches, distanceToCurvature_function, { 0, 1e5 }, { 0, -1e5 }) {}

TrajectoryPlanner::TrajectoryPlanner(double distance_inches)
	: TrajectoryPlanner(distance_inches, 0, nullptr, { 0, 1e5 }, { 0, -1e5 }) {}

TrajectoryPlanner::TrajectoryPlanner()
	: TrajectoryPlanner(0) {};

TrajectoryPlanner &TrajectoryPlanner::addConstraintSequence(ConstraintSequence constraints) {
	constraintSequences.push_back(constraints);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addConstraint_maxMotion(std::vector<double> maxMotion_dV_dT) {
	addConstraintSequence(
		ConstraintSequence()
		.addConstraints({ {0, maxMotion_dV_dT} })
	);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addConstraint_maxAngularMotion(std::vector<double> maxAngularMotion, int distanceResolution) {
	ConstraintSequence constraintSequence;
	for (int resolutionI = 0; resolutionI <= distanceResolution; resolutionI++) {
		// Get distance
		double x = aespa_lib::genutil::rangeMap(resolutionI, 0, distanceResolution, 0, distance);

		// Get curvature
		double k = distanceToCurvature_function(x);

		// Max linear velocity = max angular velocity / curvature
		std::vector<double> maxMotion = maxAngularMotion;
		if (fabs(k) < 1e-5) {
			for (auto &value : maxMotion) {
				value = 1e8;
			}
		} else {
			for (auto &value : maxMotion) {
				value /= fabs(k);
			}
		}

		// Add constraint
		constraintSequence.addConstraints({
			{x, maxAngularMotion}
		});
	}

	// Add sequence
	addConstraintSequence(constraintSequence);

	return *this;
}

PlanPoint TrajectoryPlanner::_getNextPlanPoint(PlanPoint node, double distanceStep) {
	// Get minimum constraints
	double oldDistance = node.distance;
	std::vector<Constraint> constraints0 = getConstraintsAtDistance(
		constraintSequences, oldDistance
	);
	std::vector<Constraint> constraints1 = getConstraintsAtDistance(
		constraintSequences, oldDistance + distanceStep
	);
	std::vector<Constraint> track_constraints0 = getConstraintsAtDistance(
		track_constraintSequences, oldDistance
	);
	std::vector<Constraint> track_constraints1 = getConstraintsAtDistance(
		track_constraintSequences, oldDistance + distanceStep
	);
	Constraint minConstraint0 = getMinimumConstraint(constraints0);
	Constraint track_minConstraint0 = getMinimumConstraint(track_constraints0);

	// Expand constrain degree
	int constraintSize = (int) minConstraint0.maxMotion_dV_dT.size();
	if (constraintSize > (int) node.motion_dV_dT.size()) {
		node.motion_dV_dT.resize(constraintSize);
	}

	// Minimum constrain
	node.constrain(minConstraint0);

	// Constrain left & right
	if (trackWidth > 0 && distanceToCurvature_function) {
		double curvature = distanceToCurvature_function(node.distance);
		double factor = (1 + fabs(curvature) * trackWidth / 2);
		node.motion_dV_dT = aespa_lib::genutil::multiplyVector(node.motion_dV_dT, factor);
		node.constrain(minConstraint0);
		node.constrain(track_minConstraint0);
		node.motion_dV_dT = aespa_lib::genutil::multiplyVector(node.motion_dV_dT, 1 / factor);
	}

	// Maximize last constrained degree
	int lastConstrainedIndex = fmin((int) node.motion_dV_dT.size(), (int) minConstraint0.maxMotion_dV_dT.size()) - 1;
	double maxLastMotion_value = minConstraint0.maxMotion_dV_dT[lastConstrainedIndex];
	node.motion_dV_dT[lastConstrainedIndex] = maxLastMotion_value * aespa_lib::genutil::signum(distanceStep);


	// Binary search for time step
	double timeStep_seconds = getTimeStepFromDistanceStep(node, distanceStep);
	timeStep_seconds *= aespa_lib::genutil::signum(distanceStep);

	// printf("TS: %.3f\n", timeStep_seconds);
	// printf("OLD: %.3f ", oldDistance);
	// for (double a : node.motion_dV_dT) {
	// 	printf("%.3f ", a);
	// }
	// printf("\n");

	// Integrate
	auto integral = aespa_lib::genutil::integratePolynomial(
		node.motion_dV_dT, timeStep_seconds
	);
	double newDistance = oldDistance + integral.first;
	double time_seconds = node.time_seconds + timeStep_seconds;
	PlanPoint newNode(time_seconds, newDistance, integral.second);


	// Minimum constrain
	Constraint minConstraint1 = getMinimumConstraint(constraints1);
	Constraint track_minConstraint1 = getMinimumConstraint(track_constraints1);
	newNode.constrain(minConstraint1);

	// Constrain left & right
	if (trackWidth > 0 && distanceToCurvature_function) {
		// Get curvature & linear + angular factor
		double curvature = distanceToCurvature_function(newNode.distance);
		double factor = (1 + fabs(curvature) * trackWidth / 2);

		std::vector<double> old = newNode.motion_dV_dT;

		// Constrain
		newNode.motion_dV_dT = aespa_lib::genutil::multiplyVector(newNode.motion_dV_dT, factor);
		newNode.constrain(minConstraint1);
		newNode.constrain(track_minConstraint1);
		newNode.motion_dV_dT = aespa_lib::genutil::multiplyVector(newNode.motion_dV_dT, 1 / factor);

		std::vector<double> newm = newNode.motion_dV_dT;
		double A = (newm[0] - old[0]) / timeStep_seconds;
		if (false && fabs(A) > 3.5) {
			printf("OLD: %.3f ", node.time_seconds);
			printf("%.3f ", oldDistance);
			for (double a : node.motion_dV_dT) {
				printf("%.3f ", a);
			}
			printf("\n");
			printf("NEW 0: %.3f ", newNode.time_seconds);
			printf("%.3f ", newDistance);
			for (double a : old) {
				printf("%.3f ", a);
			}
			printf("\n");
			printf("NEW: %.3f ", newNode.time_seconds);
			printf("%.3f ", newDistance);
			for (double a : newm) {
				printf("%.3f ", a);
			}
			printf("\n");
			printf("A: %.3f\n", A);
		}
	}

	// Derivative constrain
	for (int i = 1; i < (int) newNode.motion_dV_dT.size(); i++) {
		double deriv = fabs((newNode.motion_dV_dT[i - 1] - node.motion_dV_dT[i - 1]) / timeStep_seconds);
		newNode.motion_dV_dT[i] = aespa_lib::genutil::clamp(newNode.motion_dV_dT[i], -deriv, deriv);
	}

	// printf("NEW: %.3f ", newDistance);
	// for (double a : newNode.motion_dV_dT) {
	// 	printf("%.3f ", a);
	// }
	// printf("\n");

	// TODO: remove sharp deceleration

	// Return
	return newNode;
}

std::vector<PlanPoint> TrajectoryPlanner::_forwardPass(double distanceStep) {
	std::vector<PlanPoint> planningPoints;
	planningPoints.push_back(
		PlanPoint(0, 0, startMotion)
		.constrain(getMinimumConstraint(getConstraintsAtDistance(constraintSequences, 0)))
	);

	while (true) {
		// Get info
		PlanPoint lastNode = planningPoints.back();

		// Get planning point
		PlanPoint newNode = _getNextPlanPoint(lastNode, distanceStep);

		// Validate total distance not exceeded
		if (newNode.distance >= distance - 1e-5) {
			break;
		}

		// Push planning point
		planningPoints.push_back(newNode);
	}
	PlanPoint finalNode = _getNextPlanPoint(planningPoints.back(), distance - planningPoints.back().distance);
	planningPoints.push_back(finalNode);

	return planningPoints;
}

std::vector<PlanPoint> TrajectoryPlanner::_backwardPass(double distanceStep) {
	std::vector<PlanPoint> planningPoints;
	planningPoints.push_back(
		PlanPoint(0, distance, endMotion)
		.constrain(getMinimumConstraint(getConstraintsAtDistance(constraintSequences, distance)))
	);

	while (true) {
		// Get info
		PlanPoint lastNode = planningPoints.back();

		// Get planning point
		PlanPoint newNode = _getNextPlanPoint(lastNode, -distanceStep);

		// Validate total distance not exceeded
		if (newNode.distance <= 1e-5) {
			break;
		}

		// Push planning point
		planningPoints.push_back(newNode);
	}

	// Flip derivatives
	for (PlanPoint &node : planningPoints) {
		for (int i = 1; i < (int) node.motion_dV_dT.size(); i++) {
			node.motion_dV_dT[i] *= -1;
		}
	}

	// Reverse
	std::reverse(planningPoints.begin(), planningPoints.end());

	return planningPoints;
}

TrajectoryPlanner &TrajectoryPlanner::calculateMotionProfile(int distanceResolution) {
	double distanceStep = distance / (double) distanceResolution;

	// Backward pass
	std::vector<PlanPoint> backward_planningPoints = _backwardPass(distanceStep);

	// Set constraints based on backward pass
	ConstraintSequence backward_constraintSequence({}, true);
	ConstraintSequence backward_track_constraintSequence({}, true);
	double maxTime = -backward_planningPoints[0].time_seconds;
	for (PlanPoint &point : backward_planningPoints) {
		// Center
		backward_constraintSequence.addConstraints({
			{point.distance, {fabs(point.motion_dV_dT[0])}}
		});

		// Track
		if (trackWidth > 0 && distanceToCurvature_function) {
			double curvature = distanceToCurvature_function(point.distance);
			double factor = (1 + fabs(curvature) * trackWidth / 2);
			backward_track_constraintSequence.addConstraints({
				{
					point.distance,
					{fabs(point.motion_dV_dT[0]) * factor}
				}
			});
		}

		// Fix time
		point.time_seconds += maxTime;
	}
	// profilePoints = backward_planningPoints;
	// return *this;
	// backward_track_constraintSequence = ConstraintSequence();

	constraintSequences.push_back(backward_constraintSequence);
	track_constraintSequences.push_back(backward_track_constraintSequence);

	// Forward pass
	std::vector<PlanPoint> forward_planningPoints = _forwardPass(distanceStep);

	constraintSequences.pop_back();
	track_constraintSequences.pop_back();

	// Store points
	profilePoints = forward_planningPoints;

	return *this;
}

double TrajectoryPlanner::getTotalTime() {
	return profilePoints.back().time_seconds;
}

std::pair<double, std::vector<double>> TrajectoryPlanner::getMotionAtTime(double time_seconds) {
	// Empty case
	if ((int) profilePoints.size() < 2) {
		return { 0, {0, 0} };
	}

	// Sanitize time
	time_seconds = aespa_lib::genutil::clamp(time_seconds, 0, getTotalTime());

	// Binary search for planning point
	int bs_l, bs_r, bs_m;
	int bs_result = -1;
	bs_l = 0;
	bs_r = (int) profilePoints.size() - 2;
	while (bs_l <= bs_r) {
		bs_m = bs_l + (bs_r - bs_l) / 2;
		PlanPoint node = profilePoints[bs_m];
		if (node.time_seconds <= time_seconds) {
			bs_result = bs_m;
			bs_l = bs_m + 1;
		} else {
			bs_r = bs_m - 1;
		}
	}
	if (bs_result == -1) bs_result = 0;

	// Get segment
	PlanPoint point1 = profilePoints[bs_result];
	PlanPoint point2 = profilePoints[bs_result + 1];

	// Lerp motion
	int motionDegree = fmin(
		(int) point1.motion_dV_dT.size(), (int) point2.motion_dV_dT.size()
	);
	PlanPoint result(-1, 0, std::vector<double>(motionDegree, 0));
	result.distance = aespa_lib::genutil::rangeMap(
		time_seconds,
		point1.time_seconds, point2.time_seconds,
		point1.distance, point2.distance
	);
	for (int i = 0; i < motionDegree; i++) {
		result.motion_dV_dT[i] = aespa_lib::genutil::rangeMap(
			time_seconds,
			point1.time_seconds, point2.time_seconds,
			point1.motion_dV_dT[i], point2.motion_dV_dT[i]
		);
	}

	// Multiply sign
	if (distance_sign == -1) {
		result.distance *= -1;
		for (double &value : result.motion_dV_dT) {
			value *= -1;
		}
	}

	// Return
	return { result.distance, result.motion_dV_dT };
}

}
}
}
