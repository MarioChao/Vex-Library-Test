#pragma once

#include "Pas1-Lib/Planning/Trajectories/constraint.h"
#include "Pas1-Lib/Planning/Trajectories/curvature.h"

#include <algorithm>
#include <vector>
#include <functional>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Plan Point ----------

struct PlanPoint {
	PlanPoint(double time_seconds, double distance, std::vector<double> motion_dV_dT);

	PlanPoint &constrain(Constraint constraint);
	PlanPoint &maximizeLastDegree(Constraint constraint);
	PlanPoint &maximizeNthDegree(
		Constraint constraint, int dV_dT_degree,
		Constraint target_rawConstraint, bool maximizeLowerDegrees = false
	);


	double time_seconds;
	double distance;
	std::vector<double> motion_dV_dT;
};

double getTimeStepFromDistanceStep(PlanPoint node, double distanceStep);
ConstraintSequence planPoints_to_rawConstraintSequence(std::vector<PlanPoint> nodes);


// ---------- Trajectory Planner ----------

class TrajectoryPlanner {
public:
	// Constructor; unit names are just for establishing consistency.
	TrajectoryPlanner(
		double distance_inches,
		double trackWidth_inches,
		double planPoint_distanceStep,
		std::vector<double> startMotion_dInches_dSec,
		std::vector<double> endMotion_dInches_dSec
	);
	TrajectoryPlanner(
		double distance_inches, double trackWidth_inches, double planPoint_distanceStep
	);
	TrajectoryPlanner(double distance_inches, double trackWidth_inches);
	TrajectoryPlanner(double distance_inches);
	TrajectoryPlanner();

	TrajectoryPlanner &setCurvatureFunction(
		std::function<double(double)> distanceToCurvature_function,
		std::vector<double> specificDistances = {}
	);

	TrajectoryPlanner &maxSmoothCurvature(double epsilon = 1e10);
	double getCurvatureAtDistance(double distance);

	TrajectoryPlanner &addCenterConstraintSequence(ConstraintSequence constraints);
	TrajectoryPlanner &addTrackConstraintSequence(ConstraintSequence constraints);
	TrajectoryPlanner &addCenterConstraints(std::vector<std::pair<double, std::vector<double>>> constraints);
	TrajectoryPlanner &addTrackConstraints(std::vector<std::pair<double, std::vector<double>>> constraints);

	// For now, only use up to dA/dT
	TrajectoryPlanner &addCenterConstraint_maxMotion(std::vector<double> maxMotion_dV_dT);
	// For now, only use up to dA/dT
	TrajectoryPlanner &addTrackConstraint_maxMotion(std::vector<double> maxMotion_dV_dT);

	PlanPoint _getNextPlanPoint(
		PlanPoint originalNode,
		double distanceStep,
		int dV_dT_degree
	);
	std::vector<PlanPoint> _forwardPass(int dV_dT_degree);
	std::vector<PlanPoint> _backwardPass(int dV_dT_degree);
	std::pair<ConstraintSequence, ConstraintSequence> constraintSequencesFromPlanPoints(
		std::vector<PlanPoint> nodes
	);
	TrajectoryPlanner &calculateMotionProfile();

	double getTotalTime();

	/// @param time_seconds Time in seconds
	/// @return {distance, motion_dV_dT}
	std::pair<double, std::vector<double>> getMotionAtTime(double time_seconds);

private:
	double totalDistance;
	double trackWidth;
	int distance_sign;

	double planPoint_distanceStep;

	std::vector<double> startMotion;
	std::vector<double> endMotion;

	CurvatureSequence curvatureSequence;

	std::vector<ConstraintSequence> center_constraintSequences;
	std::vector<ConstraintSequence> track_constraintSequences;

	std::vector<double> planPoint_distances;
	ConstraintSequence planPoint_rawSequence;
	std::vector<PlanPoint> profilePoints;
};


}
}
}
