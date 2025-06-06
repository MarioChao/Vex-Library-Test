#include "Test-Files/test-trajectory.h"

#include "Pas1-Lib/Planning/Profiles/spline-profile.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-constraint.h"
#include "Aespa-Lib/Winter-Utilities/general.h"
#include "Aespa-Lib/Karina-Data-Structures/named-storage.h"

#include <stdio.h>
#include <fstream>
#include <vector>
#include <cmath>


namespace {
using namespace pas1_lib::planning::segments;
using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;
using pas1_lib::planning::trajectories::ConstraintSequence;
using pas1_lib::planning::trajectories::TrajectoryPlanner;
using pas1_lib::planning::trajectories::TrajectoryConstraint;
using aespa_lib::geometry::Polygon2D;
using pas1_lib::planning::trajectories::PolygonRegionConstraint;
using pas1_lib::planning::profiles::SplineProfile;

double maxVelocity = 3.216;
double maxAccel = maxVelocity * 1.5;
double trackWidth = 0.503937008;
// double trackWidth = 0;

aespa_lib::datas::NamedStorage<SplineProfile> splineStorage;

int pathIndex = 0;

void saveTrajectoryGraph(TrajectoryPlanner *motionProfile, SplineProfile *splineProfile);

void clearSplines();
void pushNewSpline(std::string profileName, SplineCurve spline, bool reverse, std::vector<TrajectoryConstraint *> constraints, double maxVel = maxVelocity);
void pushNewSpline(std::string profileName, SplineCurve spline, bool reverse = false, double maxVel = maxVelocity);
void runFollowSpline(std::string profileName);

void runDriveTrajectory(double distance_tiles, std::vector<std::pair<double, double>> velocityConstraint_tiles_pct);
}

namespace {
void saveTrajectoryGraph(TrajectoryPlanner *motionProfile, SplineProfile *splineProfile) {
	if (splineProfile) {
		motionProfile = &splineProfile->trajectoryPlan;
	}

	std::string filePrefix = std::string("dev-files/paths/") + "path" + std::to_string(pathIndex);
	std::ofstream file_dis;
	std::ofstream file_vel;
	std::ofstream file_ang_vel;
	std::ofstream file_accel;
	std::ofstream file_curvature;
	file_dis.open(filePrefix + "-dis.csv");
	file_vel.open(filePrefix + "-vel.csv");
	file_ang_vel.open(filePrefix + "-ang_vel.csv");
	file_accel.open(filePrefix + "-accel.csv");
	file_curvature.open(filePrefix + "-k.csv");
	file_dis << "time, distance\n";
	file_vel << "time, maxV, minV, right vel, left vel, velocity\n";
	file_ang_vel << "time, ang vel\n";
	file_accel << "time, maxA, minA, right accel, left accel, accel\n";
	file_curvature << "time, curvature, smoothed curvature, factorR, factorL, zero\n";

	double prevT;
	double prevLeftVelocity = 0, prevRightVelocity = 0;
	double prevLeftAccel = 0, prevRightAccel = 0;
	double prevK = 0;
	for (int mt = -100; mt <= 1000 * motionProfile->getTotalTime() + 101; mt += 5) {
		double t = mt / 1000.0;
		std::pair<double, std::vector<double>> motion = motionProfile->getMotionAtTime(t);
		double motion_distance = motion.first;
		double abs_distance = std::fabs(motion_distance);
		int degree = motion.second.size();
		double velocity = motion.second[0];
		double acceleration = (degree >= 2) ? motion.second[1] : 0;
		double profile_curvature = motionProfile->getCurvatureAtDistance(abs_distance);
		double curvature = [&]() -> double {
			if (splineProfile) {
				double t = splineProfile->curveSampler.distanceToParam(abs_distance);
				return splineProfile->spline.getCurvatureAt(t);
			}
			return profile_curvature;
		}();
		double factor = profile_curvature * trackWidth / 2;
		double leftVelocity = velocity - std::fabs(velocity) * factor;
		double rightVelocity = velocity + std::fabs(velocity) * factor;
		// double leftAccel = acceleration * (1 - factor);
		// double rightAccel = acceleration * (1 + factor);
		double leftAccel = (leftVelocity - prevLeftVelocity) / (t - prevT);
		double rightAccel = (rightVelocity - prevRightVelocity) / (t - prevT);
		prevT = t;
		prevLeftVelocity = leftVelocity;
		prevRightVelocity = rightVelocity;
		prevLeftAccel = leftAccel;
		prevRightAccel = rightAccel;
		prevK = curvature;

		file_dis << t;
		file_dis << ", " << motion_distance;
		file_dis << '\n';
		file_vel << t;
		file_vel << ", " << maxVelocity << ", " << -maxVelocity;
		file_vel << ", " << rightVelocity << ", " << leftVelocity;
		file_vel << ", " << velocity;
		file_vel << '\n';
		file_ang_vel << t;
		file_ang_vel << ", " << velocity * profile_curvature;
		file_ang_vel << "\n";
		file_accel << t;
		file_accel << ", " << maxAccel << ", " << -maxAccel;
		file_accel << ", " << rightAccel << ", " << leftAccel;
		file_accel << ", " << acceleration;
		file_accel << '\n';
		file_curvature << t;
		file_curvature << ", " << curvature;
		file_curvature << ", " << profile_curvature;
		file_curvature << ", " << (1 + factor) << ", " << (1 - factor);
		file_curvature << ", " << 0;
		file_curvature << '\n';
	}
	file_dis.close();
	file_vel.close();
	file_ang_vel.close();
	file_accel.close();
	file_curvature.close();

	pathIndex++;
}

void clearSplines() {
	splineStorage.clear();
}

void pushNewSpline(std::string profileName, SplineCurve spline, bool reverse, std::vector<TrajectoryConstraint *> constraints, double maxVel) {
	if (splineStorage.hasKey(profileName)) {
		printf("Profile '%s' already exists!\n", profileName.c_str());
		return;
	}
	CurveSampler curveSampler = CurveSampler(spline).calculateByResolution(spline.getTRange().second * 10);
	double totalDistance = curveSampler.getDistanceRange().second;
	double distanceStep = aespa_lib::genutil::clamp(totalDistance / 64, 0.077, 0.5);
	// double distanceStep = totalDistance / 64;
	TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(totalDistance * (reverse ? -1 : 1), trackWidth, distanceStep)
		.setCurvatureFunction(
			[&](double d) -> double {
		return spline.getCurvatureAt(curveSampler.distanceToParam(d));
	},
			curveSampler.integerParamsToDistances()
		)
		.setSpline(&curveSampler)
		.maxSmoothCurvature()
		.addCenterConstraint_maxMotion({ maxVel, maxAccel })
		.addTrackConstraint_maxMotion({ maxVel, maxAccel * 0.85 })
		.addCenterConstraint_maxCentripetalAcceleration(maxAccel * 0.2)
		.addCenterTrajectoryConstraints(constraints)
		// .addCenterConstraint_maxMotion({ maxVel, maxAccel, maxAccel * 5 })
		.calculateMotionProfile();
	splineStorage.store(profileName, SplineProfile(spline, curveSampler, splineTrajectoryPlan, reverse));
}

void pushNewSpline(std::string profileName, SplineCurve spline, bool reverse, double maxVel) {
	pushNewSpline(profileName, spline, reverse, {}, maxVel);
}

void runFollowSpline(std::string profileName) {
	if (!splineStorage.hasKey(profileName)) {
		printf("Profile %s not stored!\n", profileName.c_str());
		return;
	}

	SplineProfile *profile = splineStorage.getStored(profileName).get();
	SplineCurve spline = profile->spline;
	CurveSampler curveSampler = profile->curveSampler;
	TrajectoryPlanner &motionProfile = profile->trajectoryPlan;
	bool isReversed = profile->willReverse;
	aespa_lib::datas::Linegular lg = spline.getLinegularAt(0, isReversed);

	saveTrajectoryGraph(&motionProfile, profile);
}

void runDriveTrajectory(double distance_tiles, std::vector<std::pair<double, double>> velocityConstraint_tiles_pct) {
	TrajectoryPlanner motionProfile(distance_tiles, trackWidth, 0.05);
	ConstraintSequence constraintSequence;
	for (int i = 0; i < (int) velocityConstraint_tiles_pct.size(); i++) {
		auto constraint = velocityConstraint_tiles_pct[i];
		constraintSequence.addConstraints({
			{
				constraint.first,
				{
					aespa_lib::genutil::clamp(constraint.second, 1, 100) / 100.0 * maxVelocity
				}
			}
		});
	}
	motionProfile.addCenterConstraintSequence(constraintSequence);
	motionProfile.addCenterConstraint_maxMotion({ maxVelocity, maxAccel });
	motionProfile.addTrackConstraint_maxMotion({ maxVelocity, maxAccel });
	motionProfile.calculateMotionProfile();

	saveTrajectoryGraph(&motionProfile, nullptr);
}
}


void testTrajectorySmall() {
	clearSplines();
	pushNewSpline("grab goal",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {2.76, 5.81}, {1.99, 4.98}, {1.15, 3.97}, {1.02, 2.17}, {1, 0.76} }
		), false,
		{
			new PolygonRegionConstraint(Polygon2D({{0.5, 3}, {1, 2.5}, {1.5, 3}, {1, 3.5}}), maxVelocity * 0.3)
		}
	);
	runFollowSpline("grab goal");
}

void testTrajectory() {
	clearSplines();
	pushNewSpline("180",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {1.5, -0.94}, {1.5, 0.5}, {1.0, 1.15}, {1.5, 1.73}, {2.0, 1.15}, {1.5, 0.5}, {1.5, -0.94} }
		), false
	);
	pushNewSpline("test",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {2.54, 0.49}, {1.54, 0.47}, {0.47, 0.94}, {1.32, 1.59}, {1.54, 0.47}, {1.5, -0.46} }
		), true
	);
	pushNewSpline("big curvature 1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {1.59, -0.42}, {1.52, 0.5}, {1.49, 0.81}, {0.48, 1}, {1.55, 1.02}, {2.51, 1}, {1.57, 1.28}, {1.53, 1.81}, {1.53, 2.79} }
		)
	);
	pushNewSpline(
		"love shape",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{
				{ 2.62, 0.09 }, { 1.52, 0.49 }, { 0.67, 1.35 }, { 1.03, 1.97 }, { 1.54, 1.8 },
				{ 2.06, 1.95 }, { 2.49, 1.34 }, { 1.54, 0.48 }, { 0.48, 0.05 }
			}
		)
	);
	pushNewSpline(
		"m shape",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{
				{2.15, -0.38}, {0.98, 1}, {0.98, 5.02}, {3.02, 1}, {5.02, 5},
				{5.04, 1.02}, {3.95, -0.38}
			}
		)
	);
	pushNewSpline(
		"field tour",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{
				{-0.02, -0.07}, {1.36, 0.64}, {2.4, 1.55}, {0.97, 2.99}, {0.42, 4.03},
				{0.74, 5.28}, {2, 5.54}, {2.01, 3.98}, {3.02, 3}, {4.03, 4.02},
				{3.02, 4.85}, {3.02, 5.51}, {4.39, 5.49}, {4.67, 4.2}, {5.55, 3.07},
				{4.65, 1.77}, {5.49, 0.98}, {4.31, 0.42}, {4.02, 1.33}, {3.15, 1.37},
				{3, 0.48}, {3.02, -0.22}
			}
		)
	);
	runFollowSpline("180");
	runFollowSpline("test");
	runFollowSpline("big curvature 1");
	runFollowSpline("love shape");
	runFollowSpline("m shape");
	// runFollowSpline("field tour");
}

void test1DTrajectory() {
	runDriveTrajectory(1, { {0, 75} });
	runDriveTrajectory(-1, { {0, 75} });
	runDriveTrajectory(-1, { {0, 20}, {0.5, 60} });
}
