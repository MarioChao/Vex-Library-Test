#include "Pas1-Lib/Planning/Profiles/spline-profile.h"
#include "Aespa-Lib/Winter-Utilities/general.h"
#include "Aespa-Lib/Karina-Data-Structures/named-storage.h"

#include <stdio.h>
#include <fstream>
#include <vector>

namespace {
using namespace pas1_lib::planning::segments;
using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;
using pas1_lib::planning::trajectories::TrajectoryPlanner;
using pas1_lib::planning::profiles::SplineProfile;

double maxVelocity = 3.628;
double maxAccel = maxVelocity * 1.5;
double trackWidth = 0.503937008;
// double trackWidth = 0;

aespa_lib::datas::NamedStorage<SplineProfile> splineStorage;

int pathIndex;

void clearSplines();
void pushNewSpline(std::string profileName, SplineCurve spline, bool reverse = false, double maxVel = maxVelocity);
void runFollowSpline(std::string profileName);
}

namespace {
void clearSplines() {
	splineStorage.clear();
	pathIndex = 0;
}

void pushNewSpline(std::string profileName, SplineCurve spline, bool reverse, double maxVel) {
	if (splineStorage.hasKey(profileName)) {
		printf("Profile '%s' already exists!\n", profileName.c_str());
		return;
	}
	CurveSampler curveSampler = CurveSampler(spline)
		.calculateByResolution(spline.getTRange().second * 10);
	TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(
		curveSampler.getDistanceRange().second, trackWidth,
		// 40 + (int) (curveSampler.getDistanceRange().second * 4)
		64
	)
		// .setCurvatureFunction(
		// 	[&](double d) -> double { return curveSampler.distanceToParam(d); },
		// 	[&](double t) -> double { return curveSampler.paramToDistance(t); },
		// 	[&](double t) -> double { return spline.getCurvatureAt(t); }
		// )
		.setCurvatureFunction([&](double d) -> double {
		return spline.getCurvatureAt(curveSampler.distanceToParam(d));
	})
		// .smoothenCurvature()
		.addCenterConstraint_maxMotion({ maxVel, maxAccel })
		.addTrackConstraint_maxMotion({ maxVel, maxAccel })
		// .addCenterConstraint_maxMotion({ maxVel, maxAccel, maxAccel * 5 })
		.calculateMotionProfile();
	splineStorage.store(profileName, SplineProfile(spline, curveSampler, splineTrajectoryPlan, reverse));
	// splines.push_back(spline);
	// splineSamplers.push_back(curveSampler);
	// splineTrajectoryPlans.push_back(splineTrajectoryPlan);
	// willReverse.push_back(reverse);
}

void runFollowSpline(std::string profileName) {
	SplineProfile *profile = splineStorage.getStored(profileName).get();
	SplineCurve spline = profile->spline;
	CurveSampler curveSampler = profile->curveSampler;
	TrajectoryPlanner &motionProfile = profile->trajectoryPlan;
	bool isReversed = profile->willReverse;
	aespa_lib::datas::Linegular lg = spline.getLinegularAt(0, isReversed);

	std::string filePrefix = std::string("dev-files/") + "path" + std::to_string(pathIndex);
	std::ofstream file_dis;
	std::ofstream file_vel;
	std::ofstream file_accel;
	std::ofstream file_curvature;
	file_dis.open(filePrefix + "-dis.csv");
	file_vel.open(filePrefix + "-vel.csv");
	file_accel.open(filePrefix + "-accel.csv");
	file_curvature.open(filePrefix + "-k.csv");
	file_dis << "time, distance\n";
	file_vel << "time, maxV, minV, velocity, right vel, left vel\n";
	file_accel << "time, maxA, minA, accel, right accel, left accel\n";
	file_curvature << "time, curvature, smoothed curvature\n";
	double prevLeftVelocity = 0, prevRightVelocity = 0, prevT;
	for (int mt = -100; mt <= 1000 * motionProfile.getTotalTime() + 101; mt += 5) {
		double t = mt / 1000.0;
		std::pair<double, std::vector<double>> motion = motionProfile.getMotionAtTime(t);
		double distance = motion.first;
		int degree = motion.second.size();
		double velocity = motion.second[0];
		double acceleration = (degree >= 2) ? motion.second[1] : 0;
		double curvature = spline.getCurvatureAt(curveSampler.distanceToParam(distance));
		double profile_curvature = motionProfile.getCurvatureAtDistance(distance);
		double factor = profile_curvature * trackWidth / 2;
		double leftVelocity = velocity * (1 - factor);
		double rightVelocity = velocity * (1 + factor);
		// double leftAccel = acceleration * (1 - factor);
		// double rightAccel = acceleration * (1 + factor);
		double leftAccel = (leftVelocity - prevLeftVelocity) / (t - prevT);
		double rightAccel = (rightVelocity - prevRightVelocity) / (t - prevT);
		prevT = t;
		prevLeftVelocity = leftVelocity;
		prevRightVelocity = rightVelocity;

		file_dis << t;
		file_dis << ", " << distance;
		file_dis << '\n';
		file_vel << t;
		file_vel << ", " << maxVelocity << ", " << -maxVelocity;
		file_vel << ", " << velocity << ", " << rightVelocity << ", " << leftVelocity;
		file_vel << '\n';
		file_accel << t;
		file_accel << ", " << maxAccel << ", " << -maxAccel;
		file_accel << ", " << acceleration << ", " << rightAccel << ", " << leftAccel;
		file_accel << '\n';
		file_curvature << t;
		file_curvature << ", " << curvature;
		file_curvature << ", " << profile_curvature;
		file_curvature << '\n';
	}
	file_dis.close();
	file_vel.close();
	file_accel.close();
	file_curvature.close();

	pathIndex++;
}
}


void testTrajectory() {
	clearSplines();
	pushNewSpline("big curvature 1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{{1.59, -0.42}, {1.52, 0.5}, {1.49, 0.81}, {0.48, 1}, {1.55, 1.02}, {2.51, 1}, {1.57, 1.28}, {1.53, 1.81}, {1.53, 2.79}}
			}));
	pushNewSpline(
		"love shape",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}, {1.54, 1.8},
		{2.06, 1.95}, {2.49, 1.34}, {1.54, 0.48}, {0.48, 0.05},
			})
			);
	pushNewSpline(
		"m shape",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{2.15, -0.38}, {0.98, 1}, {0.98, 5.02}, {3.02, 1}, {5.02, 5},
		{5.04, 1.02}, {3.95, -0.38}
			})
	);
	pushNewSpline(
		"field tour",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{-0.02, -0.07}, {1.36, 0.64}, {2.4, 1.55}, {0.97, 2.99}, {0.42, 4.03},
		{0.74, 5.28}, {2, 5.54}, {2.01, 3.98}, {3.02, 3}, {4.03, 4.02},
		{3.02, 4.85}, {3.02, 5.51}, {4.39, 5.49}, {4.67, 4.2}, {5.55, 3.07},
		{4.65, 1.77}, {5.49, 0.98}, {4.31, 0.42}, {4.02, 1.33}, {3.15, 1.37},
		{3, 0.48}, {3.02, -0.22},
			})
			);
	pushNewSpline("test",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
			{{2.54, 0.49}, {1.54, 0.47}, {0.47, 0.94}, {1.32, 1.59}, {1.54, 0.47}, {1.5, -0.46}}
			}));
	runFollowSpline("field tour");
	runFollowSpline("big curvature 1");
	runFollowSpline("love shape");
	// runFollowSpline("m shape");
	// runFollowSpline("test");
}

void testIntegral() {
	std::pair<double, std::vector<double>> integ1 = aespa_lib::genutil::integratePolynomial({ 1, -1 }, 1);
	std::pair<double, std::vector<double>> integ2 = aespa_lib::genutil::integratePolynomial({ 1, -1 }, -1);
	printf("%.3f ", integ1.first);
	for (double a : integ1.second) printf("%.3f ", a);
	printf("\n");
	printf("%.3f ", integ2.first);
	for (double a : integ2.second) printf("%.3f ", a);
	printf("\n");
}

int main() {
	testTrajectory();
	// testIntegral();
}
