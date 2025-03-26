#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"
#include "Pas1-Lib/Planning/Splines/curve-sampler.h"
#include "Aespa-Lib/Winter-Utilities/general.h"
#include <stdio.h>
#include <fstream>
#include <vector>

namespace {
using namespace pas1_lib::planning::segments;
using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;
using pas1_lib::planning::trajectories::TrajectoryPlanner;

double maxVelocity = 3.628;
double maxAccel = maxVelocity * 1.5;
double trackWidth = 0.503937008;
// double trackWidth = 0;

std::vector<SplineCurve> splines;
std::vector<CurveSampler> splineSamplers;
std::vector<TrajectoryPlanner> splineTrajectoryPlans;
std::vector<bool> willReverse;

int pathIndex;

void clearSplines();
void pushNewSpline(pas1_lib::planning::splines::SplineCurve spline, bool reverse = false, double maxVel = maxVelocity);
void runFollowSpline();
}

namespace {
void clearSplines() {
	splines.clear();
	splineSamplers.clear();
	splineTrajectoryPlans.clear();
	willReverse.clear();
	pathIndex = 0;
}

void pushNewSpline(SplineCurve spline, bool reverse, double maxVel) {
	CurveSampler curveSampler = CurveSampler(spline)
		.calculateByResolution(spline.getTRange().second * 10);
	TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(
		curveSampler.getDistanceRange().second, trackWidth,
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
		// .addCenterConstraint_maxMotion({ maxVel, maxAccel })
		.addCenterConstraint_maxMotion({ maxVel, maxAccel, maxAccel * 2 })
		.addTrackConstraint_maxMotion({ maxVel, maxAccel })
		// .addCenterConstraint_maxMotion({maxVel, maxAccel, maxAccel * 0.5})
		.calculateMotionProfile();
	splines.push_back(spline);
	splineSamplers.push_back(curveSampler);
	splineTrajectoryPlans.push_back(splineTrajectoryPlan);
	willReverse.push_back(reverse);
}

void runFollowSpline() {
	aespa_lib::datas::Linegular lg = splines[pathIndex].getLinegularAt(0, willReverse[pathIndex]);
	SplineCurve spline = splines[pathIndex];
	CurveSampler curveSampler = splineSamplers[pathIndex];
	TrajectoryPlanner &motionProfile = splineTrajectoryPlans[pathIndex];
	bool isReversed = willReverse[pathIndex];

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
	for (int mt = -100; mt <= 1000 * motionProfile.getTotalTime() + 101; mt += 10) {
		double t = mt / 1000.0;
		std::pair<double, std::vector<double>> motion = motionProfile.getMotionAtTime(t);
		double distance = motion.first;
		int degree = motion.second.size();
		double velocity = motion.second[0];
		double acceleration = (degree >= 2) ? motion.second[1] : 0;
		double curvature = motionProfile.getCurvatureAtDistance(distance);
		double factor = curvature * trackWidth / 2;
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
		file_curvature << ", " << spline.getCurvatureAt(curveSampler.distanceToParam(distance));
		file_curvature << ", " << curvature;
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
	pushNewSpline(SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{{1.59, -0.42}, {1.52, 0.33}, {1.49, 0.81}, {0.48, 1}, {1.55, 1.02}, {2.51, 1}, {1.57, 1.28}, {1.53, 1.81}, {1.53, 2.79}}
		}));
	pushNewSpline(SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}, {1.54, 1.8},
		{2.06, 1.95}, {2.49, 1.34}, {1.54, 0.48}, {0.48, 0.05},
		}));
	pushNewSpline(SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{2.15, -0.38}, {0.98, 1}, {0.98, 5.02}, {3.02, 1}, {5.02, 5},
		{5.04, 1.02}, {3.95, -0.38}
		}));
	runFollowSpline();
	runFollowSpline();
	runFollowSpline();
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
