#include "Test-Files/test-trajectory.h"

#include "Pas1-Lib/Planning/Splines/spline-curve.h"
#include "Pas1-Lib/Planning/Splines/curve-sampler.h"

#include <fstream>


namespace {
using namespace pas1_lib::planning::segments;
using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;

int fileIndex = 0;

void saveCurvePoints(CurveSampler *curveSampler);

void sampleSpline(SplineCurve spline);
}

namespace {
void saveCurvePoints(CurveSampler *curveSampler) {
	SplineCurve *spline = &curveSampler->getSpline();
	std::string filePrefix = std::string("dev-files/splines/") + "spline" + std::to_string(fileIndex);
	std::ofstream file_points;
	file_points.open(filePrefix + ".csv");
	file_points << "t,x,y,s\n";

	double totalDistance = curveSampler->getDistanceRange().second;

	// for (int mt = 0; mt <= 1000 * spline->getTRange().second + 0.1; mt += 50) {
	// 	double t = mt / 1000.0;
	for (int md = 0; md <= 1000 * totalDistance + 0.1; md += 50) {
		double d = md / 1000.0;
		double t = curveSampler->distanceToParam(d);
		std::vector<double> positon = spline->getPositionAtT(t);

		file_points << t;
		file_points << ", " << positon[0] << ", " << positon[1];
		file_points << ", " << totalDistance - d;
		file_points << "\n";
	}
	file_points.close();

	fileIndex++;
}

void sampleSpline(SplineCurve spline) {
	CurveSampler curveSampler = CurveSampler(spline)
		.calculateByResolution(spline.getTRange().second * 10);
	printf("T: %.3f, D: %.7f\n", spline.getTRange().second, curveSampler.getDistanceRange().second);
	saveCurvePoints(&curveSampler);
}
}


void testSpline() {
	sampleSpline(
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
			{{3.06, 2.27}, {2.45, 4.2}, {1.86, 5.06}, {1.26, 4.19}, {1, 2.36}, {0.66, 0.09}}
			})
			);
	sampleSpline(
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{-0.02, -0.07}, {1.36, 0.64}, {2.4, 1.55}, {0.97, 2.99}, {0.42, 4.03},
		{0.74, 5.28}, {2, 5.54}, {2.01, 3.98}, {3.02, 3}, {4.03, 4.02},
		{3.02, 4.85}, {3.02, 5.51}, {4.39, 5.49}, {4.67, 4.2}, {5.55, 3.07},
		{4.65, 1.77}, {5.49, 0.98}, {4.31, 0.42}, {4.02, 1.33}, {3.15, 1.37},
		{3, 0.48}, {3.02, -0.22},
			})
			);
	sampleSpline(
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}, {1.54, 1.8},
		{2.06, 1.95}, {2.49, 1.34}, {1.54, 0.48}, {0.48, 0.05},
			})
			);
	// SplineCurve spline = SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
	// 	{{0.72, 0.23}, {0.02, 3.25}, {2.95, 5.45}, {3.06, 5.45}, {6.01, 3.19}, {5.34, 0.16}}
	// 		});
	// SplineCurve spline = SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
	// 	{0, 0}, {0, 3}, {3, 0}, {3, 3},
	// 	{3, 6}, {6, 3}, {6, 6}
	// 		});
	// SplineCurve spline = SplineCurve({
	// 	std::shared_ptr<CubicSplineSegment>(new CubicSplineSegment(
	// 		Cubic_Bezier, {{0, 0}, {0, 3}, {3, 0}, {3, 3}}
	// 	)),
	// 	std::shared_ptr<CubicSplineSegment>(new CubicSplineSegment(
	// 		Cubic_Bezier, {{3, 3}, {3, 6}, {6, 3}, {6, 6}}
	// 	))
	// });
	// SplineCurve spline = SplineCurve({
	// 	std::shared_ptr<CubicSplineSegment>(new CubicSplineSegment(
	// 		Cubic_Hermite, {{0, 0}, {3, 3}, {0, 3}, {0, 3}}
	// 	)),
	// 	std::shared_ptr<CubicSplineSegment>(new CubicSplineSegment(
	// 		Cubic_Hermite, {{3, 3}, {6, 6}, {0, 3}, {0, 3}}
	// 	))
	// });
}
