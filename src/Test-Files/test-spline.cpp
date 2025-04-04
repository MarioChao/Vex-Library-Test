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

void sampleSpline(SplineCurve *spline);
}

namespace {
void saveCurvePoints(CurveSampler *curveSampler) {
	SplineCurve *spline = &curveSampler->getSpline();
	std::string filePrefix = std::string("dev-files/splines/") + "spline" + std::to_string(fileIndex);
	std::ofstream file_points;
	file_points.open(filePrefix + ".csv");
	file_points << "t,x,y\n";

	// for (int mt = 0; mt <= 1000 * spline->getTRange().second + 0.1; mt += 50) {
	// 	double t = mt / 1000.0;
	for (int md = 0; md <= 1000 * curveSampler->getDistanceRange().second + 0.1; md += 50) {
		double d = md / 1000.0;
		double t = curveSampler->distanceToParam(d);
		std::vector<double> positon = spline->getPositionAtT(t);

		file_points << t;
		file_points << ", " << positon[0] << ", " << positon[1];
		file_points << "\n";
	}

	fileIndex++;
}

void sampleSpline(SplineCurve *spline) {
	CurveSampler curveSampler = CurveSampler(*spline)
		.calculateByResolution(spline->getTRange().second * 10);
	printf("T: %.3f, D: %.7f\n", spline->getTRange().second, curveSampler.getDistanceRange().second);
	saveCurvePoints(&curveSampler);
}
}


void testSpline() {
	// SplineCurve spline = SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
	// 	{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}, {1.54, 1.8},
	// 	{2.06, 1.95}, {2.49, 1.34}, {1.54, 0.48}, {0.48, 0.05},
	// 		});
	SplineCurve spline = SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
		{{0.72, 0.23}, {0.02, 3.25}, {2.95, 5.45}, {3.06, 5.45}, {6.01, 3.19}, {5.34, 0.16}}
			});
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
	sampleSpline(&spline);
}
