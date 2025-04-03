#include "Test-Files/test-trajectory.h"

#include "Pas1-Lib/Planning/Splines/spline-curve.h"

#include <fstream>


namespace {
using namespace pas1_lib::planning::segments;
using pas1_lib::planning::splines::SplineCurve;

int fileIndex = 0;

void saveSplinePoints(SplineCurve *spline);
}

namespace {
void saveSplinePoints(SplineCurve *spline) {
	std::string filePrefix = std::string("dev-files/splines/") + "spline" + std::to_string(fileIndex);
	std::ofstream file_points;
	file_points.open(filePrefix + ".csv");
	file_points << "t,x,y\n";

	for (int mt = 0; mt <= 1000 * spline->getTRange().second + 0.1; mt += 5) {
		double t = mt / 1000.0;
		std::vector<double> posiiton = spline->getPositionAtT(t);

		file_points << t;
		file_points << ", " << posiiton[0] << ", " << posiiton[1];
		file_points << "\n";
	}

	fileIndex++;
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
	saveSplinePoints(&spline);
}
