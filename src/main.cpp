#include "Test-Files/test-trajectory.h"
#include "Test-Files/test-spline.h"
#include "Test-Files/test-polygon.h"
#include "Aespa-Lib/Giselle-Geometry/vector-2d.hpp"
#include "Aespa-Lib/Winter-Utilities/general.h"
#include "Aespa-Lib/Winter-Utilities/units.h"

#include <stdio.h>
#include <fstream>
#include <vector>

namespace {
using namespace aespa_lib::units::literals;
using aespa_lib::geometry::Vector2D;
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
	printf("%.10f\n", (-2000.0_in + 200_in).m());
	printf("%.10f\n", (-90_polarDeg).polarRad());
	printf("Cross: %.10f\n", Vector2D(0, 2).cross_scalar(Vector2D(2, 0)));
	printf("Dot: %.10f\n", Vector2D(0, 2).dot(Vector2D(2, 0)));
	printf("Angle: %.10f\n", Vector2D(2, 0).angleFrom(Vector2D(0, 2)).polarDeg());
	Vector2D vectorNeg = -Vector2D(0, 2);
	Vector2D vectorAdd = Vector2D(0, 2) + Vector2D(2, 0);
	Vector2D vectorMinus = Vector2D(0, 2) - Vector2D(2, 0);
	printf("Neg: %.10f %.10f\n", vectorNeg.x, vectorNeg.y);
	printf("Add: %.10f %.10f\n", vectorAdd.x, vectorAdd.y);
	printf("Minus: %.10f %.10f\n", vectorMinus.x, vectorMinus.y);
	// testSpline();
	// testTrajectorySmall();
	// testTrajectory();
	// test1DTrajectory();
	// testIntegral();
	// testPolygon();
}
