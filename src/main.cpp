#include "Test-Files/test-trajectory.h"
#include "Test-Files/test-spline.h"
#include "Aespa-Lib/Winter-Utilities/general.h"
#include "Aespa-Lib/Winter-Utilities/units.h"

#include <stdio.h>
#include <fstream>
#include <vector>

namespace {
using namespace aespa_lib::units::literals;
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
	testSpline();
	// testTrajectory();
	// test1DTrajectory();
	// testIntegral();
}
