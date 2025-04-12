#include "Test-Files/test-polygon.h"

#include "Aespa-Lib/Giselle-Geometry/polygon-2d.hpp"

#include <fstream>


namespace {
using aespa_lib::geometry::Polygon2D;
using aespa_lib::geometry::Vector2D;

int fileIndex = 0;

void savePolygon(Polygon2D polygon);
}

namespace {
void savePolygon(Polygon2D polygon) {
	std::string filePrefix = std::string("dev-files/polygons/") + "polygon" + std::to_string(fileIndex);
	std::ofstream file_polygon;
	file_polygon.open(filePrefix + "-poly.csv");
	file_polygon << "poly_x,poly_y\n";
	for (Vector2D &point : polygon.ccwPoints) {
		file_polygon << point.x << ", " << point.y;
		file_polygon << "\n";
	}
	file_polygon.close();

	std::ofstream file_points;
	file_points.open(filePrefix + "-points.csv");
	file_points << "point_x,point_y,winding_number,is_in\n";
	for (double point_x = -10; point_x <= 10; point_x += 0.2) {
		for (double point_y = -10; point_y <= 10; point_y += 0.2) {
			Vector2D point(point_x, point_y);
			file_points << point_x << ", " << point_y;
			file_points << ", " << polygon.getWindingNumber(point);
			file_points << ", " << polygon.containsPoint(point);
			file_points << "\n";
		}
	}
	file_points.close();

	fileIndex++;
}
}


void testPolygon() {
	savePolygon(Polygon2D({
		{0, 0}, {4, 0}, {2, 5}
	}));
	savePolygon(Polygon2D({
		{0, -5}, {1, -4}, {2, -3}, {3, -2}, {4, 1}, {4, 2}, {3, 4}, {2, 5}, {1, 4}, {0, 2},
		{-1, 4}, {-2, 5}, {-3, 4}, {-4, 2}, {-4, 1}, {-3, -2}, {-2, -3}, {-1, -4}
	}));
	savePolygon(Polygon2D({
		{-8, -1}, {-2, -8}, {1, 2}, {3, -8}, {6, -3}, {8, 6},
		{6, 8}, {3, 6}, {3, 4}, {4, 6}, {5, 2}, {4, -1}, {2, 6}, {-1, 3},
		{-4, -5}, {-6, 1}
	}));
}
