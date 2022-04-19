#include <math.h>
#include <vector>
#include <tuple>

#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

// interpolates x, y points using cubic polynomials
// point inputs must be in sorted order by their x axis
std::vector<std::tuple<double,double>> 
cubicSplineInterp(std::vector<std::tuple<double,double>> &pts, double dist);

// interpolates x, y, z components using cubic polynomials
// point inputs must be ordered in the order of traversal
// dist is the time distance between each point (parametric component)
// and maxPts is the maximum interpolated points to choose
// t ranges from 0 to 1
std::vector<std::tuple<double,double,double>>
xyzCubicInterp(std::vector<std::tuple<double,double,double>> &pts, double dist, int maxPts);

void
splineExamples();

#endif