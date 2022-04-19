#include "../include/cubic_spline.hpp"

#include <algorithm>
#include <iostream> 

#define RC_TO_IDX(ROW,COL) (4 * (ROW)) + COL

double *solveTridiagonal(double *mat, int ptsLen);

std::vector<std::tuple<double,double>> 
cubicSplineInterp(std::vector<std::tuple<double,double>> &pts, double dist) {
    int ptsLen = pts.size();
    // http://fourier.eng.hmc.edu/e176/lectures/ch7/node6.html 
    // Create tridiagonal matrix, row major form
    // 4 columns and ptsLen rows
    // b1 c1 0    d1
    // a2 b2 c2   d2
    // a3 b3 c3   d3
    // .  .  .     .
    // .  .  cn-1  .
    // 0  an bn   d6
    double *mat = new double[ptsLen * 4];
    // constants in the spine matrix
    // top constants [2, 1, 0]
    mat[RC_TO_IDX(0,0)] = 2; 
    mat[RC_TO_IDX(0,1)] = 1; 
    mat[RC_TO_IDX(0,2)] = 0;
    // bottom constants [0, 1, 2]
    mat[RC_TO_IDX(ptsLen-1,0)] = 0; 
    mat[RC_TO_IDX(ptsLen-1,1)] = 1; 
    mat[RC_TO_IDX(ptsLen-1,2)] = 2;
    // load in the first and last d value
    double x1, x2, y1, y2, xn, yn, xnm1, ynm1;
    std::tie(x1, y1) = pts[0];
    std::tie(x2, y2) = pts[1];
    std::tie(xnm1, ynm1) = pts[ptsLen-2];
    std::tie(xn, yn) = pts[ptsLen-1];
    //mat[RC_TO_IDX(0,3)] = 6 * (((y2-y1)/(x2-x1)-dya)/h0);
    mat[RC_TO_IDX(0,3)] = 6 * ((y2-y1)/pow(x2-x1,2));
    //mat[RC_TO_IDX(ptsLen-1,3)] = 6 * ((dyb-(yn-ynm1)/h1)/h1);
    mat[RC_TO_IDX(ptsLen-1,3)] = 6 * -((yn-ynm1)/pow(xn-xnm1,2));
    // put in the rest of the tridiagonal values
    for(int i = 1; i < ptsLen-1; i++) {
        double xm1, ym1, x0, y0, x1, y1;
        std::tie(xm1, ym1) = pts[i-1];
        std::tie(x0, y0) = pts[i];
        std::tie(x1, y1) = pts[i+1];
        double h0 = x0 - xm1;
        double h1 = x1 - x0;
        double h2 = x1 - xm1;
        mat[RC_TO_IDX(i,0)] = h0/h2; // mu n
        mat[RC_TO_IDX(i,1)] = 2.0; 
        mat[RC_TO_IDX(i,2)] = h1/h2; // lambda n
        mat[RC_TO_IDX(i,3)] = 6.0 * (((y1-y0)/h1-(y0-ym1)/h0)/h2); // 2nd dd
    }
    double *m = solveTridiagonal(mat, ptsLen);
    delete[] mat;
    double x0, y0;
    std::tie(x0, y0) = pts[0];
    //std::cout << "X0 " << x0 << " XN " << xn << std::endl;
    int lenOut = std::ceil(std::abs(xn-x0)/dist); // TODO: still leaving out a point sometimes
    if(lenOut == 0) {
        lenOut = 1;
    }
    std::vector<std::tuple<double,double>> ptsOut(lenOut);
    int iter = 0;
    double x;
    //std::cout << "X0: " << x0 << std::endl;
    // calculate the cubic polynomials, interpolate at the given
    // distance between the points
    for(int i = 1; i < ptsLen; i++) {
        double xi, yi, xim1, yim1;
        std::tie(xim1, yim1) = pts[i-1];
        std::tie(xi, yi) = pts[i];
        double Mim1 = m[i-1];
        double Mi = m[i];
        double hi = xi - xim1;
        if(i == 1) {
            x = xim1;
        }
        while((x <= xi || i == ptsLen - 1) && iter < lenOut) {
            double cx = 
                (pow(xi - x, 3) / (6.0 * hi)) * Mim1 +
                (pow(x - xim1, 3) / (6.0 * hi)) * Mi +
                ((yim1/hi) - (Mim1*hi)/(6.0)) * (xi - x) +
                ((yi/hi) - ((Mi*hi)/6.0)) * (x - xim1);
            ptsOut[iter] = std::make_tuple(x, cx);
            iter++;
            x += dist;
        }
        //std::cout << "X" << i << ": " << xi << std::endl;
    }
    delete[] m;
    return ptsOut;
}

double *solveTridiagonal(double *mat, int rows) {
    // b1 c1 0    d1
    // a2 b2 c2   d2
    // a3 b3 c3   d3
    // .  .  .     .
    // .  .  cn-1  .
    // 0  an bn   d6
    // solve the tridiagonal matrix using specialized form of gaussian elimination
    // https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm 
    // initial pass
    // ci' = ci/bi
    mat[RC_TO_IDX(0,1)] = mat[RC_TO_IDX(0,1)] / mat[RC_TO_IDX(0,0)]; 
    // di' = di/bi
    mat[RC_TO_IDX(0,3)] = mat[RC_TO_IDX(0,3)] / mat[RC_TO_IDX(0,0)]; 
    for(int i = 1; i < rows; i++) {
        double ai, cim1, bi, di, dim1;
        cim1 = mat[RC_TO_IDX(i-1,2)];
        dim1 = mat[RC_TO_IDX(i-1,3)];
        di = mat[RC_TO_IDX(i,3)];
        if(i == rows-1) {
            // last row
            ai = mat[RC_TO_IDX(i,1)];
            bi = mat[RC_TO_IDX(i,2)];
        } else {
            // other rows
            ai = mat[RC_TO_IDX(i,0)];
            bi = mat[RC_TO_IDX(i,1)];
        }
        // to n - 1, not n
        if(i < rows - 1) {
            double ci = mat[RC_TO_IDX(i,2)];
            // ci' = ci/(bi-ai * c'i-1)
            mat[RC_TO_IDX(i,2)] = ci/(bi-(ai*cim1));
        }
        // di' = (di - ai d'i-1) / (bi - ai * c'i-1)
        mat[RC_TO_IDX(i,3)] = (di - ai * dim1) / (bi - ai * cim1);
    }
    // back substitution
    double *m = new double[rows]; // results of the spline matrix
    m[rows-1] = mat[RC_TO_IDX(rows-1,3)];
    for(int i = rows - 2; i != -1; i--) {
        double di, ci;
        di = mat[RC_TO_IDX(i,3)];
        if(i == 0) {
            ci = mat[RC_TO_IDX(i,1)];
        } else {
            ci = mat[RC_TO_IDX(i,2)];
        }
        // xi = d'i - c'i xi+1
        m[i] = di - ci * m[i+1];
    }
    return m;
}

std::vector<std::tuple<double,double,double>>
xyzCubicInterp(std::vector<std::tuple<double,double,double>> &pts, double dist, int maxPts) {
    // compute each value as a function of t and compute
    // the interpolation on each
    std::vector<std::tuple<double,double>> xOfT(pts.size());
    std::vector<std::tuple<double,double>> yOfT(pts.size());
    std::vector<std::tuple<double,double>> zOfT(pts.size());
    for(uint i = 0; i < pts.size(); i++) {
        double x, y, z, t;
        std::tie(x, y, z) = pts[i];
        t = (double)i / ((double)pts.size()-1);
        xOfT[i] = std::make_tuple(t, x);
        yOfT[i] = std::make_tuple(t, y);
        zOfT[i] = std::make_tuple(t, z);
    }
    // make sure the maximum points rule is respected
    double minDist = 1.0/(double)maxPts;
    if(minDist > dist) {
        dist = minDist;
    }
    auto xInterp = cubicSplineInterp(xOfT, dist);
    auto yInterp = cubicSplineInterp(yOfT, dist);
    auto zInterp = cubicSplineInterp(zOfT, dist);
    // solve for t and return the interpolation
    std::vector<std::tuple<double,double,double>> ret(xInterp.size());
    for(uint i = 0; i < xInterp.size(); i++) {
        double x, y, z, t;
        std::tie(t, x) = xInterp[i];
        std::tie(t, y) = yInterp[i];
        std::tie(t, z) = zInterp[i];
        ret[i] = std::make_tuple(x,y,z);
    }
    return ret;
}

void splineExamples() {
    /*std::vector<std::tuple<double,double>> a 
    {
        std::make_tuple(0.0, 0.0),
        std::make_tuple(0.78539,0.7079),
        std::make_tuple(1.5707,1),
        std::make_tuple(2.35619,0.7079),
        std::make_tuple(3.14159,0)
    };
    auto v = cubicSplineInterp(a, 0.1);
    for(uint i = 0; i < v.size(); i++) {
        double x, y;
        std::tie(x, y) = v[i];
        std::cout << x << " " << y << std::endl;
    }
    std::cout << std::endl;*/
    std::vector<std::tuple<double,double,double>> a 
        {
            std::make_tuple(0.0, 0.0, 0.0),
            std::make_tuple(0.6, 0.3, 0.8),
            std::make_tuple(1.0, 1.0, 1.0),
            std::make_tuple(1.2, 0.3, 0.8),
            std::make_tuple(1.5, 0.3, 0.8)
        };
    auto v = xyzCubicInterp(a, 0.1, 100);
    for(uint i = 0; i < v.size(); i++) {
        double x, y, z;
        std::tie(x, y, z) = v[i];
        std::cout << x << " " << y << " " << z << std::endl;
    }
    std::cout << std::endl;
}