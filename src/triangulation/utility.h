#pragma once

#include "Eigen/Dense"
#include "triangulation/types.h"

#include <stdio.h>
#include <stdlib.h>

namespace triangulation {

enum Orientation { kUnknown = 0, kClockwise = 1, kCounterClockwise = 2 };

inline Orientation
ComputeOrientation(const Point2D &p1, const Point2D &p2, const Point2D &p3) {
    auto v1 = p1 - p2, v2 = p1 - p3;
    double x1 = v1(0), y1 = v1(1), x2 = v2(0), y2 = v2(1);
    double d = x1 * y2 - y1 * x2;
    if (d < 0) return Orientation::kClockwise;
    if (d > 0) return Orientation::kCounterClockwise;
    return Orientation::kUnknown;
}

inline double VecCos(const Point2D &v1, const Point2D &v2) {
    return v1.dot(v2) / (v1.norm() * v2.norm());
}

inline double Square(double x) {
    return x * x;
}

inline bool InCircle(
    const Point2D &a, const Point2D &b, const Point2D &c, const Point2D &d) {
    double ax = a(0), ay = a(1), bx = b(0), by = b(1), cx = c(0), cy = c(1),
           dx = d(0), dy = d(1);
    double adx = ax - dx, ady = ay - dy, bdx = bx - dx, bdy = by - dy,
           cdx = cx - dx, cdy = cy - dy;
    Eigen::Matrix3d m;
    m << adx, ady, (Square(adx) + Square(ady)), bdx, bdy,
        (Square(bdx) + Square(bdy)), cdx, cdy, (Square(cdx) + Square(cdy));
    return m.determinant() > 0;
}

inline bool InCircleO(
    const Point2D &a, const Point2D &b, const Point2D &c, const Point2D &d,
    Orientation o) {
    if (o == kCounterClockwise) {
        return InCircle(a, b, c, d);
    }
    return InCircle(a, c, b, d);
}

#ifdef NDEBUG

#define LOGF(fmt, ...)
#define LOGLN(ln)

#else

#define LOGF(fmt, ...) fprintf(stderr, "[DEBUG] " fmt "\n", __VA_ARGS__)
#define LOGLN(ln) fputs("[DEBUG] " ln "\n", stderr);

#endif

} // namespace triangulation