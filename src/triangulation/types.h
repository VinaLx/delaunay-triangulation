#pragma once

#include "Eigen/Dense"

#include <stdint.h>
#include <tuple>
#include <vector>

namespace triangulation {

using Point2D = Eigen::Matrix<double, 2, 1>;

using Index = std::int64_t;

struct PointRef {
    Point2D point;
    Index id;

    double X() const {
        return point(0);
    }
    double Y() const {
        return point(1);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PointPtr = const PointRef*;

struct EdgeRef {
    PointPtr p1;
    PointPtr p2;
};

struct IdEdge {
    Index p1, p2;
};

inline bool operator==(const EdgeRef &e1, const EdgeRef &e2) {
    return std::tie(e1.p1, e1.p2) == std::tie(e2.p1, e2.p2) or
           std::tie(e1.p1, e1.p2) == std::tie(e2.p2, e2.p1);
}

inline bool operator!=(const EdgeRef& e1, const EdgeRef& e2) {
    return not (e1 == e2);
}

inline std::vector<PointRef>
TagPointWithIndex(const std::vector<Point2D> &pts) {
    std::vector<PointRef> result;
    result.reserve(pts.size());
    for (int i = 0; i < pts.size(); ++i) {
        result.push_back(PointRef{pts[i], i});
    }
    return result;
}

} // namespace triangulation
