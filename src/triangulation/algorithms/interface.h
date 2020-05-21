#pragma once

#include "triangulation/types.h"

namespace triangulation {

struct Triangulator {
    using InputPoints = std::vector<PointRef>;
    using OutputEdges = std::vector<IdEdge>;

    virtual OutputEdges Triangulate(InputPoints points) const = 0;

    virtual ~Triangulator() = default;
};

} // namespace triangulation