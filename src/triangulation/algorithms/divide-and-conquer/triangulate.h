#pragma once

#include "triangulation/algorithms/interface.h"
#include "triangulation/types.h"

namespace triangulation {

struct DivideAndConquer : Triangulator {
    OutputEdges Triangulate(InputPoints points) const override;
};

} // namespace triangulation