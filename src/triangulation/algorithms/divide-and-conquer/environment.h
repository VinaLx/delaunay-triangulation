#pragma once

#include "triangulation/types.h"
#include "triangulation/utility.h"

#include <stdio.h>
#include <vector>

namespace triangulation {

namespace divide_and_conquer {

struct Environment {
  public:
    Environment(const std::vector<PointRef> &pts)
        : pts_(pts), edges_(pts.size()) {}

    int64_t PointSize() const {
        return pts_.size();
    }

    PointPtr GetPointByIndex(int64_t i) const {
        return &pts_[i];
    }

    std::vector<const PointRef *> &GetEdges(Index i) {
        return edges_.at(i);
    }

    bool AddEdge(PointPtr p1, PointPtr p2) {
        LOGF("Adding edge %lld, %lld", p1->id, p2->id);
        bool ij = AddEdgeImpl(p1->id, p2);
        bool ji = AddEdgeImpl(p2->id, p1);
        assert(ij == ji);
        return ij and ji;
    }

    bool AddEdge(EdgeRef edge) {
        return AddEdge(edge.p1, edge.p2);
    }

    bool RemoveEdge(PointPtr p1, PointPtr p2) {
        LOGF("Removing edge %lld, %lld\n", p1->id, p2->id);
        bool ij = RemoveEdgeImpl(p1->id, p2);
        bool ji = RemoveEdgeImpl(p2->id, p1);
        assert(ij == ji);
        return ij and ji;
    }

    std::vector<IdEdge> Edges() const {
        std::vector<IdEdge> result;
        result.reserve(edges_.size() / 2 + 1);
        for (int i = 0; i < edges_.size(); ++i) {
            for (const PointRef *p2 : edges_[i]) {
                if (i < p2->id) {
                    result.push_back(IdEdge{i, p2->id});
                }
            }
        }
        return result;
    }

  private:
    bool AddEdgeImpl(Index i, const PointRef *p) {
        for (const PointRef *point : edges_[i]) {
            if (point == p) return false;
        }
        edges_.at(i).push_back(p);
        return true;
    }
    bool RemoveEdgeImpl(Index i, const PointRef *j) {
        for (auto it = begin(edges_[i]); it != end(edges_[i]); ++it) {
            if (*it == j) {
                edges_.at(i).erase(it);
                return true;
            }
        }
        return false;
    }

    const std::vector<PointRef> &pts_;
    std::vector<std::vector<const PointRef *>> edges_;
};

} // namespace divide_and_conquer

} // namespace triangulation