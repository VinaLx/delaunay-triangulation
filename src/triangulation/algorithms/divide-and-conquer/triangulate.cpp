#include "triangulation/algorithms/divide-and-conquer/triangulate.h"

#include "triangulation/algorithms/divide-and-conquer/convex-hull.h"
#include "triangulation/algorithms/divide-and-conquer/environment.h"
#include "triangulation/utility.h"

#include <algorithm>
#include <chrono>
#include <optional>
#include <stdio.h>
#include <tuple>

namespace triangulation {

using namespace divide_and_conquer;
using namespace std::chrono;

namespace {

void SortByAngle(std::vector<PointPtr> &ps, PointPtr pa, PointPtr pb) {
    Point2D v = pb->point - pa->point;
    std::sort(std::begin(ps), ps.end(), [&v, pa](PointPtr p1, PointPtr p2) {
        return VecCos(p1->point - pa->point, v) >
               VecCos(p2->point - pa->point, v);
    });
}

void PrintEdges(const std::vector<IdEdge> &edges) {
    LOGLN("----- Logging edges start -----");
    for (int i = 0; i < edges.size(); ++i) {
        fprintf(stderr, "%d %d\n", (int)edges[i].p1, (int)edges[i].p2);
    }
    LOGLN("----- Logging edges end -----");
}

} // namespace

struct DivideAndConquerImpl {
  public:
    DivideAndConquerImpl(Environment &env) : env_(env) {}

    void Go() {
        auto hull = Recurse(0, env_.PointSize());
        hull.Destruct();
    }

    ConvexHull Recurse(Index i, Index j) {
        assert(j - i >= 2);
        if (j - i == 2) {
            return BaseCase2Points(i);
        }
        if (j - i == 3) {
            return BaseCase3Points(i);
        }
        return DivideRecurse(i, (i + j) / 2, j);
    }

  private:
    ConvexHull BaseCase2Points(int64_t i) {
        PointPtr p1 = env_.GetPointByIndex(i);
        PointPtr p2 = env_.GetPointByIndex(i + 1);
        env_.AddEdge(p1, p2);
        DebugEdge();
        return ConvexHull::From2Points(p1, p2);
    }
    ConvexHull BaseCase3Points(int64_t i) {
        PointPtr p1 = env_.GetPointByIndex(i);
        PointPtr p2 = env_.GetPointByIndex(i + 1);
        PointPtr p3 = env_.GetPointByIndex(i + 2);
        env_.AddEdge(p1, p2);
        env_.AddEdge(p2, p3);
        env_.AddEdge(p1, p3);
        DebugEdge();
        return ConvexHull::From3Points(p1, p2, p3);
    }
    ConvexHull DivideRecurse(int64_t i, int64_t m, int64_t j) {
        ConvexHull left_hull = Recurse(i, m);
        ConvexHull right_hull = Recurse(m, j);
        LOGF("Merging of: %lld %lld %lld", i, m, j);
        auto [hull, bot, top] = ConvexHull::Merge(left_hull, right_hull);
        LOGF(
            "Bottom: %lld %lld, Top: %lld %lld", bot.p1->id, bot.p2->id,
            top.p1->id, top.p2->id);
        LogConvexHull(hull);
        bool add_edge_success = env_.AddEdge(bot);
        assert(add_edge_success);
        auto final_edge [[maybe_unused]] = MergeWithBaseEdge(bot);
        // assert(final_edge == top); // must it?
        return hull;
    }

    EdgeRef MergeWithBaseEdge(EdgeRef edge) {
        PointPtr left = edge.p1, right = edge.p2;
        PointPtr lc = GetLeftCandidate(left, right),
                 rc = GetRightCandidate(left, right);
        auto new_edge = DebateCandidates(left, right, lc, rc);
        if (not new_edge) {
            return edge;
        }
        env_.AddEdge(*new_edge);
        return MergeWithBaseEdge(*new_edge);
    }

    std::vector<PointPtr>
    GetCandidates(PointPtr pa, PointPtr pb, Orientation o) {
        const auto &links = env_.GetEdges(pa->id);
        std::vector<PointPtr> result;
        for (auto p : links) {
            if (ComputeOrientation(pa->point, pb->point, p->point) == o and
                p != pb) {
                result.push_back(p);
            }
        }
        SortByAngle(result, pa, pb);
        return result;
    }

    std::vector<PointPtr> GetLeftCandidates(PointPtr left, PointPtr right) {
        return GetCandidates(left, right, Orientation::kCounterClockwise);
    }
    std::vector<PointPtr> GetRightCandidates(PointPtr left, PointPtr right) {
        return GetCandidates(right, left, Orientation::kClockwise);
    }

    PointPtr SelectCandidate(
        const std::vector<PointPtr> &cs, PointPtr pa, PointPtr pb,
        Orientation o) {
        if (cs.empty()) return nullptr;
        for (int i = 0; i < cs.size() - 1; ++i) {
            PointPtr cur = cs[i], next = cs[i + 1];
            if (InCircleO(pa->point, pb->point, cur->point, next->point, o)) {
                bool remove_success = env_.RemoveEdge(pa, cur);
                assert(remove_success);
            } else {
                return cur;
            }
        }
        return cs.back();
    }

    PointPtr SelectLeftCandidate(
        const std::vector<PointPtr> &cs, PointPtr left, PointPtr right) {
        return SelectCandidate(cs, left, right, kCounterClockwise);
    }
    PointPtr SelectRightCandidate(
        const std::vector<PointPtr> &cs, PointPtr left, PointPtr right) {
        return SelectCandidate(cs, right, left, kClockwise);
    }

    PointPtr GetLeftCandidate(PointPtr left, PointPtr right) {
        auto candidates = GetLeftCandidates(left, right);
        return SelectLeftCandidate(candidates, left, right);
    }
    PointPtr GetRightCandidate(PointPtr left, PointPtr right) {
        auto candidates = GetRightCandidates(left, right);
        return SelectRightCandidate(candidates, left, right);
    }

    static std::optional<EdgeRef>
    DebateCandidates(PointPtr left, PointPtr right, PointPtr lc, PointPtr rc) {
        if (lc == nullptr and rc == nullptr) {
            return {};
        }
        if (rc == nullptr) {
            return EdgeRef{lc, right};
        }
        if (lc == nullptr) {
            return EdgeRef{left, rc};
        }
        if (not InCircle(left->point, right->point, lc->point, rc->point)) {
            return EdgeRef{lc, right};
        }
        if (not InCircle(left->point, right->point, rc->point, lc->point)) {
            return EdgeRef{left, rc};
        }
        assert(false); // this should never happen??
        return {};
    }

    void DebugEdge() {
#ifndef NDEBUG
        PrintEdges(env_.Edges());
#endif
    }

    void LogConvexHull(ConvexHull hull) {
#ifdef NDEBUG
        LOGLN("Convex Hull:");
        LOGF(
            "left-most: %lld, right-most: %lld", hull.left_most->Pid(),
            hull.right_most->Pid());
        hull.TraverseEdges(
            [](EdgeRef e) { LOGF("%lld %lld", e.p1->id, e.p2->id); });
#endif
    }

    Environment &env_;
};

void SortFromLeftToRight(std::vector<PointRef> &pts) {
    std::sort(begin(pts), end(pts), [](const PointRef &l, const PointRef &r) {
        return std::tie(l.point(0), l.point(1)) <
               std::tie(r.point(0), r.point(1));
    });
}

Triangulator::OutputEdges
DivideAndConquer::Triangulate(Triangulator::InputPoints pts) const {
    SortFromLeftToRight(pts);
    Environment env(pts);
    DivideAndConquerImpl driver(env);
    driver.Go();
    return env.Edges();
}

} // namespace triangulation