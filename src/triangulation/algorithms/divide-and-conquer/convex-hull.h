#pragma once

#include "triangulation/types.h"
#include "triangulation/utility.h"

#include <memory>
#include <tuple>

namespace triangulation {

namespace divide_and_conquer {

struct ConvexHull {
    struct Node {
        Node(const PointRef *point)
            : point(point), prev(nullptr), next(nullptr) {}

        PointPtr point;
        Node *prev; // first node (point) going clockwise
        Node *next; // first node (point) going counter-clockwise

        double X() const {
            return point->X();
        }
        double Y() const {
            return point->Y();
        }
        Index Pid() const {
            return point->id;
        }
        const Point2D &PrimPoint() const {
            return point->point;
        }

        void SetNext(Node *that) {
            this->SetNextImpl(that);
            that->SetPrevImpl(this);
        }

        static void ReleaseForward(Node *n) {
            assert(n != nullptr);
            auto start = n;
            auto cur = start;
            do {
                auto next = cur->next;
                LOGF("release %lld in hull", cur->Pid());
                delete cur;
                cur = next;
            } while (cur != nullptr and cur != start);
        }
        static void ReleaseBackward(Node *n) {
            assert(n != nullptr);
            auto start = n;
            auto cur = start;
            do {
                auto prev = cur->prev;
                delete cur;
                cur = prev;
            } while (cur != nullptr and cur != start);
        }
        static Node *FromPoint(PointPtr p) {
            return new Node(p);
        }

      private:
        void SetNextImpl(Node *new_next) {
            if (next != nullptr) {
                next->prev = nullptr;
            }
            next = new_next;
        }
        void SetPrevImpl(Node *new_prev) {
            if (prev != nullptr) {
                prev->next = nullptr;
            }
            prev = new_prev;
        }
    };

    static ConvexHull From2Points(PointPtr p1, PointPtr p2);

    static ConvexHull From3Points(PointPtr p1, PointPtr p2, PointPtr p3);

    static std::tuple<ConvexHull, EdgeRef, EdgeRef>
    Merge(ConvexHull &left, ConvexHull &right);

    bool Valid() const {
        return left_most != nullptr and right_most != nullptr;
    }

    void Destruct() {
        if (Valid()) {
            Node::ReleaseForward(left_most);
        }
    }

    template <typename F>
    void TraverseEdges(F f) {
        if (not Valid()) return;
        Node* start = left_most;
        Node* next = start->next;
        do {
            f(EdgeRef{start->point, next->point});
            start = next;
            next = next->next;
        } while (start != left_most);
    }

    Node *left_most;
    Node *right_most;

  private:
    void Invalidate() {
        left_most = nullptr;
        right_most = nullptr;
    }
};

} // namespace divide_and_conquer

} // namespace triangulation