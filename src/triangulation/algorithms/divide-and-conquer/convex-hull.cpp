#include "triangulation/algorithms/divide-and-conquer/convex-hull.h"

#include "triangulation/utility.h"

namespace triangulation::divide_and_conquer {

using Node = ConvexHull::Node;

namespace {

std::pair<Node *, Node *> LeftAndRight(Node *n1, Node *n2, Node *n3) {
    Node *temp[3]{n1, n2, n3};
    std::sort(temp, temp + 3, [](Node *const l, Node *const r) {
        return l->X() < r->X();
    });
    return std::make_pair(temp[0], temp[2]);
}

void ArrangeThreeNodes(Node *n1, Node *n2, Node *n3) {
    if (ComputeOrientation(n1->PrimPoint(), n2->PrimPoint(), n3->PrimPoint()) ==
        Orientation::kCounterClockwise) {
        n1->SetNext(n2);
        n2->SetNext(n3);
        n3->SetNext(n1);
    } else {
        n1->SetNext(n3);
        n3->SetNext(n2);
        n2->SetNext(n1);
    }
}

Node *TraceNodeBackWhen(Node *n, Node *nref, Orientation o) {
    for (; ComputeOrientation(
               n->PrimPoint(), nref->PrimPoint(), n->prev->PrimPoint()) == o;
         n = n->prev) {
        // LOGF("%lld", n->Pid());
        continue;
    }
    // LOGF("%lld", n->Pid());
    return n;
}

Node *TraceNodeForwardWhen(Node *n, Node *nref, Orientation o) {
    for (; ComputeOrientation(
               n->PrimPoint(), nref->PrimPoint(), n->next->PrimPoint()) == o;
         n = n->next) {
        continue;
    }
    return n;
}

void ReleaseLinkBetween(Node *back, Node *front) {
    if (back->next == front) {
        return;
    }
    front->prev->next = nullptr;
    front->prev = nullptr;
    Node* pending = back->next;
    back->next = nullptr;
    Node::ReleaseForward(pending);
}

std::pair<Node *, Node *> FindBottomEdge(Node *left, Node *right) {
    bool left_change = false, right_change = false;
    do {
        Node *new_left = TraceNodeBackWhen(left, right, kClockwise);
        left_change = new_left != left;
        left = new_left;

        Node *new_right = TraceNodeForwardWhen(right, left, kCounterClockwise);
        right_change = new_right != right;
        right = new_right;
    } while (left_change or right_change);

    return std::make_pair(left, right);
}

std::pair<Node *, Node *> FindTopEdge(Node *left, Node *right) {
    auto [r, l] = FindBottomEdge(right, left);
    return std::make_pair(l, r);
}

} // namespace

ConvexHull ConvexHull::From2Points(PointPtr p1, PointPtr p2) {
    Node *n1 = Node::FromPoint(p1);
    Node *n2 = Node::FromPoint(p2);
    n1->SetNext(n2);
    n2->SetNext(n1);
    if (p1->X() < p2->X()) {
        return ConvexHull{n1, n2};
    }
    return ConvexHull{n1, n2};
}

ConvexHull ConvexHull::From3Points(PointPtr p1, PointPtr p2, PointPtr p3) {
    Node *n1 = Node::FromPoint(p1);
    Node *n2 = Node::FromPoint(p2);
    Node *n3 = Node::FromPoint(p3);
    ArrangeThreeNodes(n1, n2, n3);
    auto [left, right] = LeftAndRight(n1, n2, n3);
    return ConvexHull{left, right};
}

std::tuple<ConvexHull, EdgeRef, EdgeRef>
ConvexHull::Merge(ConvexHull &left, ConvexHull &right) {
    auto [bot_left, bot_right] =
        FindBottomEdge(left.right_most, right.left_most);
    auto [top_left, top_right] = FindTopEdge(left.right_most, right.left_most);
    ConvexHull result = {left.left_most, right.right_most};
    ReleaseLinkBetween(bot_left, top_left);
    ReleaseLinkBetween(top_right, bot_right);
    bot_left->SetNext(bot_right);
    top_right->SetNext(top_left);
    left.Invalidate();
    right.Invalidate();
    return std::make_tuple(
        result, EdgeRef{bot_left->point, bot_right->point},
        EdgeRef{top_left->point, top_right->point});
}

} // namespace triangulation::divide_and_conquer