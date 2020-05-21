#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.collections as pltc

from math import sqrt
from typing import List, Tuple, Dict, Set
from argparse import ArgumentParser


def read_data_from_stream(f):
    n = int(f.readline())
    points = []
    for i in range(n):
        line = f.readline().split(' ')
        points.append((float(line[0]), float(line[1])))
    edges = []
    for line in f:
        line = line.split(' ')
        x, y = int(line[0]), int(line[1])
        edges.append((x, y))
    return points, edges


def read_data(path: str):
    with open(path) as f:
        return read_data_from_stream(f)


def split_list(ls: List[Tuple[float, float]]):
    xs = []
    ys = []
    for x, y in ls:
        xs.append(x)
        ys.append(y)
    return (xs, ys)


def edges_to_segments(points, edges):
    segments = []
    for i, j in edges:
        segments.append((points[i], points[j]))
    return segments


def set_axis_equal():
    x1, x2, y1, y2 = plt.axis()
    low = min(x1, y1)
    high = max(x2, y2)
    plt.axis((low, high, low, high))
    plt.gca().set_aspect("equal")


def circumcircle(a, b, c) -> Tuple[Tuple[float, float], float]:
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    x = (ax ** 2 + ay ** 2) * (by - cy) + (bx ** 2 + by ** 2) * \
        (cy - ay) + (cx ** 2 + cy ** 2) * (ay - by)
    x = x / d
    y = (ax ** 2 + ay ** 2) * (cx - bx) + (bx ** 2 + by ** 2) * \
        (ax - cx) + (cx ** 2 + cy ** 2) * (bx - ax)
    y = y / d
    r = sqrt((x - ax) ** 2 + (y - ay) ** 2)
    return (x, y), r


def add_to_dict_set(d: Dict[int, Set[int]], i, j):
    if d.get(i) is None:
        d[i] = {j}
    else:
        d[i].add(j)


def build_graph(edges: List[Tuple[int, int]]) -> Dict[int, Set[int]]:
    graph = dict()
    for i, j in edges:
        add_to_dict_set(graph, i, j)
        add_to_dict_set(graph, j, i)
    return graph


def sort_tuple(xyz):
    ls = sorted(xyz)
    return (ls[0], ls[1], ls[2])


def triangles(edges: List[Tuple[int, int]]) -> Set[Tuple[int, int, int]]:
    graph = build_graph(edges)
    result_set = set()
    for x, y in edges:
        xe, ye = graph[x], graph[y]
        for z in xe:
            if z in ye:
                result_set.add(sort_tuple((x, y, z)))
    return result_set


def triangles_to_circles(points, tris):
    result = []
    for x, y, z in tris:
        result.append(circumcircle(points[x], points[y], points[z]))
    return result


def circumcircles_from_data(points, edges):
    tris = triangles(edges)
    return triangles_to_circles(points, tris)


def cmd_parser():
    parser = ArgumentParser("Displaying the triangulation result")
    parser.add_argument(
        "-c",
        "--circle",
        default=False,
        action="store_true",
        help="draw circumscribed circle for all triangles")
    parser.add_argument(
        "-nd",
        "--no-display",
        default=False,
        action="store_true",
        help="do not display the result directly")
    parser.add_argument(
        "-o",
        "--save",
        metavar="file",
        action="store",
        help="saving the image to destination")
    parser.add_argument(
        "-i",
        "--input",
        metavar="file",
        type=str,
        required=True,
        help="input file emitted by the triangulation program")
    return parser


def plot(points, edges, args):
    xs, ys = split_list(points)
    lines = pltc.LineCollection(
        edges_to_segments(
            points,
            edges),
        linewidths=[0.7])
    print(lines.get_linewidth())
    fig, ax = plt.subplots()
    ax.add_collection(lines)
    plt.scatter(xs, ys, s=[3], color="black")

    if args.circle:
        for c, r in circumcircles_from_data(points, edges):
            circle = plt.Circle(c, r, fill=False, color="gray", linewidth=0.6)
            ax.add_artist(circle)

    # for i, p in enumerate(points):
    #   ax.annotate(str(i), (p[0] + 0.7, p[1]))

    set_axis_equal()

    if args.save is not None:
        plt.savefig(args.save, dpi=400)
    if not args.no_display:
        plt.show()


if __name__ == '__main__':
    args = cmd_parser().parse_args()

    points, edges = read_data(args.input)
    plot(points, edges, args)
