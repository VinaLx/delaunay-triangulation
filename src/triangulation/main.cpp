#include "triangulation/algorithms/divide-and-conquer/triangulate.h"
#include "triangulation/algorithms/interface.h"
#include "triangulation/types.h"
#include "triangulation/utility.h"

#include <chrono>
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace triangulation;
using namespace std::string_view_literals;
namespace chrono = std::chrono;

void LogPoints(const std::vector<Point2D> &pts) {
#ifndef NDEBUG
    LOGLN("---- Logging input points start -----");
    fprintf(stderr, "%d\n", (int)pts.size());
    for (int i = 0; i < pts.size(); ++i) {
        fprintf(stderr, "%.3f %.3f\n", pts[i](0), pts[i](1));
    }
    LOGLN("----- Logging input points end -----");
#endif
}

void WritePointsToStream(const std::vector<Point2D> &pts, FILE *f) {
    fprintf(f, "%d\n", (int)pts.size());
    for (int i = 0; i < pts.size(); ++i) {
        fprintf(f, "%.3f %.3f\n", pts[i](0), pts[i](1));
    }
}

void WriteEdgesToStream(const std::vector<IdEdge> &edges, FILE *f) {
    for (int i = 0; i < edges.size(); ++i) {
        fprintf(f, "%d %d\n", (int)edges[i].p1, (int)edges[i].p2);
    }
}

void WriteResultToStream(
    const std::vector<Point2D> &pts, const std::vector<IdEdge> &edges,
    FILE *f) {
    WritePointsToStream(pts, f);
    WriteEdgesToStream(edges, f);
}

void PrintResult(
    const std::vector<Point2D> &pts, const std::vector<IdEdge> &edges) {
    WriteResultToStream(pts, edges, stdin);
}

void WriteResultToFile(
    const std::vector<Point2D> &pts, const std::vector<IdEdge> &edges,
    const char *path) {
    FILE *f = fopen(path, "w");
    WriteResultToStream(pts, edges, f);
    fclose(f);
}

std::vector<Point2D> RandomPoints(int n, int max = -1) {
    if (max == -1) {
        max = n * 5;
    }
    std::vector<Point2D> points;
    points.reserve(n);
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dis(0, max);
    for (int i = 0; i < n; ++i) {
        points.emplace_back(dis(gen), dis(gen));
    }
    return points;
}

chrono::microseconds RunTriangulation(
    const Triangulator &algo, const std::vector<Point2D> &pts, FILE *output) {
    auto inputs = TagPointWithIndex(pts);
    auto start_time = chrono::system_clock::now();
    auto edges = algo.Triangulate(std::move(inputs));
    auto end_time = chrono::system_clock::now();
    WriteResultToStream(pts, edges, output);
    return end_time - start_time;
}

std::vector<Point2D> ReadPoints(FILE *f) {
    std::vector<Point2D> pts;
    int num;
    fscanf(f, "%d", &num);
    pts.reserve(num);
    for (int i = 0; i < num; ++i) {
        double x, y;
        auto err = fscanf(f, "%lf %lf", &x, &y);
        assert(err != -1);
        pts.emplace_back(x, y);
    }
    return pts;
}

std::vector<Point2D> ReadPointsFromFile(const char *path) {
    auto f = fopen(path, "r");
    auto result = ReadPoints(f);
    fclose(f);
    return result;
}

constexpr const char *kHelpMessage =
    "usage: triangulation "
    "[-r | --random] [-n <int>] [-i | --input file] [-o | --output file]\n"
    "\n"
    "-r | --random\n\tRandomly generate point data\n"
    "-n <int> = 50\n\tThe number of points, "
    "only valid when --random is specified\n"
    "-o | --out   file\n\tOutput file path\n"
    "-i | --input file\n\tInput point file path, overrides --random and -n\n"
    "-t | --time\n\tPrint algorithm execution time\n"
    "\nInput file format:\n"
    "<number-of-points : int>\n"
    "<x1 : double> <y1 : double>\n"
    "<x2 : double> <y2 : double>\n"
    "...\n";

int main(int argc, char *argv[]) {
    bool random = true;
    int n = 20;
    const char *inpath = nullptr;
    FILE *out_stream = stdout;
    std::vector<Point2D> points;
    std::unique_ptr<Triangulator> algo = std::make_unique<DivideAndConquer>();
    bool time = false;

    for (int i = 1; i < argc; ++i) {
        const char *arg = argv[i];
        if (arg == "-i"sv or arg == "--input"sv) {
            inpath = argv[++i];
        } else if (arg == "-r"sv or arg == "--random"sv) {
            random = true;
        } else if (arg == "-n"sv) {
            n = atoi(argv[++i]);
        } else if (arg == "-o"sv or arg == "--out"sv) {
            out_stream = fopen(argv[++i], "w");
        } else if (arg == "-h"sv or arg == "--help"sv) {
            printf("%s", kHelpMessage);
            exit(0);
        } else if (arg == "-t"sv or arg == "--time"sv) {
            time = true;
        } else {
            fprintf(stderr, "%s: invalid option: %s\n", argv[0], argv[i]);
            exit(1);
        }
    }

    if (inpath != nullptr) {
        points = ReadPointsFromFile(inpath);
    } else {
        points = RandomPoints(n);
    }

    LogPoints(points);

    auto running_time = RunTriangulation(*algo, points, out_stream);
    if (time) {
        fprintf(
            stderr, "Algorithm Execution Time: %.2f ms\n",
            running_time.count() / 1000.);
    }
}