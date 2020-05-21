# Delauny Triangulation Demo

HKU COMP8504 Artifact Repo by Xue Mingqi

## Build

### Dependency

- C++17 compiler (tested with apple-clang 11.0.0)
- cmake (tested with version 3.17)
- [Eigen3](http://eigen.tuxfamily.org/dox/) (tested with version 3.3.7)

- python3 for plotting (tested with python 3.7)
- [matplotlib](https://matplotlib.org/) (tested with version 3.2)

### Build

- Modify the entry `Eigen3_DIR` in `CMakeLists.txt` to point to
  `<your-eigen3-install-dir>/share/eigen3/cmake/`,
  which contains `Eigen3Config.cmake`
- `mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4`
- `triangulation` demo executable should be in `triangulation/bin`

### Execution

- `./triangulation --help` should be self-explanatory
- The plot tool is in `triangulation/scripts`, which accepts the output
  format of `triangulation` as input
- `./scripts/plot.py --help` should be self-explanatory


