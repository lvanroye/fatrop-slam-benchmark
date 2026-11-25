#!/usr/bin/env bash
# Build and install every solver in external/ into build/deps.
#
# Usage: scripts/build_deps.sh [options] [component ...]   (default: all components)
#   --jobs N          parallel build jobs (default: 4)
#   --prefix PATH     install prefix (default: build/deps)
# Components: blasfeo fatrop eigen ceres g2o gtsam
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="$ROOT/build/deps"
JOBS=4
components=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --jobs) JOBS="$2"; shift 2 ;;
        --prefix) PREFIX="$2"; shift 2 ;;
        -h|--help) sed -n '2,7p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) components+=("$1"); shift ;;
    esac
done
FLAGS="-O3 -march=native"

common=(
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX="$PREFIX"
    -DCMAKE_PREFIX_PATH="$PREFIX"
    -DCMAKE_CXX_FLAGS="$FLAGS"
    -DCMAKE_C_FLAGS="$FLAGS"
    -DBUILD_TESTING=OFF
)

build() {
    local name="$1"; shift
    echo "=== $name"
    cmake -S "$ROOT/external/$name" -B "$ROOT/build/external/$name" "${common[@]}" "$@" >/dev/null
    cmake --build "$ROOT/build/external/$name" -j "$JOBS" >/dev/null
    cmake --install "$ROOT/build/external/$name" >/dev/null
}

blasfeo() { build blasfeo -DBLASFEO_EXAMPLES=OFF; }
fatrop()  { build fatrop -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_WITH_C_INTERFACE=OFF; }
eigen()   { build eigen -DEIGEN_BUILD_DOC=OFF -DEIGEN_BUILD_PKGCONFIG=OFF; }
ceres()   { build ceres-solver -DMINIGLOG=ON -DMINIGLOG_MAX_LOG_LEVEL=-1 -DGFLAGS=OFF -DSUITESPARSE=OFF -DLAPACK=OFF \
                -DACCELERATESPARSE=OFF -DUSE_CUDA=OFF -DEIGENSPARSE=ON \
                -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DPROVIDE_UNINSTALL_TARGET=OFF; }
g2o()     { build g2o -DG2O_USE_CHOLMOD=OFF -DG2O_USE_CSPARSE=OFF -DG2O_USE_OPENGL=OFF \
                -DG2O_USE_OPENMP=OFF -DG2O_USE_LOGGING=OFF -DG2O_BUILD_APPS=OFF \
                -DG2O_BUILD_EXAMPLES=OFF -DBUILD_UNITTESTS=OFF -DBUILD_WITH_MARCH_NATIVE=ON; }
gtsam()   { build gtsam -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
                -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF \
                -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF -DGTSAM_USE_BOOST_FEATURES=OFF \
                -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF \
                -DGTSAM_BUILD_WITH_MARCH_NATIVE=ON -DCMAKE_INSTALL_RPATH="$PREFIX/lib"; }

[[ ${#components[@]} -eq 0 ]] && components=(blasfeo fatrop eigen ceres g2o gtsam)
for c in "${components[@]}"; do "$c"; done
echo "installed into $PREFIX"
