cmake_minimum_required(VERSION 3.16)
project(slam_benchmark LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

# Dependencies are installed here by scripts/build_deps.sh.
list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/build/deps")
set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/build/deps/lib")

# Opt-in CHOLMOD variants of Ceres and g2o (LGPL/GPL), built by
# hand into build/deps-cholmod (SuiteSparse, OpenBLAS, METIS in build/deps-suitesparse).
option(SLAM_CHOLMOD "Add CHOLMOD variants of Ceres and g2o" OFF)
if(SLAM_CHOLMOD)
    list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/build/deps-cholmod"
                                   "${CMAKE_CURRENT_SOURCE_DIR}/build/deps-suitesparse")
    list(PREPEND CMAKE_BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/build/deps-cholmod/lib"
                                   "${CMAKE_CURRENT_SOURCE_DIR}/build/deps-suitesparse/lib")
endif()

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(slam_common INTERFACE)
target_include_directories(slam_common INTERFACE src)
target_link_libraries(slam_common INTERFACE Eigen3::Eigen)

add_executable(slam_benchmark src/main.cpp)
target_link_libraries(slam_benchmark PRIVATE slam_common)

# One folder per solver; each defines a target slam_<name> when its
# dependency is found.
set(SLAM_SOLVERS "fatrop;ceres;g2o;gtsam" CACHE STRING "Solvers to build")
foreach(solver ${SLAM_SOLVERS})
    add_subdirectory(src/solvers/${solver})
    if(TARGET slam_${solver})
        string(TOUPPER ${solver} SOLVER)
        target_link_libraries(slam_benchmark PRIVATE slam_${solver})
        target_compile_definitions(slam_benchmark PRIVATE SLAM_WITH_${SOLVER})
        message(STATUS "solver ${solver}: enabled")
    else()
        message(STATUS "solver ${solver}: not found, skipped")
    endif()
endforeach()
