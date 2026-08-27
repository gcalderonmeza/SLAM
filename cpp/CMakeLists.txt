cmake_minimum_required(VERSION 3.16)
project(slam_cpp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─── Core library ────────────────────────────────────────────────────────────
add_library(slam STATIC
    src/occupancy_grid_map.cpp
    src/beam_range_finder_model.cpp
    src/fast_slam.cpp
)
target_include_directories(slam PUBLIC include)
target_compile_options(slam PRIVATE -Wall -Wextra)

# ─── Console runner ──────────────────────────────────────────────────────────
add_executable(slam_runner src/main.cpp)
target_link_libraries(slam_runner PRIVATE slam)

# Optional OpenCV visualisation (off by default).
option(WITH_OPENCV "Enable OpenCV live visualisation in slam_runner" OFF)
if(WITH_OPENCV)
    find_package(OpenCV REQUIRED core imgproc highgui)
    target_compile_definitions(slam_runner PRIVATE WITH_OPENCV)
    target_link_libraries(slam_runner PRIVATE ${OpenCV_LIBS})
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
endif()

# ─── Unit tests ──────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
