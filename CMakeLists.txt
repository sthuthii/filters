cmake_minimum_required(VERSION 3.10)
project(radar_tracker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Eigen is vendored under lib/Eigen (header-only), so just add lib/ to the
# include path — headers are included as "Eigen/Dense" from within Eigen
# itself, and as "../lib/Eigen/Dense" from this project's own headers.
include_directories(${CMAKE_SOURCE_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Main tracker executable (Task 1: linear KF, constant velocity model).
add_executable(radar_tracker
    src/main.cpp
    src/KalmanFilter.cpp
)

# Test executable (simple assert-based sanity checks, no framework needed).
add_executable(test_kalman_filter
    tests/test_kalman_filter.cpp
    src/KalmanFilter.cpp
)

enable_testing()
add_test(NAME KalmanFilterTest COMMAND test_kalman_filter)
