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

# KalmanFilter and ExtendedKalmanFilter are now thin header-only
# subclasses of KalmanFilterBase (Template Method — see
# include/KalmanFilterBase.hpp), so every executable that uses either
# filter links this one shared .cpp instead of a per-filter one.
set(FILTER_CORE src/KalmanFilterBase.cpp)

# Task 1: linear KF, constant velocity model, position-only sensor.
add_executable(radar_tracker
    src/main.cpp
    ${FILTER_CORE}
)

# Task 2: extended KF, constant velocity model, nonlinear radar
# (range/bearing) sensor.
add_executable(ekf_tracker
    src/main_ekf.cpp
    ${FILTER_CORE}
)

# Task 3: unscented KF, same CV model + radar sensor as Task 2, useful
# for comparing EKF vs UKF on identical data. UnscentedKalmanFilter.cpp
# is separate from FILTER_CORE since it doesn't use predictImpl/updateImpl
# (see include/UnscentedKalmanFilter.hpp), but still needs KalmanFilterBase
# for its shared x_/P_ storage and accessors.
add_executable(ukf_tracker
    src/main_ukf.cpp
    src/UnscentedKalmanFilter.cpp
    ${FILTER_CORE}
)

# Test executables (simple assert-based sanity checks, no framework needed).
add_executable(test_kalman_filter
    tests/test_kalman_filter.cpp
    ${FILTER_CORE}
)

add_executable(test_extended_kalman_filter
    tests/test_extended_kalman_filter.cpp
    ${FILTER_CORE}
)

add_executable(test_unscented_kalman_filter
    tests/test_unscented_kalman_filter.cpp
    src/UnscentedKalmanFilter.cpp
    ${FILTER_CORE}
)

enable_testing()
add_test(NAME KalmanFilterTest COMMAND test_kalman_filter)
add_test(NAME ExtendedKalmanFilterTest COMMAND test_extended_kalman_filter)
add_test(NAME UnscentedKalmanFilterTest COMMAND test_unscented_kalman_filter)