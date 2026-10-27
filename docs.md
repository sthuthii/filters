# Radar Tracker Documentation

## Overview

The project generates synthetic target trajectories, adds sensor noise, runs
Kalman-filter variants, writes estimates to CSV, and creates plots for
evaluation.

```text
Generate trajectory -> Generate noisy measurements -> Build -> Run filters
-> Write CSV results -> Plot and calculate RMSE
```

## Repository layout

```text
include/       Filter, motion, measurement, and coordinate-model headers
src/           Filter implementations and executable entry points
tests/         Assertion-based filter tests
python/        Scenario-generation and plotting scripts
scenarios/     Trajectories and noisy measurement CSV files
results/       Filter outputs and plots
lib/Eigen/     Vendored Eigen matrix library
lib/Utils/     Shared CSV utilities
```

## C++ components

- `KalmanFilterBase.hpp/.cpp`: shared state, covariance, prediction, and
  update algebra for the KF and EKF.
- `KalmanFilter.hpp`: linear Kalman Filter using `F`, `Q`, `H`, and `R`.
- `ExtendedKalmanFilter.hpp`: EKF interface for caller-provided nonlinear
  predictions and Jacobians.
- `UnscentedKalmanFilter.hpp/.cpp`: UKF sigma-point prediction and update.
- `MotionModel.hpp/.cpp`: constant-velocity and coordinated-turn motion
  models, including transition matrices, Jacobians, and process noise.
- `MeasurementModel.hpp/.cpp`: position and radar measurement models.
- `CoordinateConverter.hpp`: Cartesian/polar conversion and angle
  normalization.
- `TrackingMetrics.hpp/.cpp`: position error and RMSE calculations.
- `CsvUtils.hpp`: simple numeric CSV reader and writer.

## Executables

Defined in `CMakeLists.txt`:

- `radar_tracker`: basic linear KF using position measurements.
- `ekf_tracker`: EKF using nonlinear radar range/bearing measurements.
- `ukf_tracker`: UKF using the same radar input as the EKF.
- `compare_filters`: compares a constant-velocity KF with a coordinated-turn
  EKF on identical 2D Cartesian measurements and optionally reports RMSE.
- `test_kalman_filter`, `test_extended_kalman_filter`, and
  `test_unscented_kalman_filter`: filter tests run by CTest.

## Python scripts

- `generate_trajectory.py`: creates a 1D constant-velocity trajectory.
- `generate_measurements.py`: adds Gaussian noise to 1D position data.
- `generate_2d_trajectory.py`: creates a 2D constant-velocity trajectory.
- `generate_2d_measurements.py`: adds Gaussian Cartesian noise for comparison.
- `generate_radar_measurements.py`: creates noisy range/bearing radar data.
- `plot_results.py`: plots the basic KF against truth and measurements.
- `plot_ekf_results.py`: plots EKF radar measurements, trajectory, and
  velocity.
- `plot_ukf_results.py`: plots UKF radar measurements, trajectory, and
  velocity.
- `plot_filter_comparison.py`: creates a four-panel KF/EKF comparison plot
  showing trajectories, position error, velocities, and cumulative RMSE.

## Build and test on Windows

From `D:\radar_tracker`:

```powershell
cmake -S . -B build-vs -G "Visual Studio 17 2022"
cmake --build build-vs --config Release --parallel
ctest --test-dir build-vs -C Release --output-on-failure
```

The VS Code IntelliSense configuration is in
`.vscode/c_cpp_properties.json`. It selects the MSVC 2022 compiler, C++17,
project headers, and vendored Eigen.

## Basic KF workflow

```powershell
.\.venv\Scripts\python.exe python\generate_trajectory.py `
    scenarios\trajectory.csv 20 0.5 2.0 0.0

.\.venv\Scripts\python.exe python\generate_measurements.py `
    scenarios\trajectory.csv scenarios\measurements.csv 2.0

.\build-vs\Release\radar_tracker.exe `
    scenarios\measurements.csv results\output.csv

.\.venv\Scripts\python.exe python\plot_results.py `
    scenarios\trajectory.csv results\output.csv results\plot.png
```

## KF versus EKF comparison

Generate a 2D scenario and Cartesian measurements:

```powershell
.\.venv\Scripts\python.exe python\generate_2d_trajectory.py `
    scenarios\trajectory_2d.csv

.\.venv\Scripts\python.exe python\generate_2d_measurements.py `
    scenarios\trajectory_2d.csv scenarios\measurements_2d.csv
```

Run the comparison:

```powershell
.\build-vs\Release\compare_filters.exe `
    scenarios\measurements_2d.csv `
    results\comparison\cv_output.csv `
    results\comparison\ekf_output.csv `
    scenarios\trajectory_2d.csv
```

Generate the comparison plot:

```powershell
.\.venv\Scripts\python.exe python\plot_filter_comparison.py
```

The comparison command writes:

- `results/comparison/cv_output.csv`
- `results/comparison/ekf_output.csv`
- Position RMSE values to the console

The plotting script writes:

- `results/comparison/filter_comparison.png`

The current comparison executable evaluates **KF versus EKF**. The standalone
UKF is run separately with `ukf_tracker`.

## Input and output formats

- 1D position measurements: `t,z` or `t,z1,...`
- Radar measurements: `t,range,bearing`
- 2D comparison measurements: `time,px,py`
- 1D KF output: time, measurements, estimated position, estimated velocity
- Radar output: time, range, bearing, Cartesian position, and velocity state
- Comparison output:
  - CV-KF: `time,px,py,vx,vy`
  - EKF: `time,px,py,v,theta,omega`

Lower position RMSE indicates better tracking accuracy for the supplied
ground-truth scenario.
