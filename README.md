# radar_tracker

A radar tracking project containing linear, extended, and unscented Kalman
filter examples with a Constant Velocity (CV) motion model.

## Structure

```
radar_tracker/
├── lib/Eigen/          Vendored Eigen headers (header-only, no install needed)
├── lib/Utils/          Reserved for future shared utilities
├── include/            Filter, motion, measurement, and coordinate-model headers
├── src/                Linear KF, EKF, and UKF applications and implementations
├── tests/              Assert-based sanity tests for all three filters
├── scenarios/          Generated trajectory/measurement CSVs live here
├── python/             Scripts to generate synthetic data and plot results
└── results/            Filter output CSV + plot land here
```

## Design

- `KalmanFilter` (`include/KalmanFilter.hpp`) is a generic linear KF:
  `predict(F, Q)` / `update(z, H, R)`. It doesn't know anything about CV
  specifically.
- `ExtendedKalmanFilter` (`include/ExtendedKalmanFilter.hpp`) accepts caller-
  evaluated nonlinear predictions and Jacobians.
- `UnscentedKalmanFilter` (`include/UnscentedKalmanFilter.hpp`) evaluates the
  motion and measurement models at sigma points.
- `ConstantVelocityModel` (`include/MotionModel.hpp`) builds `F` and `Q` for the
  1D CV model, state = `[position, velocity]`.
- `PositionMeasurementModel` (`include/MeasurementModel.hpp`) builds `H` and `R`
  for a sensor that only observes position.
- `src/main.cpp` wires these together: reads `t,z` measurements from a CSV file,
  runs predict/update at each step, and writes `t,z,pos_est,vel_est` to a CSV.

## Build after cloning

Requires CMake (>=3.10) and a C++17 compiler. Eigen is vendored under `lib/Eigen`,
so no external Eigen install is needed.

From the repository root, create a fresh out-of-source build directory and
configure the project:

```text
cmake -S . -B build
cmake --build build --parallel
```

On Windows, the same commands work from PowerShell with Visual Studio or a
modern MinGW-w64 toolchain installed. Use a compiler with complete C++17
standard-library support; very old MinGW/GCC installations (for example GCC
6.x) are not compatible with the vendored Eigen headers. For a Visual Studio
multi-configuration generator, specify the configuration when building:

```powershell
cmake -S . -B build-msvc -G "Visual Studio 17 2022"
cmake --build build-msvc --config Release --parallel
```

For a single-configuration generator such as MinGW Makefiles:

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-mingw --parallel
```

The build creates these executables:

- `radar_tracker` — linear KF with position measurements
- `ekf_tracker` — extended KF with range/bearing measurements
- `ukf_tracker` — unscented KF with range/bearing measurements
- `test_kalman_filter`, `test_extended_kalman_filter`,
  `test_unscented_kalman_filter` — assert-based tests

## Run

1. Generate a synthetic true trajectory and noisy position measurements:

   ```bash
   python3 python/generate_trajectory.py scenarios/trajectory.csv 20 0.5 2.0 0.0
   python3 python/generate_measurements.py scenarios/trajectory.csv scenarios/measurements.csv 2.0
   ```

2. Run the tracker:

   ```bash
   ./build/radar_tracker scenarios/measurements.csv results/output.csv
   ```

   On Windows PowerShell, use `.\build\Release\radar_tracker.exe` for a
   Visual Studio build, or `.\build-mingw\radar_tracker.exe` for MinGW.

3. Plot the result:

   ```bash
   python3 python/plot_results.py scenarios/trajectory.csv results/output.csv results/plot.png
   ```

For radar EKF or UKF output, use the corresponding Cartesian trajectory and
velocity plot:

```bash
./build/ekf_tracker scenarios/radar_measurements.csv results/ekf_output.csv
python3 python/plot_ekf_results.py results/ekf_output.csv results/ekf_output.png
./build/ukf_tracker scenarios/radar_measurements.csv results/ukf_output.csv
python3 python/plot_ukf_results.py results/ukf_output.csv results/ukf_output.png
```

To compare a constant-velocity KF with the coordinated-turn EKF on the same
Cartesian measurements, generate a 2D scenario and run:

```bash
python3 python/generate_2d_trajectory.py scenarios/trajectory_2d.csv
python3 python/generate_2d_measurements.py scenarios/trajectory_2d.csv scenarios/measurements_2d.csv
./build/compare_filters scenarios/measurements_2d.csv \
  results/comparison/cv_output.csv results/comparison/ekf_output.csv \
  scenarios/trajectory_2d.csv
```

The comparison command writes both estimate CSVs and reports each filter's
position RMSE against the supplied ground truth.

`scenarios/measurements.csv` needs a header `t,z1,z2,...` and one time plus
position sample per line. Radar inputs need a `t,range,bearing` header.

## Notes on tuning

- `sigma_a` (in `src/main.cpp`) is the assumed process noise (how much the
  target's velocity is expected to wander) — raise it to trust new
  measurements more, lower it to trust the constant-velocity model more.
- `r_variance` is the sensor's position measurement noise variance — should
  match (or approximate) the actual sensor noise.

## Tests

```bash
cd build
ctest
```

On Windows, run the tests from the build directory with:

```powershell
ctest --test-dir build-mingw --output-on-failure
# Visual Studio generator:
ctest --test-dir build-msvc -C Release --output-on-failure
```

The test feeds the filter a noise-free CV trajectory with a deliberately wrong
initial velocity guess and checks that the estimate converges to the truth,
and that the covariance shrinks after each update.
