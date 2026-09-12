# radar_tracker

A simple 1D radar tracking project. **Task 1 only** is implemented here:
a Linear Kalman Filter with a Constant Velocity (CV) motion model.

## Structure

```
radar_tracker/
├── lib/Eigen/          Vendored Eigen headers (header-only, no install needed)
├── lib/Utils/          Reserved for future shared utilities
├── include/            KalmanFilter, MotionModel, MeasurementModel (implemented);
│                        EKF / CoordinateConverter / ValidationGate / TrackingMetrics
│                        are placeholder headers for future tasks
├── src/                KalmanFilter.cpp + main.cpp (reads a CSV of measurements,
│                        runs the filter, writes a CSV of estimates)
├── tests/              Minimal assert-based sanity test, no test framework needed
├── scenarios/          Generated trajectory/measurement CSVs live here
├── python/             Scripts to generate synthetic data and plot results
└── results/            Filter output CSV + plot land here
```

## Design

- `KalmanFilter` (`include/KalmanFilter.hpp`, `src/KalmanFilter.cpp`) is a generic
  linear KF: `predict(F, Q)` / `update(z, H, R)`. It doesn't know anything about
  CV specifically.
- `ConstantVelocityModel` (`include/MotionModel.hpp`) builds `F` and `Q` for the
  1D CV model, state = `[position, velocity]`.
- `PositionMeasurementModel` (`include/MeasurementModel.hpp`) builds `H` and `R`
  for a sensor that only observes position.
- `src/main.cpp` wires these together: reads `t,z` measurements from a CSV file,
  runs predict/update at each step, and writes `t,z,pos_est,vel_est` to a CSV.

## Build

Requires CMake (>=3.10) and a C++17 compiler. Eigen is vendored under `lib/Eigen`,
so no external Eigen install is needed.

```bash
cd radar_tracker
mkdir -p build && cd build
cmake ..
make
```

This produces two executables in `build/`:
- `radar_tracker` — the main tracker
- `test_kalman_filter` — sanity checks (also runnable via `ctest`)

## Run

1. Generate a synthetic true trajectory and noisy measurements:

   ```bash
   cd radar_tracker
   python3 python/generate_trajectory.py scenarios/trajectory.csv 20 0.5 2.0 0.0
   python3 python/generate_measurements.py scenarios/trajectory.csv scenarios/measurements.csv 2.0
   ```

2. Run the tracker:

   ```bash
   ./build/radar_tracker scenarios/measurements.csv results/output.csv
   ```

3. Plot the result:

   ```bash
   python3 python/plot_results.py scenarios/trajectory.csv results/output.csv results/plot.png
   ```

`scenarios/measurements.csv` just needs a header `t,z` and one `time,position`
pair per line — you can swap in your own data (e.g. from a file or a UDP
client) instead of the generated one, as long as it follows that format.

## Notes on tuning

- `sigma_a` (in `src/main.cpp`) is the assumed process noise (how much the
  target's velocity is expected to wander) — raise it to trust new
  measurements more, lower it to trust the constant-velocity model more.
- `r_variance` is the sensor's position measurement noise variance — should
  match (or approximate) the actual sensor noise.

## Tests

```bash
cd build
./test_kalman_filter
# or:
ctest
```

The test feeds the filter a noise-free CV trajectory with a deliberately wrong
initial velocity guess and checks that the estimate converges to the truth,
and that the covariance shrinks after each update.
