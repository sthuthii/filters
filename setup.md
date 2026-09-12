# Ubuntu Setup

This guide builds and runs the radar tracker on Ubuntu.

## 1. Install system dependencies

From a terminal:

```bash
sudo apt update
sudo apt install -y build-essential cmake python3 python3-venv python3-pip
```

The project includes Eigen under `lib/Eigen`, so no separate Eigen package is required.

## 2. Create the Python environment

From the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Activate the environment again whenever you open a new terminal:

```bash
source .venv/bin/activate
```

## 3. Configure and build

Run these commands from the repository root:

```bash
cmake -S . -B build
cmake --build build --parallel
```

The executables are created at:

```text
build/radar_tracker
build/test_kalman_filter
```

## 4. Run the tests

```bash
ctest --test-dir build --output-on-failure
```

You can also run the test executable directly:

```bash
./build/test_kalman_filter
```

## 5. Generate sample data and run the tracker

```bash
python python/generate_trajectory.py scenarios/trajectory.csv 20 0.5 2.0 0.0
python python/generate_measurements.py scenarios/trajectory.csv scenarios/measurements.csv 2.0
./build/radar_tracker scenarios/measurements.csv results/output.csv
```

The tracker writes estimates to `results/output.csv`.

## 6. Create a plot

```bash
python python/plot_results.py scenarios/trajectory.csv results/output.csv results/plot.png
```

The plot is saved to `results/plot.png`.

## Clean rebuild

To remove CMake's generated build files and configure from scratch:

```bash
rm -rf build
cmake -S . -B build
cmake --build build --parallel
```
