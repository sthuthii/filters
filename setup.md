# Build and Run Setup

This guide builds and runs the radar tracker after a fresh clone. Eigen is
vendored in `lib/Eigen`, so no separate Eigen installation is required.

## 1. Install dependencies

### Ubuntu/Debian

From a terminal:

```bash
sudo apt update
sudo apt install -y build-essential cmake python3 python3-venv python3-pip
```

The project includes Eigen under `lib/Eigen`, so no separate Eigen package is required.

### Windows

Install:

- CMake (add it to `PATH`)
- Visual Studio 2022 with **Desktop development with C++**, or a modern
  MinGW-w64 toolchain with complete C++17 standard-library support
- Python 3

Check the tools from PowerShell:

```powershell
cmake --version
python --version
```

## 2. Create the Python environment

From the repository root:

```text
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Activate the environment again whenever you open a new terminal:

```text
source .venv/bin/activate
```

On Windows PowerShell:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## 3. Configure and build

Run these commands from the repository root:

```text
cmake -S . -B build
cmake --build build --parallel
```

The executables are created at:

```text
build/radar_tracker
build/ekf_tracker
build/ukf_tracker
build/test_kalman_filter
build/test_extended_kalman_filter
build/test_unscented_kalman_filter
```

For Visual Studio on Windows, use a separate build directory and include the
configuration name when building:

```powershell
cmake -S . -B build-msvc -G "Visual Studio 17 2022"
cmake --build build-msvc --config Release --parallel
```

## 4. Run the tests

```text
ctest --test-dir build --output-on-failure
```

For the Visual Studio build:

```powershell
ctest --test-dir build-msvc -C Release --output-on-failure
```

## 5. Generate sample data and run the tracker

```text
python python/generate_trajectory.py scenarios/trajectory.csv 20 0.5 2.0 0.0
python python/generate_measurements.py scenarios/trajectory.csv scenarios/measurements.csv 2.0
./build/radar_tracker scenarios/measurements.csv results/output.csv
```

On Windows PowerShell:

```powershell
python python\generate_trajectory.py scenarios\trajectory.csv 20 0.5 2.0 0.0
python python\generate_measurements.py scenarios\trajectory.csv scenarios\measurements.csv 2.0
.\build-msvc\Release\radar_tracker.exe scenarios\measurements.csv results\output.csv
```

The tracker writes estimates to `results/output.csv`.

## 6. Create a plot

```text
python python/plot_results.py scenarios/trajectory.csv results/output.csv results/plot.png
```

The plot is saved to `results/plot.png`.

## Clean rebuild

To remove CMake's generated build files and configure from scratch:

```text
rm -rf build
cmake -S . -B build
cmake --build build --parallel
```

On Windows, remove only the generated build directory and configure it again:

```powershell
Remove-Item -Recurse -Force build-msvc
cmake -S . -B build-msvc -G "Visual Studio 17 2022"
cmake --build build-msvc --config Release --parallel
```
