"""Generate noisy Cartesian measurements from a 2D trajectory.

Usage:
    python generate_2d_measurements.py [trajectory_csv] [output_csv] [noise_std] [seed]
"""
import csv
import random
import sys


def generate_measurements(trajectory_path, output_path, noise_std=3.0, seed=42):
    random.seed(seed)
    with open(trajectory_path, "r", newline="") as source:
        rows = csv.DictReader(source)
        samples = [(float(row["t"]), float(row["x"]), float(row["y"])) for row in rows]

    with open(output_path, "w", newline="") as destination:
        writer = csv.writer(destination)
        writer.writerow(["time", "px", "py"])
        for time, x, y in samples:
            writer.writerow(
                [
                    round(time, 4),
                    round(x + random.gauss(0.0, noise_std), 6),
                    round(y + random.gauss(0.0, noise_std), 6),
                ]
            )

    print(f"Wrote {len(samples)} noisy 2D measurements to {output_path}")


def main():
    trajectory_path = (
        sys.argv[1] if len(sys.argv) > 1 else "scenarios/trajectory_2d.csv"
    )
    output_path = (
        sys.argv[2] if len(sys.argv) > 2 else "scenarios/measurements_2d.csv"
    )
    noise_std = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    seed = int(sys.argv[4]) if len(sys.argv) > 4 else 42
    generate_measurements(trajectory_path, output_path, noise_std, seed)


if __name__ == "__main__":
    main()
