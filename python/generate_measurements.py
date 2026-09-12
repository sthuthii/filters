"""Generate noisy position measurements from a true trajectory CSV.

Reads trajectory.csv (t,p,v) and writes measurements.csv (t,z), where
z = p + Gaussian noise, matching the sensor noise variance the filter
in main.cpp expects (see r_variance in src/main.cpp).

Usage:
    python3 generate_measurements.py [trajectory_csv] [output_csv] [noise_std] [seed]
"""
import sys
import csv
import random


def main():
    traj_path = sys.argv[1] if len(sys.argv) > 1 else "scenarios/trajectory.csv"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "scenarios/measurements.csv"
    noise_std = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0  # sqrt(r_variance)=sqrt(4.0)
    seed = int(sys.argv[4]) if len(sys.argv) > 4 else 42

    random.seed(seed)

    with open(traj_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = [(float(row["t"]), float(row["p"])) for row in reader]

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "z"])
        for t, p in rows:
            z = p + random.gauss(0.0, noise_std)
            writer.writerow([round(t, 4), round(z, 6)])

    print(f"Wrote {len(rows)} noisy measurements to {out_path} (noise_std={noise_std})")


if __name__ == "__main__":
    main()
