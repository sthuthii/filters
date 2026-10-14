"""Generate noisy range/bearing (radar) measurements from a 2D trajectory.

Reads trajectory_2d.csv (t,x,y,vx,vy) and writes radar_measurements.csv
(t,range,bearing), where range/bearing are computed from (x,y) plus
Gaussian noise. This is the exact CSV format src/main_ekf.cpp and
src/main_ukf.cpp both read, so the same file feeds either filter —
useful for comparing EKF vs UKF on identical data.

Usage:
    python3 generate_radar_measurements.py [trajectory_csv] [output_csv] [sigma_range] [sigma_bearing] [seed]
"""
import sys
import csv
import math
import random


def main():
    traj_path = sys.argv[1] if len(sys.argv) > 1 else "scenarios/trajectory_2d.csv"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "scenarios/radar_measurements.csv"
    sigma_range = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    sigma_bearing = float(sys.argv[4]) if len(sys.argv) > 4 else 0.02
    seed = int(sys.argv[5]) if len(sys.argv) > 5 else 42

    random.seed(seed)

    with open(traj_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = [(float(r["t"]), float(r["x"]), float(r["y"])) for r in reader]

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "range", "bearing"])
        for t, x, y in rows:
            r = math.hypot(x, y) + random.gauss(0.0, sigma_range)
            b = math.atan2(y, x) + random.gauss(0.0, sigma_bearing)
            writer.writerow([round(t, 4), round(r, 6), round(b, 8)])

    print(f"Wrote {len(rows)} noisy radar measurements to {out_path}")


if __name__ == "__main__":
    main()
