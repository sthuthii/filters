"""Plot the true trajectory, noisy measurements, and Kalman filter estimate.

Usage:
    python3 plot_results.py [trajectory_csv] [output_csv] [save_path]

If save_path is given, the plot is saved to that file instead of shown
interactively (useful when running without a display).
"""
import sys
import csv

import matplotlib
import matplotlib.pyplot as plt


def read_csv(path, fields):
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        cols = {field: [] for field in fields}
        for row in reader:
            for field in fields:
                cols[field].append(float(row[field]))
    return cols


def main():
    traj_path = sys.argv[1] if len(sys.argv) > 1 else "scenarios/trajectory.csv"
    output_path = sys.argv[2] if len(sys.argv) > 2 else "results/output.csv"
    save_path = sys.argv[3] if len(sys.argv) > 3 else None

    if save_path:
        matplotlib.use("Agg")

    traj = read_csv(traj_path, ["t", "p"])
    est = read_csv(output_path, ["t", "z", "pos_est"])

    plt.figure(figsize=(10, 6))
    plt.plot(traj["t"], traj["p"], label="True Position", color="tab:blue", linewidth=2)
    plt.scatter(est["t"], est["z"], label="Noisy Measurement", color="tab:green", s=15, alpha=0.6)
    plt.plot(est["t"], est["pos_est"], label="Kalman Estimate", color="tab:orange", linewidth=2)

    plt.xlabel("Time (s)")
    plt.ylabel("Position")
    plt.title("1D Kalman Filter Tracker (Constant Velocity Model)")
    plt.legend()
    plt.grid(True, alpha=0.3)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved plot to {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
