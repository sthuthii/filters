"""Plot CV-KF versus EKF comparison results.

Usage:
    python python/plot_filter_comparison.py \
        scenarios/trajectory_2d.csv \
        results/comparison/cv_output.csv \
        results/comparison/ekf_output.csv \
        results/comparison/filter_comparison.png

The figure contains the estimated trajectories, position error over time,
velocity estimates, and cumulative RMSE so both accuracy and behavior are
visible in one artifact.
"""
import csv
import math
import sys

import matplotlib


def read_csv(path, required):
    with open(path, "r", newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None or any(
            field not in reader.fieldnames for field in required
        ):
            raise ValueError(f"{path} must contain columns: {', '.join(required)}")
        return [
            {field: float(row[field]) for field in required}
            for row in reader
        ]


def cumulative_rmse(errors):
    values = []
    sum_squared = 0.0
    for index, error in enumerate(errors, start=1):
        sum_squared += error * error
        values.append(math.sqrt(sum_squared / index))
    return values


def main():
    truth_path = (
        sys.argv[1] if len(sys.argv) > 1 else "scenarios/trajectory_2d.csv"
    )
    cv_path = (
        sys.argv[2]
        if len(sys.argv) > 2
        else "results/comparison/cv_output.csv"
    )
    ekf_path = (
        sys.argv[3]
        if len(sys.argv) > 3
        else "results/comparison/ekf_output.csv"
    )
    save_path = (
        sys.argv[4]
        if len(sys.argv) > 4
        else "results/comparison/filter_comparison.png"
    )

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    truth = read_csv(truth_path, ["t", "x", "y", "vx", "vy"])
    cv = read_csv(cv_path, ["time", "px", "py", "vx", "vy"])
    ekf = read_csv(ekf_path, ["time", "px", "py", "v", "theta", "omega"])

    if not truth or len(truth) != len(cv) or len(truth) != len(ekf):
        raise ValueError("Truth, CV-KF, and EKF files must contain equal non-zero rows")

    time = [row["t"] for row in truth]
    cv_x = [row["px"] for row in cv]
    cv_y = [row["py"] for row in cv]
    ekf_x = [row["px"] for row in ekf]
    ekf_y = [row["py"] for row in ekf]
    truth_x = [row["x"] for row in truth]
    truth_y = [row["y"] for row in truth]

    cv_errors = [
        math.hypot(row["px"] - truth_row["x"], row["py"] - truth_row["y"])
        for row, truth_row in zip(cv, truth)
    ]
    ekf_errors = [
        math.hypot(row["px"] - truth_row["x"], row["py"] - truth_row["y"])
        for row, truth_row in zip(ekf, truth)
    ]
    cv_rmse = math.sqrt(sum(error * error for error in cv_errors) / len(cv_errors))
    ekf_rmse = math.sqrt(sum(error * error for error in ekf_errors) / len(ekf_errors))

    figure, axes = plt.subplots(2, 2, figsize=(14, 10))

    axes[0, 0].plot(truth_x, truth_y, color="black", linewidth=2, label="Ground truth")
    axes[0, 0].plot(cv_x, cv_y, color="tab:blue", label=f"CV-KF (RMSE {cv_rmse:.2f})")
    axes[0, 0].plot(ekf_x, ekf_y, color="tab:orange", label=f"EKF (RMSE {ekf_rmse:.2f})")
    axes[0, 0].scatter(truth_x[0], truth_y[0], color="tab:green", marker="o", label="Start")
    axes[0, 0].set_title("Estimated 2D trajectories")
    axes[0, 0].set_xlabel("x position")
    axes[0, 0].set_ylabel("y position")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    axes[0, 1].plot(time, cv_errors, color="tab:blue", label="CV-KF")
    axes[0, 1].plot(time, ekf_errors, color="tab:orange", label="EKF")
    axes[0, 1].set_title("Position error over time")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Euclidean position error")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()

    axes[1, 0].plot(time, [row["vx"] for row in truth], "k--", label="True vx")
    axes[1, 0].plot(time, [row["vy"] for row in truth], "k:", label="True vy")
    axes[1, 0].plot(time, [row["vx"] for row in cv], color="tab:blue", label="CV-KF vx")
    axes[1, 0].plot(time, [row["vy"] for row in cv], color="tab:cyan", label="CV-KF vy")
    ekf_vx = [row["v"] * math.cos(row["theta"]) for row in ekf]
    ekf_vy = [row["v"] * math.sin(row["theta"]) for row in ekf]
    axes[1, 0].plot(time, ekf_vx, color="tab:orange", label="EKF vx")
    axes[1, 0].plot(time, ekf_vy, color="tab:red", label="EKF vy")
    axes[1, 0].set_title("Velocity estimates")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Velocity")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend(ncol=2, fontsize="small")

    axes[1, 1].plot(time, cumulative_rmse(cv_errors), color="tab:blue", label="CV-KF")
    axes[1, 1].plot(time, cumulative_rmse(ekf_errors), color="tab:orange", label="EKF")
    axes[1, 1].set_title("Cumulative position RMSE")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("RMSE")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    figure.suptitle("CV-KF versus EKF filter comparison", fontsize=16)
    figure.tight_layout()
    figure.savefig(save_path, dpi=160, bbox_inches="tight")
    print(f"CV-KF RMSE: {cv_rmse:.6f}")
    print(f"EKF RMSE:   {ekf_rmse:.6f}")
    print(f"Saved comparison plot to {save_path}")


if __name__ == "__main__":
    main()
