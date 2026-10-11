"""Plot UKF radar measurements and Cartesian state estimates.

Usage:
    python python/plot_ukf_results.py [input_csv] [save_path]

The UKF output contains polar radar measurements (range/bearing) and
Cartesian state estimates (x/y/vx/vy). The radar measurements are converted
to Cartesian points for comparison with the estimated trajectory.
"""
import csv
import math
import sys

import matplotlib


def read_ukf_output(path):
    required = ("t", "range", "bearing", "x_est", "y_est", "vx_est", "vy_est")
    values = {field: [] for field in required}

    with open(path, "r", newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None or any(
            field not in reader.fieldnames for field in required
        ):
            raise ValueError(
                f"{path} must contain columns: {', '.join(required)}"
            )

        for row in reader:
            for field in required:
                values[field].append(float(row[field]))

    if not values["t"]:
        raise ValueError(f"{path} contains no UKF samples")
    return values


def main():
    input_path = (
        sys.argv[1] if len(sys.argv) > 1 else "results/ukf_output.csv"
    )
    save_path = sys.argv[2] if len(sys.argv) > 2 else "results/ukf_output.png"

    if save_path:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    data = read_ukf_output(input_path)

    measured_x = [
        radius * math.cos(angle)
        for radius, angle in zip(data["range"], data["bearing"])
    ]
    measured_y = [
        radius * math.sin(angle)
        for radius, angle in zip(data["range"], data["bearing"])
    ]

    figure, axes = plt.subplots(1, 2, figsize=(13, 5))

    axes[0].scatter(
        measured_x,
        measured_y,
        label="Radar measurement",
        color="tab:green",
        alpha=0.7,
    )
    axes[0].plot(
        data["x_est"],
        data["y_est"],
        label="UKF estimate",
        color="tab:orange",
        marker="o",
    )
    axes[0].scatter(
        data["x_est"][0],
        data["y_est"][0],
        label="Initial estimate",
        color="tab:blue",
        marker="x",
        s=70,
    )
    axes[0].set_xlabel("x position")
    axes[0].set_ylabel("y position")
    axes[0].set_title("UKF Cartesian trajectory")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(data["t"], data["vx_est"], label="vx", color="tab:purple")
    axes[1].plot(data["t"], data["vy_est"], label="vy", color="tab:red")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Velocity")
    axes[1].set_title("UKF velocity estimates")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    figure.tight_layout()
    if save_path:
        figure.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved plot to {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
