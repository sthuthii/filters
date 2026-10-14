"""Generate a true 2D constant-velocity trajectory as CSV: t,x,y,vx,vy

This is the ground-truth path a radar would be tracking — feed it to
generate_radar_measurements.py to get noisy range/bearing readings for
main_ekf.cpp / main_ukf.cpp.

Usage:
    python3 generate_2d_trajectory.py [output_csv] [duration_s] [dt] [vx] [vy] [start_x] [start_y]
"""
import sys
import csv


def generate(duration_s=30.0, dt=0.5, vx=3.0, vy=1.0, start_x=100.0, start_y=50.0):
    rows = []
    t = 0.0
    x, y = start_x, start_y
    while t <= duration_s + 1e-9:
        rows.append((round(t, 4), round(x, 6), round(y, 6), vx, vy))
        t += dt
        x += vx * dt
        y += vy * dt
    return rows


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "scenarios/trajectory_2d.csv"
    duration_s = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    dt = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
    vx = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
    vy = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
    start_x = float(sys.argv[6]) if len(sys.argv) > 6 else 100.0
    start_y = float(sys.argv[7]) if len(sys.argv) > 7 else 50.0

    rows = generate(duration_s, dt, vx, vy, start_x, start_y)

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "vx", "vy"])
        writer.writerows(rows)

    print(f"Wrote {len(rows)} true 2D trajectory points to {out_path}")


if __name__ == "__main__":
    main()
