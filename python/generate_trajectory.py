"""Generate a true 1D constant-velocity trajectory as CSV: t,p,v

Usage:
    python3 generate_trajectory.py [output_csv] [duration_s] [dt] [velocity] [start_pos]
"""
import sys
import csv


def generate(duration_s=20.0, dt=0.5, velocity=2.0, start_pos=0.0):
    rows = []
    t = 0.0
    p = start_pos
    while t <= duration_s + 1e-9:
        rows.append((round(t, 4), round(p, 6), velocity))
        t += dt
        p += velocity * dt
    return rows


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "scenarios/trajectory.csv"
    duration_s = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    dt = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
    velocity = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
    start_pos = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0

    rows = generate(duration_s, dt, velocity, start_pos)

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "p", "v"])
        writer.writerows(rows)

    print(f"Wrote {len(rows)} true trajectory points to {out_path}")


if __name__ == "__main__":
    main()
