#!/usr/bin/env python3
"""Fit the FFP twist correction tables in Joystick.c from joystick recordings.

The recordings are the CSV files the Andromeda engine's Input Devices panel writes
("Record" button): time_ms, axis0..axisN, buttons, hats, all raw SDL values. Record
them with firmware that has TWIST_DRIFT_CORRECTION 0 (hex 12), so the twist is raw.

  --centre FILE   Rest the stick for two seconds, then push it everywhere, into every
                  corner and along every edge, by the top of the handle with one
                  fingertip, without twisting. Gives twist_centre and the rest position.
  --stops FILE    Hold the stick at many positions (centre, edges, corners) and twist
                  the handle to both stops at each. Gives twist_low and twist_high.

Prints the twist_grid_x/_y and the three tables, in quarter counts, to paste into
Joystick.c. --grid sets the grid size (TWIST_GRID there, 5 by default): the middle
row and column sit at the rest position, the others evenly spaced to the ends.
Only the Python standard library is used.
"""

import argparse
import csv
import statistics
import sys

TWIST_STEP = 1057		# SDL raw units per twist count: 0..63 maps to about -32768..32767
AXIS_SCALE = 64			# SDL raw units per X/Y count: -512..511 maps to -32768..32767
REST_MS = 1000			# the first second of the centre recording is the rest position


def read_samples(path, x_axis, y_axis, twist_axis):
    """Returns (time_ms, x, y, twist) per row, in firmware units."""
    samples = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            x = max(-512, min(511, round(int(row[f"axis{x_axis}"]) / AXIS_SCALE)))
            y = max(-512, min(511, round(int(row[f"axis{y_axis}"]) / AXIS_SCALE)))
            twist = max(0, min(63, round(int(row[f"axis{twist_axis}"]) / TWIST_STEP) + 32))
            samples.append((int(row["time_ms"]), x, y, twist))
    if not samples:
        sys.exit(f"{path}: no rows")
    return samples


def grid_through(rest, n):
    """n grid values from -512 to 511 with the middle one at rest."""
    half = n // 2
    return ([round(-512 + (rest + 512) * i / half) for i in range(half)]
            + [round(rest + (511 - rest) * i / half) for i in range(half + 1)])


def cell(grid, value):
    """The index of the grid cell value falls in, as Joystick.c's TwistCell."""
    c = len(grid) - 2
    while c > 0 and value < grid[c]:
        c -= 1
    return c


def weights(x, y, grid_x, grid_y):
    """The bilinear weights of the four nodes around (x, y), as Joystick.c's TwistTable."""
    col = cell(grid_x, x)
    row = cell(grid_y, y)
    fx = (x - grid_x[col]) / (grid_x[col + 1] - grid_x[col])
    fy = (y - grid_y[row]) / (grid_y[row + 1] - grid_y[row])
    return [((row, col), (1 - fx) * (1 - fy)), ((row, col + 1), fx * (1 - fy)),
            ((row + 1, col), (1 - fx) * fy), ((row + 1, col + 1), fx * fy)]


def solve(a, b):
    """Solves a x = b by Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(m[r][i]))
        m[i], m[p] = m[p], m[i]
        for r in range(i + 1, n):
            f = m[r][i] / m[i][i]
            for c in range(i, n + 1):
                m[r][c] -= f * m[i][c]
    x = [0.0] * n
    for i in reversed(range(n)):
        x[i] = (m[i][n] - sum(m[i][c] * x[c] for c in range(i + 1, n))) / m[i][i]
    return x


def fit_centre(samples, grid_x, grid_y, ridge):
    """Least-squares node values; a small ridge pulls nodes without data to the mean."""
    n = len(grid_x)
    mean = statistics.fmean(s[3] for s in samples)
    a = [[0.0] * (n * n) for _ in range(n * n)]
    b = [0.0] * (n * n)
    coverage = [0.0] * (n * n)
    for _, x, y, twist in samples:
        w = [(r * n + c, v) for (r, c), v in weights(x, y, grid_x, grid_y)]
        for i, wi in w:
            coverage[i] += wi
            b[i] += wi * twist
            for j, wj in w:
                a[i][j] += wi * wj
    for i in range(n * n):
        a[i][i] += ridge
        b[i] += ridge * mean
    return solve(a, b), coverage


def fit_stops(samples, grid_x, grid_y):
    """Per node, the lowest and highest twist among samples close to that node."""
    n = len(grid_x)
    near = [[] for _ in range(n * n)]
    for _, x, y, twist in samples:
        for (r, c), v in weights(x, y, grid_x, grid_y):
            if v >= 0.5:
                near[r * n + c].append(twist)
    low, high = [], []
    for values in near:
        if len(values) < 5:
            low.append(None)
            high.append(None)
            continue
        values.sort()
        k = len(values) // 100		# drop the outer 1 % as glitches
        low.append(values[k])
        high.append(values[-1 - k])
    return low, high, [len(v) for v in near]


def c_table(name, values, n, note):
    """values in counts, printed in quarter counts as Joystick.c stores them."""
    lines = [f"static const uint8_t {name}[TWIST_GRID][TWIST_GRID] PROGMEM =", "\t{"]
    labels = {0: "forward", n // 2: "rest", n - 1: "back"}
    for r in range(n):
        row = ", ".join(f"{max(0, min(252, round(v * 4))):3d}" for v in values[r * n:r * n + n])
        label = f"\t// {labels[r]}" if r in labels else ""
        lines.append(f"\t{{ {row} }},{label}")
    lines.append("\t};")
    return f"// {note}\n" + "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--centre", required=True, help="recording with the handle untouched")
    parser.add_argument("--stops", help="recording twisting to both stops at many positions")
    parser.add_argument("--x-axis", type=int, default=0)
    parser.add_argument("--y-axis", type=int, default=1)
    parser.add_argument("--twist-axis", type=int, default=5)
    parser.add_argument("--ridge", type=float, default=0.5, help="pull of nodes without data towards the mean")
    parser.add_argument("--grid", type=int, default=5, help="grid size, odd (TWIST_GRID in Joystick.c)")
    args = parser.parse_args()

    centre_samples = read_samples(args.centre, args.x_axis, args.y_axis, args.twist_axis)
    rest = [s for s in centre_samples if s[0] - centre_samples[0][0] < REST_MS]
    rest_x = round(statistics.median(s[1] for s in rest))
    rest_y = round(statistics.median(s[2] for s in rest))
    n = args.grid
    if n < 3 or n % 2 == 0:
        sys.exit("--grid must be odd and at least 3")
    grid_x = grid_through(rest_x, n)
    grid_y = grid_through(rest_y, n)

    centre, coverage = fit_centre(centre_samples, grid_x, grid_y, args.ridge)
    centre = [max(0.0, min(63.0, v)) for v in centre]

    if args.stops:
        stop_samples = read_samples(args.stops, args.x_axis, args.y_axis, args.twist_axis)
        low, high, counts = fit_stops(stop_samples, grid_x, grid_y)
    else:
        low, high, counts = [None] * (n * n), [None] * (n * n), [0] * (n * n)

    print(f"// {len(centre_samples)} centre samples, rest at x {rest_x}, y {rest_y}")
    print("// node coverage (centre weight / stop samples), rows forward to back:")
    for r in range(n):
        print("//  " + "  ".join(f"{coverage[r * n + c]:7.1f} / {counts[r * n + c]:5d}" for c in range(n)))
    missing = [i for i in range(n * n) if low[i] is None]
    if args.stops and missing:
        print(f"// no stop data near nodes {missing}: 0 and 63 used there; record those positions too")
    low = [0 if v is None else v for v in low]
    high = [63 if v is None else v for v in high]
    for i in range(n * n):
        if not low[i] < centre[i] < high[i]:
            print(f"// warning: node {i} has low {low[i]}, centre {centre[i]:.2f}, high {high[i]} out of order")
    print()
    print(f"#define TWIST_GRID\t\t\t\t\t{n}")
    print()
    print("static const int16_t twist_grid_x[TWIST_GRID] PROGMEM = { " + ", ".join(map(str, grid_x)) + " };")
    print("static const int16_t twist_grid_y[TWIST_GRID] PROGMEM = { " + ", ".join(map(str, grid_y)) + " };")
    print()
    print(c_table("twist_low", low, n, "raw twist at the left stop"))
    print()
    print(c_table("twist_centre", centre, n, "raw twist with the handle untouched"))
    print()
    print(c_table("twist_high", high, n, "raw twist at the right stop"))


if __name__ == "__main__":
    main()
