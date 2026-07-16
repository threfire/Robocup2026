#!/usr/bin/env python3
"""将原始遥测 CSV 绘制为位置—时间与速度—时间 SVG 曲线。"""

import argparse
import csv
from pathlib import Path


def points(values, left, top, width, height, low, high):
    duration_s = max(values[-1][0], 0.001)
    return " ".join(
            f"{left + width * time_s / duration_s:.1f},"
            f"{top + height * (high - value) / (high - low):.1f}"
            for time_s, value in values)


def write_position_chart(output, x_values, y_values):
    low = min(value for _, value in x_values + y_values)
    high = max(value for _, value in x_values + y_values)
    if high - low < 0.001:
        high = low + 0.001
    margin = (high - low) * 0.08
    low -= margin
    high += margin
    output.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="650">
<rect width="100%" height="100%" fill="white"/>
<text x="70" y="35" font-size="24">x(t), y(t)</text>
<line x1="80" y1="70" x2="80" y2="570" stroke="black"/>
<line x1="80" y1="570" x2="1160" y2="570" stroke="black"/>
<polyline fill="none" stroke="#1677ff" stroke-width="2" points="{points(x_values, 80, 70, 1080, 500, low, high)}"/>
<polyline fill="none" stroke="#20a050" stroke-width="2" points="{points(y_values, 80, 70, 1080, 500, low, high)}"/>
<text x="100" y="620" fill="#1677ff" font-size="18">x(t)</text>
<text x="180" y="620" fill="#20a050" font-size="18">y(t)</text>
<text x="1080" y="620" font-size="18">time (s)</text>
<text x="10" y="85" font-size="16">{high:.3f} m</text>
<text x="10" y="570" font-size="16">{low:.3f} m</text>
</svg>''', encoding="utf-8")


def derive_speed(rows, start_ms):
    values = []
    previous = None
    for row in rows:
        time_s = (int(row["timestamp_ms"]) - start_ms) * 0.001
        if previous is None:
            speed = 0.0
        else:
            dt = max(time_s - previous[0], 0.001)
            speed = ((float(row["position_x_m"]) - previous[1]) ** 2 +
                    (float(row["position_y_m"]) - previous[2]) ** 2) ** 0.5 / dt
        values.append((time_s, speed))
        previous = (time_s, float(row["position_x_m"]), float(row["position_y_m"]))
    return values


def write_speed_chart(output, rows, start_ms):
    if "velocity_x_mps" in rows[0]:
        measured = [((int(row["timestamp_ms"]) - start_ms) * 0.001,
                (float(row["velocity_x_mps"]) ** 2 +
                float(row["velocity_y_mps"]) ** 2) ** 0.5) for row in rows]
        command = [((int(row["timestamp_ms"]) - start_ms) * 0.001,
                (float(row["command_vx_mps"]) ** 2 +
                float(row["command_vy_mps"]) ** 2) ** 0.5) for row in rows]
    else:
        measured = derive_speed(rows, start_ms)
        command = []
    high = max(value for _, value in measured + command)
    high = max(high * 1.08, 0.01)
    command_line = ""
    command_legend = ""
    if command:
        command_line = f'<polyline fill="none" stroke="#f59e0b" stroke-width="2" points="{points(command, 80, 70, 1080, 500, 0.0, high)}"/>'
        command_legend = '<text x="245" y="620" fill="#f59e0b" font-size="18">command speed</text>'
    output.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="650">
<rect width="100%" height="100%" fill="white"/>
<text x="70" y="35" font-size="24">speed(t)</text>
<line x1="80" y1="70" x2="80" y2="570" stroke="black"/>
<line x1="80" y1="570" x2="1160" y2="570" stroke="black"/>
<polyline fill="none" stroke="#1677ff" stroke-width="2" points="{points(measured, 80, 70, 1080, 500, 0.0, high)}"/>
{command_line}
<text x="100" y="620" fill="#1677ff" font-size="18">measured speed</text>
{command_legend}
<text x="1080" y="620" font-size="18">time (s)</text>
<text x="10" y="85" font-size="16">{high:.3f} m/s</text>
<text x="10" y="570" font-size="16">0.000 m/s</text>
</svg>''', encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("telemetry_csv", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--speed-output", type=Path)
    args = parser.parse_args()

    with args.telemetry_csv.open(encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError("遥测 CSV 没有数据。")

    start_ms = int(rows[0]["timestamp_ms"])
    x_values = [((int(row["timestamp_ms"]) - start_ms) * 0.001,
            float(row["position_x_m"])) for row in rows]
    y_values = [((int(row["timestamp_ms"]) - start_ms) * 0.001,
            float(row["position_y_m"])) for row in rows]
    output = args.output or args.telemetry_csv.with_suffix(".svg")
    speed_output = args.speed_output or args.telemetry_csv.with_name(
            args.telemetry_csv.stem + "_speed.svg")
    write_position_chart(output, x_values, y_values)
    write_speed_chart(speed_output, rows, start_ms)
    print(output)
    print(speed_output)


if __name__ == "__main__":
    main()
