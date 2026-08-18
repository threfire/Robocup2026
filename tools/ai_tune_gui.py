#!/usr/bin/env python3
"""坐标与偏航闭环固定路线实时观察界面。"""

import csv
import math
import os
import struct
import sys
import time
from collections import deque
from pathlib import Path

if getattr(sys, "frozen", False):
    _tk_runtime_dir = getattr(sys, "_MEIPASS", "")
    os.environ.setdefault("TCL_LIBRARY",
            os.path.join(_tk_runtime_dir, "_tcl_data"))
    os.environ.setdefault("TK_LIBRARY",
            os.path.join(_tk_runtime_dir, "_tk_data"))

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from ai_tuner import (ACK, BAUDRATE, CMD_START, TELEMETRY, FrameReader,
                      decode_ack, decode_telemetry, make_frame)


REFRESH_MS = 33
TEST_WINDOW_S = 10.0
TEST_WINDOW_MS = int(TEST_WINDOW_S * 1000.0)
START_X_MM = -800.0
START_Y_MM = 0.0
MAX_SAMPLES = 6000
MAX_PLOT_SAMPLES = 1200
MAX_STAT_SEGMENTS = 4
POSITION_PLOT_MIN_SPAN_M = 0.05
STAT_PLAN_SPEED_MPS = 0.01


class TuneWindow(tk.Tk):
    """串口测试启动和 x/y/yaw 实时曲线窗口。"""

    def __init__(self):
        super().__init__()
        self.title("底盘调参测试")
        self.minsize(1450, 760)
        self.serial_port = None
        self.reader = FrameReader()
        self.sequence = 1
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.session_sample_count = 0
        self.plot_dirty = False
        self.start_time_ms = None
        self.capture_active = False
        self.port_name = tk.StringVar(value="COM3")
        self.speed_mps = tk.StringVar(value="1.8")
        self.status_text = tk.StringVar(value="未连接")
        self._build_controls()
        self._build_plots()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(REFRESH_MS, self._poll_serial)

    def _build_controls(self):
        controls = ttk.Frame(self, padding=10)
        controls.pack(fill=tk.X)

        ttk.Label(controls, text="串口").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(controls, width=10,
                textvariable=self.port_name, state="normal")
        self.port_box.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Button(controls, text="刷新", command=self._refresh_ports).pack(
                side=tk.LEFT, padx=(0, 12))
        self.connect_button = ttk.Button(controls, text="连接",
                command=self._toggle_connection)
        self.connect_button.pack(side=tk.LEFT)
        ttk.Label(controls, text="测试速度 m/s").pack(side=tk.LEFT,
                padx=(20, 6))
        ttk.Entry(controls, width=8, textvariable=self.speed_mps).pack(
                side=tk.LEFT)
        self.start_button = ttk.Button(controls, text="启动固定测试路线",
                command=self._start_test, state=tk.DISABLED)
        self.start_button.pack(side=tk.LEFT, padx=12)
        ttk.Button(controls, text="保存 CSV", command=self._save_csv).pack(
                side=tk.LEFT)
        ttk.Label(controls, textvariable=self.status_text).pack(side=tk.RIGHT)
        self._refresh_ports()

    def _build_plots(self):
        content = ttk.Frame(self)
        content.pack(fill=tk.BOTH, expand=True)
        figure = Figure(figsize=(10, 11), dpi=100)
        axes_grid = figure.subplots(4, 3, sharex="col")
        self.axes = list(axes_grid.flat)
        labels = (("x", "m"), ("y", "m"), ("yaw", "deg"))
        self.position_actual_lines = []
        self.position_target_lines = []
        self.velocity_actual_lines = []
        self.velocity_command_lines = []
        self.accel_actual_lines = []
        self.control_lines = []
        velocity_labels = (("vx", "m/s"), ("vy", "m/s"),
                ("yaw rate", "deg/s"))
        accel_labels = (("ax", "m/s²"), ("ay", "m/s²"),
                ("yaw accel", "deg/s²"))
        for column, (name, unit) in enumerate(labels):
            position_axis = axes_grid[0, column]
            velocity_axis = axes_grid[1, column]
            accel_axis = axes_grid[2, column]
            control_axis = axes_grid[3, column]
            actual, = position_axis.plot([], [], color="#1677ff",
                    label="实际")
            target, = position_axis.plot([], [], color="#f59e0b",
                    linestyle="--", label="MPC参考")
            velocity, = velocity_axis.plot([], [], color="#1677ff",
                    label="实际")
            command, = velocity_axis.plot([], [], color="#f59e0b",
                    linestyle="--", label="指令(世界)")
            if column == 2:
                command.set_visible(False)
            accel, = accel_axis.plot([], [], color="#7c3aed",
                    label="实际")
            plan, = control_axis.plot([], [], color="#1677ff", label="plan")
            feedforward, = control_axis.plot([], [], color="#f59e0b",
                    label="feedforward")
            feedback, = control_axis.plot([], [], color="#dc2626",
                    label="feedback")
            velocity_feedback, = control_axis.plot([], [], color="#16a34a",
                    label="velocity feedback")
            position_axis.set_ylabel(f"{name} ({unit})")
            velocity_name, velocity_unit = velocity_labels[column]
            accel_name, accel_unit = accel_labels[column]
            velocity_axis.set_ylabel(f"{velocity_name} ({velocity_unit})")
            accel_axis.set_ylabel(f"{accel_name} ({accel_unit})")
            control_axis.set_ylabel(f"{name} control (m/s)")
            if column == 2:
                command.set_visible(False)
            for axis in (position_axis, velocity_axis, accel_axis, control_axis):
                axis.grid(True, alpha=0.25)
                axis.legend(loc="upper right", fontsize=8)
            self.position_actual_lines.append(actual)
            self.position_target_lines.append(target)
            self.velocity_actual_lines.append(velocity)
            self.velocity_command_lines.append(command)
            self.accel_actual_lines.append(accel)
            self.control_lines.append(
                    (plan, feedforward, feedback, velocity_feedback))
            accel_axis.set_xlabel("时间 (s)")
            control_axis.set_xlabel("time (s)")
        axes_grid[3, 2].set_visible(False)
        figure.tight_layout(pad=1.5)
        self.canvas = FigureCanvasTkAgg(figure, master=content)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH,
                expand=True, padx=(10, 4), pady=(0, 10))

        stats_frame = ttk.LabelFrame(content, text="分段速度统计", padding=6)
        stats_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(4, 10), pady=(0, 10))
        columns = ("segment", "target", "mean", "peak", "duration")
        self.stats_table = ttk.Treeview(stats_frame, columns=columns,
                show="headings", height=MAX_STAT_SEGMENTS)
        headings = (("segment", "段", 42), ("target", "目标", 145),
                ("mean", "匀速均值", 72), ("peak", "峰值", 62),
                ("duration", "耗时", 62))
        for column, heading, width in headings:
            self.stats_table.heading(column, text=heading)
            self.stats_table.column(column, width=width, anchor=tk.CENTER)
        self.stats_table.pack(fill=tk.Y, expand=True)

    def _refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.port_box["values"] = ports
        if self.port_name.get() not in ports and ports:
            self.port_name.set(ports[0])

    def _toggle_connection(self):
        if self.serial_port is not None:
            self._disconnect()
            return
        try:
            self.serial_port = serial.Serial(self.port_name.get(), BAUDRATE,
                    timeout=0)
        except serial.SerialException as error:
            messagebox.showerror("串口连接失败", str(error))
            return
        self.reader = FrameReader()
        self.connect_button.configure(text="断开")
        self.start_button.configure(state=tk.NORMAL)
        self.status_text.set(f"已连接 {self.port_name.get()}，等待遥测")

    def _disconnect(self):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None
        self.connect_button.configure(text="连接")
        self.start_button.configure(state=tk.DISABLED)
        self.status_text.set("未连接")

    def _start_test(self):
        try:
            speed_mps = float(self.speed_mps.get())
        except ValueError:
            messagebox.showerror("速度无效", "测试速度需为数字。")
            return
        if speed_mps < 0.20 or speed_mps > 7.0:
            messagebox.showerror("速度无效", "测试速度范围为 0.20 到 7.0 m/s。")
            return
        self.samples.clear()
        self.session_sample_count = 0
        self.start_time_ms = None
        self.capture_active = True
        self.plot_dirty = False
        self._update_segment_stats([])
        self._update_plots()
        frame = make_frame(CMD_START, struct.pack("<3f", START_X_MM,
                START_Y_MM, speed_mps), self.sequence)
        self.serial_port.write(frame)
        self.status_text.set("已发送固定测试路线启动指令")
        self.sequence += 1

    def _poll_serial(self):
        if self.serial_port is not None:
            try:
                count = self.serial_port.in_waiting
                data = self.serial_port.read(count) if count else b""
                for frame_type, _, payload in self.reader.feed(data):
                    self._handle_frame(frame_type, payload)
                if self.plot_dirty:
                    self.plot_dirty = False
                    self._update_plots()
            except serial.SerialException as error:
                self._disconnect()
                messagebox.showerror("串口已断开", str(error))
        self.after(REFRESH_MS, self._poll_serial)

    def _handle_frame(self, frame_type, payload):
        if frame_type == TELEMETRY:
            telemetry = decode_telemetry(payload)
            if telemetry is None:
                return
            if not self.capture_active:
                return
            if self.start_time_ms is None:
                self.start_time_ms = telemetry.timestamp_ms
            elif telemetry.timestamp_ms - self.start_time_ms > TEST_WINDOW_MS:
                self.capture_active = False
                self.plot_dirty = True
                self._update_segment_stats(list(self.samples))
                self.status_text.set(
                        f"测试已到 {TEST_WINDOW_S:g} 秒，曲线已停止")
                return
            self.samples.append(telemetry)
            self.session_sample_count += 1
            self.plot_dirty = True
            self.status_text.set(
                    f"实时接收：{self.session_sample_count} 帧，"
                    f"x={telemetry.position_x_m:.3f} m，"
                    f"y={telemetry.position_y_m:.3f} m，"
                    f"yaw={telemetry.yaw_deg:.1f}°")
        elif frame_type == ACK:
            ack = decode_ack(payload)
            if ack is not None:
                self.status_text.set(f"设备已确认启动指令，参数版本 {ack[2]}")

    def _update_segment_stats(self, samples):
        for item in self.stats_table.get_children():
            self.stats_table.delete(item)
        if not samples:
            return

        segments = []
        current = []
        for sample in samples:
            plan_speed_mps = math.hypot(sample.plan_vx_mps,
                    sample.plan_vy_mps)
            if plan_speed_mps >= STAT_PLAN_SPEED_MPS:
                current.append(sample)
            elif current:
                segments.append(current)
                current = []
        if current:
            segments.append(current)

        for index, segment in enumerate(segments[:MAX_STAT_SEGMENTS], start=1):
            speeds = [math.hypot(sample.velocity_x_mps,
                    sample.velocity_y_mps) for sample in segment]
            if not speeds:
                continue
            first, last = segment[0], segment[-1]
            duration_s = max(0.0, (last.timestamp_ms - first.timestamp_ms) *
                    0.001)
            target = (f"({last.target_x_m:.2f}, {last.target_y_m:.2f}, "
                    f"{last.target_yaw_deg:.0f}°)")
            self.stats_table.insert("", tk.END, values=(
                    index, target, f"{sum(speeds) / len(speeds):.3f}",
                    f"{max(speeds):.3f}", f"{duration_s:.2f} s"))

    def _update_plots(self):
        if not self.samples or self.start_time_ms is None:
            control_plot_lines = [line for group in self.control_lines
                    for line in group]
            lines = (self.position_actual_lines + self.position_target_lines +
                    self.velocity_actual_lines + self.velocity_command_lines +
                    self.accel_actual_lines + control_plot_lines)
            for line in lines:
                line.set_data([], [])
            self.canvas.draw_idle()
            return
        samples = list(self.samples)
        sample_step = max(1, (len(samples) + MAX_PLOT_SAMPLES - 1) //
                MAX_PLOT_SAMPLES)
        samples = samples[::sample_step]
        time_s = [(sample.timestamp_ms - self.start_time_ms) * 0.001
                for sample in samples]
        position_values = ([sample.position_x_m for sample in samples],
                [sample.position_y_m for sample in samples],
                [sample.yaw_deg for sample in samples])
        reference_x, reference_y = self._derive_mpc_reference(samples)
        target_values = (reference_x, reference_y,
                [sample.target_yaw_deg for sample in samples])
        velocity_values, command_values, accel_values = \
                self._derive_motion_series(samples)
        control_values = self._derive_control_series(samples)
        for column in range(3):
            position_axis = self.axes[column]
            velocity_axis = self.axes[3 + column]
            accel_axis = self.axes[6 + column]
            control_axis = self.axes[9 + column]
            self.position_actual_lines[column].set_data(
                    time_s, position_values[column])
            self.position_target_lines[column].set_data(
                    time_s, target_values[column])
            self.velocity_actual_lines[column].set_data(
                    time_s, velocity_values[column])
            self.velocity_command_lines[column].set_data(
                    time_s, command_values[column])
            self.accel_actual_lines[column].set_data(
                    time_s, accel_values[column])
            for line, values in zip(self.control_lines[column],
                    control_values[column]):
                line.set_data(time_s, values)
            for axis in (position_axis, velocity_axis, accel_axis):
                axis.relim()
                axis.autoscale_view()
                if not self.capture_active:
                    axis.set_xlim(0.0, TEST_WINDOW_S)
            if column < 2:
                self._fit_position_axis(position_axis, position_values[column],
                        target_values[column])
            if column < 2:
                control_axis.relim()
                control_axis.autoscale_view()
                if not self.capture_active:
                    control_axis.set_xlim(0.0, TEST_WINDOW_S)
        self.canvas.draw_idle()

    @staticmethod
    def _derive_mpc_reference(samples):
        return ([sample.reference_x_m for sample in samples],
                [sample.reference_y_m for sample in samples])

    @staticmethod
    def _fit_position_axis(axis, actual_values, reference_values):
        values = actual_values + reference_values
        lower = min(values)
        upper = max(values)
        span = max(upper - lower, POSITION_PLOT_MIN_SPAN_M)
        padding = max(span * 0.08, 0.005)

        axis.set_ylim(lower - padding, upper + padding)

    @staticmethod
    def _derive_control_series(samples):
        return (
                ([sample.plan_vx_mps for sample in samples],
                 [sample.feedforward_vx_mps for sample in samples],
                 [sample.feedback_vx_mps for sample in samples],
                 [sample.velocity_feedback_vx_mps for sample in samples]),
                ([sample.plan_vy_mps for sample in samples],
                 [sample.feedforward_vy_mps for sample in samples],
                 [sample.feedback_vy_mps for sample in samples],
                 [sample.velocity_feedback_vy_mps for sample in samples]),
                ([], [], [], []))

    @staticmethod
    def _derive_motion_series(samples):
        velocity_values = (
                [sample.velocity_x_mps for sample in samples],
                [sample.velocity_y_mps for sample in samples], [])
        command_x_world = []
        command_y_world = []
        for sample in samples:
            yaw_rad = math.radians(sample.yaw_deg)
            command_x_world.append(sample.command_vx_mps * math.cos(yaw_rad) -
                    sample.command_vy_mps * math.sin(yaw_rad))
            command_y_world.append(sample.command_vx_mps * math.sin(yaw_rad) +
                    sample.command_vy_mps * math.cos(yaw_rad))
        command_values = (command_x_world, command_y_world, [])
        yaw_rates = [0.0]
        accel_x = [0.0]
        accel_y = [0.0]
        yaw_accel = [0.0]
        for previous, current in zip(samples, samples[1:]):
            dt = (current.timestamp_ms - previous.timestamp_ms) * 0.001
            if dt <= 0.0:
                yaw_rates.append(yaw_rates[-1])
                accel_x.append(accel_x[-1])
                accel_y.append(accel_y[-1])
                yaw_accel.append(yaw_accel[-1])
                continue
            yaw_delta = (current.yaw_deg - previous.yaw_deg + 180.0) % \
                    360.0 - 180.0
            yaw_rate = yaw_delta / dt
            yaw_rates.append(yaw_rate)
            accel_x.append((current.velocity_x_mps -
                    previous.velocity_x_mps) / dt)
            accel_y.append((current.velocity_y_mps -
                    previous.velocity_y_mps) / dt)
            yaw_accel.append((yaw_rate - yaw_rates[-2]) / dt)
        velocity_values[2].extend(yaw_rates)
        command_values[2].extend([0.0] * len(samples))
        accel_values = (accel_x, accel_y, yaw_accel)
        return velocity_values, command_values, accel_values

    def _save_csv(self):
        if not self.samples:
            messagebox.showinfo("没有数据", "接收遥测后即可保存 CSV。")
            return
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        path = filedialog.asksaveasfilename(
                title="保存遥测 CSV",
                initialfile=f"ai_tune_gui_{timestamp}.csv",
                defaultextension=".csv",
                filetypes=[("CSV 文件", "*.csv")])
        if not path:
            return
        fieldnames = ("timestamp_ms", "version", "target_x_m", "target_y_m",
                "reference_x_m", "reference_y_m", "position_x_m", "position_y_m",
                "target_yaw_deg", "yaw_deg",
                "velocity_x_mps", "velocity_y_mps", "command_vx_mps",
                "command_vy_mps", "plan_accel_mps2", "plan_vx_mps",
                "plan_vy_mps", "feedforward_vx_mps", "feedforward_vy_mps",
                "feedback_vx_mps", "feedback_vy_mps",
                "velocity_feedback_vx_mps", "velocity_feedback_vy_mps",
                "position_reached", "guarded")
        with Path(path).open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            for sample in self.samples:
                writer.writerow({name: getattr(sample, name) for name in fieldnames})
        self.status_text.set(f"已保存 {len(self.samples)} 帧到 {path}")

    def _close(self):
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    TuneWindow().mainloop()
