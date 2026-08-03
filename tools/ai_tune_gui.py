#!/usr/bin/env python3
"""坐标与偏航闭环固定路线实时观察界面。"""

import csv
import struct
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from ai_tuner import (ACK, BAUDRATE, CMD_START, TELEMETRY, FrameReader,
                      decode_ack, decode_telemetry, make_frame)


REFRESH_MS = 33
START_X_MM = -800.0
START_Y_MM = 0.0
MAX_SAMPLES = 6000
MAX_PLOT_SAMPLES = 1200


class TuneWindow(tk.Tk):
    """串口测试启动和 x/y/yaw 实时曲线窗口。"""

    def __init__(self):
        super().__init__()
        self.title("底盘调参测试")
        self.minsize(1040, 760)
        self.serial_port = None
        self.reader = FrameReader()
        self.sequence = 1
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.session_sample_count = 0
        self.plot_dirty = False
        self.start_time_ms = None
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
        figure = Figure(figsize=(10, 7), dpi=100)
        self.axes = [figure.add_subplot(311), figure.add_subplot(312),
                figure.add_subplot(313)]
        labels = (("x", "m"), ("y", "m"), ("yaw", "deg"))
        self.actual_lines = []
        self.target_lines = []
        for axis, (name, unit) in zip(self.axes, labels):
            actual, = axis.plot([], [], color="#1677ff", label="实际")
            target, = axis.plot([], [], color="#f59e0b", linestyle="--",
                    label="目标")
            axis.set_ylabel(f"{name} ({unit})")
            axis.grid(True, alpha=0.25)
            axis.legend(loc="upper right")
            self.actual_lines.append(actual)
            self.target_lines.append(target)
        self.axes[-1].set_xlabel("时间 (s)")
        figure.tight_layout(pad=2.0)
        self.canvas = FigureCanvasTkAgg(figure, master=self)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True,
                padx=10, pady=(0, 10))

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
            if self.start_time_ms is None:
                self.start_time_ms = telemetry.timestamp_ms
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

    def _update_plots(self):
        if not self.samples or self.start_time_ms is None:
            for line in self.actual_lines + self.target_lines:
                line.set_data([], [])
            self.canvas.draw_idle()
            return
        samples = list(self.samples)
        sample_step = max(1, (len(samples) + MAX_PLOT_SAMPLES - 1) //
                MAX_PLOT_SAMPLES)
        samples = samples[::sample_step]
        time_s = [(sample.timestamp_ms - self.start_time_ms) * 0.001
                for sample in samples]
        actual_values = ([sample.position_x_m for sample in samples],
                [sample.position_y_m for sample in samples],
                [sample.yaw_deg for sample in samples])
        target_values = ([sample.target_x_m for sample in samples],
                [sample.target_y_m for sample in samples],
                [sample.target_yaw_deg for sample in samples])
        for axis, actual, target, actual_line, target_line in zip(self.axes,
                actual_values, target_values, self.actual_lines, self.target_lines):
            actual_line.set_data(time_s, actual)
            target_line.set_data(time_s, target)
            axis.relim()
            axis.autoscale_view()
        self.canvas.draw_idle()

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
                "position_x_m", "position_y_m", "target_yaw_deg", "yaw_deg",
                "velocity_x_mps", "velocity_y_mps", "command_vx_mps",
                "command_vy_mps", "plan_accel_mps2", "position_reached",
                "guarded")
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
