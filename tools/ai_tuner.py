#!/usr/bin/env python3
"""坐标闭环串口调参与在线评分程序。"""

import argparse
import csv
import math
import random
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial

HEAD = b"\xC3\x3C"
CMD_PARAM = 0x01
CMD_COMMIT = 0x02
CMD_START = 0x03
TELEMETRY = 0x81
ACK = 0x82
BAUDRATE = 2_000_000
TELEMETRY_PAYLOAD_SIZE = 44
ACK_PAYLOAD_SIZE = 6
EPISODE_MIN_DISTANCE_M = 0.05
PARAM_NAMES = ("kp", "kd", "accel_feedback_gain", "force_to_speed_ff", "static_friction_speed_mps")
LIMITS = ((2.0, 10.0), (0.0, 0.10), (0.0, 0.40), (0.0, 0.040), (0.04, 0.16))
BASELINE_VALUES = (6.50, 0.0555565, 0.274512, 0.0103671, 0.0500433)
SEARCH_SPANS = (1.2, 0.015, 0.08, 0.004, 0.015)
SEARCH_TARGET_VALUES = (7.2, 0.070, 0.34, 0.014, 0.060)
STOP_SPEED_MPS = 0.05
STOP_HOLD_S = 0.30
SPEED_ERROR_WEIGHT = 5.0
TOTAL_TIME_WEIGHT = 2.0
MEAN_SPEED_WEIGHT = 2.0
TIMING_WEIGHT = 1.0
OSCILLATION_WEIGHT = 4.0
PLATEAU_ACCEPT_MIN_RATIO = 0.90
PLATEAU_ACCEPT_MAX_RATIO = 1.05
STOP_DISTANCE_MAX_M = 0.35
OPTIMIZER_WARMUP_ITERATIONS = 4
TRAJECTORY_START = (-800.0, 0.0)
TRAJECTORY_END = (0.0, 0.0)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def make_frame(frame_type: int, payload: bytes, sequence: int) -> bytes:
    body = struct.pack("<BBH", frame_type, len(payload), sequence) + payload
    return HEAD + body + struct.pack("<H", crc16(body))


@dataclass(frozen=True)
class Parameters:
    version: int
    kp: float
    kd: float
    accel_feedback_gain: float
    force_to_speed_ff: float
    static_friction_speed_mps: float

    def values(self):
        return (self.kp, self.kd, self.accel_feedback_gain,
                self.force_to_speed_ff, self.static_friction_speed_mps)


@dataclass
class Telemetry:
    timestamp_ms: int
    version: int
    target_x_m: float
    target_y_m: float
    position_x_m: float
    position_y_m: float
    target_yaw_deg: float
    yaw_deg: float
    error_x_m: float
    error_y_m: float
    velocity_x_mps: float
    velocity_y_mps: float
    command_vx_mps: float
    command_vy_mps: float
    plan_accel_mps2: float
    position_reached: int
    guarded: int


@dataclass
class Evaluation:
    parameters: Parameters
    score: float


@dataclass
class EpisodeResult:
    score: float
    actual_peak_mps: float
    plateau_mean_mps: float
    stopping_distance_m: float
    total_time_s: float
    mean_actual_mps: float
    rise_time_s: float
    stop_time_s: float
    guarded: int


@dataclass
class Trial:
    parameters: Parameters
    result: EpisodeResult


class FrameReader:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 8:
            head_index = self.buffer.find(HEAD)
            if head_index < 0:
                self.buffer.clear()
                break
            if head_index:
                del self.buffer[:head_index]
            if len(self.buffer) < 8:
                break
            payload_size = self.buffer[3]
            frame_size = payload_size + 8
            if len(self.buffer) < frame_size:
                break
            frame = bytes(self.buffer[:frame_size])
            del self.buffer[:frame_size]
            body = frame[2:-2]
            if crc16(body) == struct.unpack_from("<H", frame, frame_size - 2)[0]:
                sequence = struct.unpack_from("<H", frame, 4)[0]
                frames.append((frame[2], sequence, frame[6:-2]))
        return frames


def decode_telemetry(payload: bytes) -> Telemetry | None:
    if len(payload) != TELEMETRY_PAYLOAD_SIZE:
        return None
    timestamp_ms, version = struct.unpack_from("<II", payload)
    target_x_mm, target_y_mm, position_x_mm, position_y_mm = \
            struct.unpack_from("<iiii", payload, 8)
    target_yaw_mdeg, yaw_mdeg = struct.unpack_from("<ii", payload, 24)
    reached, guarded = struct.unpack_from("<BB", payload, 42)
    target_x_m = target_x_mm * 0.001
    target_y_m = target_y_mm * 0.001
    position_x_m = position_x_mm * 0.001
    position_y_m = position_y_mm * 0.001
    velocity_x_mmps, velocity_y_mmps, command_vx_mmps, command_vy_mmps, \
            plan_accel_mmps2 = struct.unpack_from("<hhhhh", payload, 32)
    return Telemetry(timestamp_ms, version, target_x_m, target_y_m,
            position_x_m, position_y_m, target_yaw_mdeg * 0.001,
            yaw_mdeg * 0.001, target_x_m - position_x_m,
            target_y_m - position_y_m, velocity_x_mmps * 0.001,
            velocity_y_mmps * 0.001, command_vx_mmps * 0.001,
            command_vy_mmps * 0.001, plan_accel_mmps2 * 0.001, reached,
            guarded)


def decode_ack(payload: bytes):
    if len(payload) != ACK_PAYLOAD_SIZE:
        return None
    command, status, version = struct.unpack("<BBI", payload)
    return command, status, version


class KernelOptimizer:
    def __init__(self):
        self.history: list[Evaluation] = []

    def add(self, evaluation: Evaluation):
        self.history.append(evaluation)

    def sample(self, version: int) -> Parameters:
        values = []
        for baseline, target, span, (low, high) in zip(BASELINE_VALUES,
                SEARCH_TARGET_VALUES, SEARCH_SPANS, LIMITS):
            directed = random.uniform(0.45, 1.0) * (target - baseline)
            jitter = random.uniform(-0.25, 0.25) * span
            values.append(min(high, max(low, baseline + directed + jitter)))
        return Parameters(version, *values)

    def predict(self, candidate: Parameters) -> tuple[float, float]:
        weights = []
        scores = []
        for evaluation in self.history:
            distance = sum(((a - b) / (high - low)) ** 2 for a, b, (low, high)
                    in zip(candidate.values(), evaluation.parameters.values(), LIMITS))
            weight = math.exp(-4.0 * distance)
            weights.append(weight)
            scores.append(evaluation.score)
        weight_sum = sum(weights)
        if weight_sum < 1e-6:
            return 100.0, 100.0
        mean = sum(weight * score for weight, score in zip(weights, scores)) / weight_sum
        variance = sum(weight * (score - mean) ** 2
                for weight, score in zip(weights, scores)) / weight_sum
        return mean, math.sqrt(variance + 1.0 / weight_sum)

    def propose(self, version: int) -> Parameters:
        if len(self.history) < OPTIMIZER_WARMUP_ITERATIONS:
            return self.sample(version)
        candidates = [self.sample(version) for _ in range(256)]
        return max(candidates,
                key=lambda candidate: self.predict(candidate)[0] + 0.8 * self.predict(candidate)[1])


def score_episode(samples: list[Telemetry], target_speed_mps: float) -> EpisodeResult:
    first = samples[0]
    last = samples[-1]
    target_distance = math.hypot(last.target_x_m - last.position_x_m,
            last.target_y_m - last.position_y_m)
    distance_integral = 0.0
    speed_error_integral = 0.0
    actual_speed_integral = 0.0
    overshoot = 0.0
    oscillations = 0
    target_dx = first.target_x_m - first.position_x_m
    target_dy = first.target_y_m - first.position_y_m
    target_length = math.hypot(target_dx, target_dy)
    cruise_threshold_mps = target_speed_mps * 0.90
    actual_peak_mps = 0.0
    plateau_speeds = []
    rise_timestamp_ms = None
    last_cruise_timestamp_ms = None
    braking_progress_m = None
    cruise_seen = False
    previous_sign_x = 0
    previous_sign_y = 0
    for previous, current in zip(samples, samples[1:]):
        dt = max(0.0, (current.timestamp_ms - previous.timestamp_ms) * 0.001)
        actual_speed_mps = math.hypot(current.velocity_x_mps,
                current.velocity_y_mps)
        command_speed_mps = math.hypot(current.command_vx_mps,
                current.command_vy_mps)
        actual_peak_mps = max(actual_peak_mps, actual_speed_mps)
        actual_speed_integral += actual_speed_mps * dt
        distance_integral += math.hypot(current.target_x_m - current.position_x_m,
                current.target_y_m - current.position_y_m) * dt
        if command_speed_mps >= cruise_threshold_mps:
            cruise_seen = True
            plateau_speeds.append(actual_speed_mps)
            speed_error_integral += abs(actual_speed_mps - command_speed_mps) * dt
            last_cruise_timestamp_ms = current.timestamp_ms
        elif cruise_seen and braking_progress_m is None and target_length > 1e-4:
            braking_progress_m = ((current.position_x_m - first.position_x_m) *
                    target_dx + (current.position_y_m - first.position_y_m) *
                    target_dy) / target_length
        if rise_timestamp_ms is None and actual_speed_mps >= cruise_threshold_mps:
            rise_timestamp_ms = current.timestamp_ms
        if target_length > 1e-4:
            progress = ((current.position_x_m - first.position_x_m) * target_dx +
                    (current.position_y_m - first.position_y_m) * target_dy) / target_length
            overshoot = max(overshoot, progress - target_length)
        sign_x = 1 if current.error_x_m > 0.002 else -1 if current.error_x_m < -0.002 else 0
        sign_y = 1 if current.error_y_m > 0.002 else -1 if current.error_y_m < -0.002 else 0
        if sign_x and previous_sign_x and sign_x != previous_sign_x:
            oscillations += 1
        if sign_y and previous_sign_y and sign_y != previous_sign_y:
            oscillations += 1
        if sign_x:
            previous_sign_x = sign_x
        if sign_y:
            previous_sign_y = sign_y
    settle_time = (last.timestamp_ms - first.timestamp_ms) * 0.001
    guard_penalty = 1 if last.guarded else 0
    rise_time_s = settle_time if rise_timestamp_ms is None else \
            (rise_timestamp_ms - first.timestamp_ms) * 0.001
    stop_time_s = settle_time if last_cruise_timestamp_ms is None else \
            (last.timestamp_ms - last_cruise_timestamp_ms) * 0.001
    plateau_mean_mps = sum(plateau_speeds) / len(plateau_speeds) \
            if plateau_speeds else 0.0
    mean_actual_mps = actual_speed_integral / settle_time \
            if settle_time > 1e-6 else 0.0
    stopping_distance_m = target_length if braking_progress_m is None else \
            max(0.0, target_length - braking_progress_m)
    score = -(4.0 * target_distance + 1.5 * distance_integral +
            TOTAL_TIME_WEIGHT * settle_time + 2.5 * overshoot +
            OSCILLATION_WEIGHT * oscillations +
            8.0 * guard_penalty + SPEED_ERROR_WEIGHT * speed_error_integral +
            TIMING_WEIGHT * (rise_time_s + stop_time_s)) + \
            MEAN_SPEED_WEIGHT * mean_actual_mps
    return EpisodeResult(score, actual_peak_mps, plateau_mean_mps,
            stopping_distance_m, settle_time, mean_actual_mps, rise_time_s,
            stop_time_s, guard_penalty)


def score_trajectory(samples: list[Telemetry], target_speed_mps: float) -> EpisodeResult:
    segments: list[list[Telemetry]] = []
    current_segment: list[Telemetry] = []
    previous_target = None

    for sample in samples:
        target = (sample.target_x_m, sample.target_y_m)
        if previous_target is not None and (
                abs(target[0] - previous_target[0]) > 0.001 or
                abs(target[1] - previous_target[1]) > 0.001):
            if current_segment:
                segments.append(current_segment)
            current_segment = []
        current_segment.append(sample)
        previous_target = target
    if current_segment:
        segments.append(current_segment)

    results = [score_episode(segment, target_speed_mps)
            for segment in segments if len(segment) >= 2]
    if not results:
        return score_episode(samples, target_speed_mps)

    count = len(results)
    return EpisodeResult(
            sum(result.score for result in results) / count,
            max(result.actual_peak_mps for result in results),
            sum(result.plateau_mean_mps for result in results) / count,
            max(result.stopping_distance_m for result in results),
            sum(result.total_time_s for result in results) / count,
            sum(result.mean_actual_mps for result in results) / count,
            sum(result.rise_time_s for result in results) / count,
            sum(result.stop_time_s for result in results) / count,
            max(result.guarded for result in results))


def result_accepted(result: EpisodeResult, target_speed_mps: float) -> bool:
    return (result.guarded == 0 and
            result.plateau_mean_mps >= target_speed_mps * PLATEAU_ACCEPT_MIN_RATIO and
            result.plateau_mean_mps <= target_speed_mps * PLATEAU_ACCEPT_MAX_RATIO and
            result.stopping_distance_m <= STOP_DISTANCE_MAX_M)


def merge_results(first: EpisodeResult, second: EpisodeResult) -> EpisodeResult:
    return EpisodeResult(
            (first.score + second.score) * 0.5,
            max(first.actual_peak_mps, second.actual_peak_mps),
            (first.plateau_mean_mps + second.plateau_mean_mps) * 0.5,
            max(first.stopping_distance_m, second.stopping_distance_m),
            (first.total_time_s + second.total_time_s) * 0.5,
            (first.mean_actual_mps + second.mean_actual_mps) * 0.5,
            (first.rise_time_s + second.rise_time_s) * 0.5,
            (first.stop_time_s + second.stop_time_s) * 0.5,
            max(first.guarded, second.guarded))


def send_parameters(port, parameters: Parameters, sequence: int):
    payload = struct.pack("<I5f", parameters.version, *parameters.values())
    port.write(make_frame(CMD_PARAM, payload, sequence))


def send_start(port, target_x_mm: float, target_y_mm: float, speed_mps: float,
        sequence: int):
    port.write(make_frame(CMD_START, struct.pack("<3f", target_x_mm,
            target_y_mm, speed_mps), sequence))


def send_commit(port, version: int, sequence: int):
    port.write(make_frame(CMD_COMMIT, struct.pack("<I", version), sequence))


def wait_for_ack(port, reader: FrameReader, command: int, sequence: int,
        timeout_s: float = 0.40):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for frame_type, frame_sequence, payload in reader.feed(
                read_serial_chunk(port)):
            if frame_type != ACK or frame_sequence != sequence:
                continue
            ack = decode_ack(payload)
            if ack is None or ack[0] != command:
                continue
            if ack[1] != 0:
                raise RuntimeError(
                        f"firmware rejected command={command} status={ack[1]}")
            return ack[2]
    raise RuntimeError(
            f"no ACK for command={command} sequence={sequence}; "
            "telemetry may be TX-only or UART5 RX is unavailable")


def send_with_ack(port, reader: FrameReader, command: int, frame: bytes,
        sequence: int):
    for _ in range(3):
        port.write(frame)
        try:
            return wait_for_ack(port, reader, command, sequence)
        except RuntimeError as error:
            if "firmware rejected" in str(error):
                raise
    raise RuntimeError(
            f"command transaction failed: command={command}, "
            f"sequence={sequence}")


def start_trial(port, reader: FrameReader, parameters: Parameters,
        speed_mps: float, sequence: int) -> int:
    send_with_ack(port, reader, CMD_PARAM,
            make_frame(CMD_PARAM,
                    struct.pack("<I5f", parameters.version,
                            *parameters.values()), sequence), sequence)
    sequence += 1
    send_with_ack(port, reader, CMD_START,
            make_frame(CMD_START,
                    struct.pack("<3f", *TRAJECTORY_START, speed_mps),
                    sequence), sequence)
    return sequence + 1


def read_serial_chunk(port):
    pending = port.in_waiting
    return port.read(min(pending, 4096) if pending else 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--speed", type=float, default=1.8)
    parser.add_argument("--iterations", type=int, default=12)
    parser.add_argument("--validate-top", type=int, default=3)
    parser.add_argument("--csv", type=Path, default=Path("ai_tune.csv"))
    parser.add_argument("--telemetry-csv", type=Path)
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error("--iterations 必须大于 0")
    if args.validate_top < 1:
        parser.error("--validate-top 必须大于 0")

    optimizer = KernelOptimizer()
    reader = FrameReader()
    sequence = 0
    version = 1
    exploration_trials: list[Trial] = []
    validation_trials: list[Trial] = []
    validation_queue: list[Trial] = []
    validation_index = 0
    phase = "explore"
    active_samples: list[Telemetry] = []
    episode_active = False
    guard_seen = False
    stop_hold_s = 0.0
    last_telemetry = None
    active_params = Parameters(version, *BASELINE_VALUES)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    telemetry_csv = args.telemetry_csv or args.csv.with_name(
            args.csv.stem + "_telemetry.csv")

    with serial.Serial(args.port, BAUDRATE, timeout=0.05) as port, \
            args.csv.open("w", newline="", encoding="utf-8", buffering=1) as csv_file, \
            telemetry_csv.open("w", newline="", encoding="utf-8", buffering=1) as telemetry_file:
        writer = csv.DictWriter(csv_file, fieldnames=["phase", "version", "score",
                "actual_peak_mps", "plateau_mean_mps", "stopping_distance_m",
                "total_time_s", "mean_actual_mps",
                "rise_time_s", "stop_time_s", "accepted", *PARAM_NAMES])
        telemetry_writer = csv.DictWriter(telemetry_file, fieldnames=[
                "version", "timestamp_ms", "target_x_m", "target_y_m",
                "position_x_m", "position_y_m", "error_x_m", "error_y_m",
                "target_yaw_deg", "yaw_deg",
                "velocity_x_mps", "velocity_y_mps", "command_vx_mps",
                "command_vy_mps", "plan_accel_mps2",
                "position_reached", "guarded"])
        writer.writeheader()
        telemetry_writer.writeheader()
        initial_telemetry = None
        initial_deadline = time.monotonic() + 3.0
        while time.monotonic() < initial_deadline:
            for frame_type, frame_sequence, payload in reader.feed(
                    read_serial_chunk(port)):
                if frame_type == TELEMETRY:
                    initial_telemetry = decode_telemetry(payload)
        if initial_telemetry is None:
            raise RuntimeError("未收到 UART5 遥测帧。")
        print("telemetry connected", flush=True)
        sequence = start_trial(port, reader, active_params, args.speed,
                sequence)

        while phase != "complete":
            for frame_type, frame_sequence, payload in reader.feed(
                    read_serial_chunk(port)):
                if frame_type != TELEMETRY:
                    continue
                telemetry = decode_telemetry(payload)
                if telemetry is None:
                    continue
                last_telemetry = telemetry
                telemetry_writer.writerow({
                        "version": telemetry.version,
                        "timestamp_ms": telemetry.timestamp_ms,
                        "target_x_m": telemetry.target_x_m,
                        "target_y_m": telemetry.target_y_m,
                        "position_x_m": telemetry.position_x_m,
                        "position_y_m": telemetry.position_y_m,
                        "target_yaw_deg": telemetry.target_yaw_deg,
                        "yaw_deg": telemetry.yaw_deg,
                        "error_x_m": telemetry.error_x_m,
                        "error_y_m": telemetry.error_y_m,
                        "velocity_x_mps": telemetry.velocity_x_mps,
                        "velocity_y_mps": telemetry.velocity_y_mps,
                        "command_vx_mps": telemetry.command_vx_mps,
                        "command_vy_mps": telemetry.command_vy_mps,
                        "plan_accel_mps2": telemetry.plan_accel_mps2,
                        "position_reached": telemetry.position_reached,
                        "guarded": telemetry.guarded})
                if (telemetry.version != active_params.version and
                        not telemetry.guarded and not guard_seen):
                    continue
                if telemetry.guarded:
                    guard_seen = True
                distance_to_target = math.hypot(telemetry.error_x_m,
                        telemetry.error_y_m)
                if (abs(telemetry.target_x_m - TRAJECTORY_START[0] * 0.001) <=
                        0.001 and abs(telemetry.target_y_m -
                        TRAJECTORY_START[1] * 0.001) <= 0.001 and
                        distance_to_target >= EPISODE_MIN_DISTANCE_M):
                    episode_active = True
                if telemetry.guarded:
                    episode_active = True
                if not episode_active:
                    continue
                active_samples.append(telemetry)
                speed_mps = math.hypot(telemetry.velocity_x_mps,
                        telemetry.velocity_y_mps)
                if len(active_samples) >= 2:
                    dt = max(0.0, (active_samples[-1].timestamp_ms -
                            active_samples[-2].timestamp_ms) * 0.001)
                else:
                    dt = 0.0
                if (((abs(telemetry.target_x_m - TRAJECTORY_END[0] * 0.001) <=
                        0.001 and abs(telemetry.target_y_m -
                        TRAJECTORY_END[1] * 0.001) <= 0.001 and
                        telemetry.position_reached) or telemetry.guarded) and
                        speed_mps <= STOP_SPEED_MPS):
                    stop_hold_s += dt
                else:
                    stop_hold_s = 0.0
                if stop_hold_s < STOP_HOLD_S:
                    continue

                episode_result = score_trajectory(active_samples, args.speed)
                active_samples.clear()
                stop_hold_s = 0.0
                if phase == "explore":
                    exploration_trials.append(Trial(active_params, episode_result))
                    optimizer.add(Evaluation(active_params, episode_result.score))
                    result = episode_result
                    writer.writerow({"phase": phase, "version": active_params.version,
                            "score": result.score,
                            "actual_peak_mps": result.actual_peak_mps,
                            "plateau_mean_mps": result.plateau_mean_mps,
                            "stopping_distance_m": result.stopping_distance_m,
                            "total_time_s": result.total_time_s,
                            "mean_actual_mps": result.mean_actual_mps,
                            "rise_time_s": result.rise_time_s,
                            "stop_time_s": result.stop_time_s,
                            "accepted": 0,
                            **dict(zip(PARAM_NAMES, active_params.values()))})
                    print(f"phase={phase} version={active_params.version} "
                            f"score={result.score:.4f} "
                            f"plateau={result.plateau_mean_mps:.3f} "
                            f"total={result.total_time_s:.3f} "
                            f"mean={result.mean_actual_mps:.3f} "
                            f"stop={result.stopping_distance_m:.3f} "
                            f"guarded={result.guarded}", flush=True)
                    if len(exploration_trials) >= args.iterations:
                        validation_queue = sorted(exploration_trials,
                                key=lambda item: item.result.score,
                                reverse=True)[:args.validate_top]
                        phase = "validate"
                        validation_index = 0
                        active_params = validation_queue[validation_index].parameters
                    else:
                        version += 1
                        active_params = optimizer.propose(version)
                else:
                    first_trial = validation_queue[validation_index]
                    result = merge_results(first_trial.result, episode_result)
                    validation_trials.append(Trial(first_trial.parameters, result))
                    accepted = result_accepted(result, args.speed)
                    writer.writerow({"phase": phase, "version": active_params.version,
                            "score": result.score,
                            "actual_peak_mps": result.actual_peak_mps,
                            "plateau_mean_mps": result.plateau_mean_mps,
                            "stopping_distance_m": result.stopping_distance_m,
                            "total_time_s": result.total_time_s,
                            "mean_actual_mps": result.mean_actual_mps,
                            "rise_time_s": result.rise_time_s,
                            "stop_time_s": result.stop_time_s,
                            "accepted": int(accepted),
                            **dict(zip(PARAM_NAMES, active_params.values()))})
                    print(f"phase={phase} version={active_params.version} "
                            f"score={result.score:.4f} "
                            f"plateau={result.plateau_mean_mps:.3f} "
                            f"total={result.total_time_s:.3f} "
                            f"mean={result.mean_actual_mps:.3f} "
                            f"stop={result.stopping_distance_m:.3f} "
                            f"guarded={result.guarded} accepted={int(accepted)}",
                            flush=True)
                    validation_index += 1
                    if validation_index >= len(validation_queue):
                        best_trial = max(validation_trials,
                                key=lambda item: item.result.score)
                        if result_accepted(best_trial.result, args.speed):
                            send_with_ack(port, reader, CMD_COMMIT,
                                    make_frame(CMD_COMMIT, struct.pack("<I",
                                            best_trial.parameters.version),
                                            sequence), sequence)
                            sequence += 1
                        print(f"best version={best_trial.parameters.version} "
                                f"score={best_trial.result.score:.4f} "
                                f"plateau={best_trial.result.plateau_mean_mps:.3f} "
                                f"total={best_trial.result.total_time_s:.3f} "
                                f"mean={best_trial.result.mean_actual_mps:.3f} "
                                f"accepted={int(result_accepted(best_trial.result, args.speed))} "
                                f"params={best_trial.parameters.values()}", flush=True)
                        phase = "complete"
                        break
                    active_params = validation_queue[validation_index].parameters
                guard_seen = False
                episode_active = False
                csv_file.flush()
                telemetry_file.flush()
                sequence = start_trial(port, reader, active_params, args.speed,
                        sequence)

        telemetry_file.flush()
        subprocess.run([sys.executable, str(Path(__file__).with_name(
                "plot_ai_tune.py")), str(telemetry_csv)], check=False)


if __name__ == "__main__":
    main()
