#!/usr/bin/env python3
"""Animate schema-v3 MPC state/input horizons against CrazySim truth.

All vehicle states are expressed in world NWU. Rodrigues attitude states are
converted to world ZYX roll/pitch/yaw. Body rates are rotated into world axes.
One video frame is rendered for every completed solve.
"""
import argparse
import json
import struct
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import numpy as np

RECORD_SIZE = 3848
STATE_KNOTS = 20
INPUT_KNOTS = 19
STATE_DT_DEFAULT = 0.01
HOVER_THRUST_N = 0.1054575


def rotation(quaternion):
    quaternion = quaternion / np.linalg.norm(
        quaternion, axis=-1, keepdims=True)
    x, y, z, w = np.moveaxis(quaternion, -1, 0)
    values = np.stack([
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
        2 * (x * z + y * w), 2 * (x * y + z * w),
        1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w),
        1 - 2 * (x * x + y * y)], axis=-1)
    return values.reshape(quaternion.shape[:-1] + (3, 3))


def angles(matrix):
    return np.stack([
        np.arctan2(matrix[..., 2, 1], matrix[..., 2, 2]),
        np.arcsin(np.clip(-matrix[..., 2, 0], -1, 1)),
        np.arctan2(matrix[..., 1, 0], matrix[..., 0, 0])], axis=-1)


def state_to_world(states, frame):
    cosine, sine = np.cos(frame[3]), np.sin(frame[3])
    yaw_rotation = np.array([
        [cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])
    local_quaternion = np.concatenate(
        [states[:, 3:6], np.ones((len(states), 1))], axis=1)
    body_to_world = yaw_rotation @ rotation(local_quaternion)
    return np.concatenate([
        states[:, :3] @ yaw_rotation.T + frame[:3],
        angles(body_to_world),
        states[:, 6:9] @ yaw_rotation.T,
        np.einsum("nij,nj->ni", body_to_world, states[:, 9:12])], axis=1)


def counter_time(counter, telemetry, field):
    values = telemetry[field]
    valid = values > 0
    unique, indices = np.unique(values[valid], return_index=True)
    times = telemetry["time_s"][valid][indices]
    if np.any(counter < unique[0]) or np.any(counter > unique[-1] + 20):
        raise ValueError("Diagnostic counter outside supported simulation range")
    result = np.interp(counter, unique, times)
    tail = counter > unique[-1]
    if np.any(tail):
        count = min(100, len(times))
        slope = ((times[-1] - times[-count])
                 / (unique[-1] - unique[-count]))
        result[tail] = times[-1] + (counter[tail] - unique[-1]) * slope
    return result


def rpm_thrust(rpm, coefficients):
    constant, linear, quadratic = coefficients
    return np.maximum(
        constant + linear * rpm + quadratic * rpm * rpm, 0.0)


def load(run):
    config = json.loads((run / "run_config.json").read_text())
    diagnostic = config["mpc_diagnostic"]
    if not diagnostic["container"]["validation"]["safe_to_extract"]:
        raise ValueError("Diagnostic container failed validation")
    layout = diagnostic.get("record_layout", {})
    state_dt = float(layout.get("state_dt_s", STATE_DT_DEFAULT))

    raw = (run / "mpc_diag.bin").read_bytes()
    if len(raw) % RECORD_SIZE:
        raise ValueError("Truncated diagnostic record")
    records = []
    for offset in range(0, len(raw), RECORD_SIZE):
        record = raw[offset:offset + RECORD_SIZE]
        magic, version, size, event = struct.unpack_from("<IHHI", record)
        if (magic, version, size) != (0x544D5043, 3, RECORD_SIZE):
            raise ValueError("Requires diagnostic schema v3")
        if event == 1:
            records.append(record)
    if not records:
        raise ValueError("No solve records")

    def integers(offset):
        return np.array([
            struct.unpack_from("<I", record, offset)[0]
            for record in records])

    def floats(offset, count):
        return np.array([
            np.frombuffer(record, dtype="<f4", count=count, offset=offset)
            for record in records])

    solve_ids = integers(20)
    if not np.all(np.diff(solve_ids) == 1):
        raise ValueError("Missing or reordered solve records")
    telemetry = np.genfromtxt(
        run / "state.csv", delimiter=",", names=True)
    frames = floats(296, 4)
    local_states = [
        floats(offset, 240).reshape(-1, STATE_KNOTS, 12)
        for offset in (360, 1320, 2280)]
    input_horizons = [
        floats(offset, 76).reshape(-1, INPUT_KNOTS, 4)
        for offset in (3240, 3544)]
    if not all(np.isfinite(array).all()
               for array in [frames, *local_states, *input_horizons]):
        raise ValueError("Nonfinite horizon value")

    state_horizons = [
        np.stack([state_to_world(state, frame)
                  for state, frame in zip(horizon, frames)])
        for horizon in local_states]
    quaternion = np.column_stack([
        telemetry[name] for name in ("qx", "qy", "qz", "qw")])
    body_to_world = rotation(quaternion)
    world_rates = np.einsum(
        "nij,nj->ni", body_to_world,
        np.column_stack([
            telemetry[name]
            for name in ("wx_radps", "wy_radps", "wz_radps")]))
    truth_states = np.column_stack([
        telemetry[name] for name in ("x_m", "y_m", "z_m")] + [
        angles(body_to_world)[:, index] for index in range(3)] + [
        telemetry[name] for name in ("vx_mps", "vy_mps", "vz_mps")] + [
        world_rates[:, index] for index in range(3)])

    plant = config["acceptance_contract"]["resolved_plant_profile"]
    coefficients = plant["rpm_to_thrust"]
    command_thrust = rpm_thrust(
        np.column_stack([
            telemetry[f"rpm_ref_{motor}"] for motor in range(1, 5)]),
        coefficients)
    realized_thrust = rpm_thrust(
        np.column_stack([
            telemetry[f"rpm_{motor}"] for motor in range(1, 5)]),
        coefficients)
    # Direct rigid-body solver inputs are motor-thrust deviations from hover.
    predicted_inputs = [
        horizon + HOVER_THRUST_N for horizon in input_horizons]

    snapshot_time = counter_time(
        integers(36), telemetry, "stabilizer_sequence")
    available_time = counter_time(
        integers(32), telemetry, "firmware_tick")
    available_time = np.maximum.accumulate(
        np.maximum(snapshot_time, available_time))

    truth_states[:, 3:6] = np.unwrap(truth_states[:, 3:6], axis=0)
    for horizon in state_horizons:
        horizon[:, :, 3:6] = np.unwrap(horizon[:, :, 3:6], axis=1)
        for index in range(3, 6):
            anchor = np.interp(
                snapshot_time, telemetry["time_s"], truth_states[:, index])
            horizon[:, :, index] += (
                2 * np.pi * np.round(
                    (anchor - horizon[:, 0, index]) / (2 * np.pi)))[:, None]

    return (telemetry["time_s"], truth_states, state_horizons,
            command_thrust, realized_thrust, predicted_inputs,
            snapshot_time, available_time, solve_ids, state_dt)


def update_limits(axis, values):
    finite = np.concatenate([
        np.asarray(value).reshape(-1)
        for value in values])
    finite = finite[np.isfinite(finite)]
    low, high = finite.min(), finite.max()
    minimum_pad = max(abs(low), abs(high), 1.0) * 1e-4
    pad = max((high - low) * 0.12, minimum_pad)
    axis.set_ylim(low - pad, high + pad)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--dt", type=float, default=None,
                        help="Override archived model knot interval")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--history-seconds", type=float, default=1.0)
    parser.add_argument("--future-seconds", type=float, default=3.0)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    if args.fps <= 0 or args.history_seconds < 0 or args.future_seconds <= 0:
        parser.error("fps/future must be positive and history nonnegative")

    (time, truth, state_horizons, command_thrust, realized_thrust,
     input_horizons, snapshot, available, solve_ids, archived_dt) = load(
         args.run)
    dt = args.dt if args.dt is not None else archived_dt
    if dt <= 0:
        parser.error("dt must be positive")
    output = args.out or args.run / "horizon_world_inputs_window.mp4"
    if output.exists():
        raise FileExistsError(output)

    # Exact conversion checks for position, attitude and body-to-world rates.
    check_state = np.zeros((1, 12))
    check_state[0, 0] = 1
    check_state[0, 9] = 1
    check = state_to_world(
        check_state, np.array([2, 3, 4, np.pi / 2]))
    np.testing.assert_allclose(check[0, :3], [2, 4, 4], atol=1e-12)
    np.testing.assert_allclose(check[0, 9:], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(check[0, 5], np.pi / 2, atol=1e-12)

    truth = truth.copy()
    truth[:, 3:6] *= 180 / np.pi
    for horizon in state_horizons:
        horizon[:, :, 3:6] *= 180 / np.pi

    plt.style.use("seaborn-v0_8-whitegrid")
    figure, axes = plt.subplots(
        4, 4, figsize=(19.2, 10.8), sharex=True)
    figure.subplots_adjust(
        top=0.84, bottom=0.08, left=0.055, right=0.985,
        hspace=0.34, wspace=0.24)
    titles = [
        "World x [m]", "World y [m]", "World z [m]", "Motor 1 thrust [N]",
        "World roll [deg]", "World pitch [deg]", "World yaw [deg]",
        "Motor 2 thrust [N]",
        "World vx [m/s]", "World vy [m/s]", "World vz [m/s]",
        "Motor 3 thrust [N]",
        "World omega x [rad/s]", "World omega y [rad/s]",
        "World omega z [rad/s]", "Motor 4 thrust [N]"]
    state_indices = [0, 1, 2, None, 3, 4, 5, None,
                     6, 7, 8, None, 9, 10, 11, None]
    motor_indices = [None, None, None, 0, None, None, None, 1,
                     None, None, None, 2, None, None, None, 3]

    state_lines = {}
    input_lines = {}
    cursors = []
    dots = []
    state_colors = ["#169873", "#ea7d23", "#7c4dcc"]
    for panel, axis in enumerate(axes.flat):
        state_index = state_indices[panel]
        motor = motor_indices[panel]
        if state_index is not None:
            axis.plot(time, truth[:, state_index], color="#344b62", lw=1.3,
                      label="Closed-loop simulation")
            state_lines[panel] = [
                axis.plot([], [], color=color, lw=1.8, ls=style,
                          label=label)[0]
                for color, style, label in zip(
                    state_colors, ["--", "-", "-."],
                    ["Reference horizon", "Primal prediction",
                     "Projected prediction"])]
            dots.append(axis.plot(
                [], [], "o", color="#d32f45", ms=4)[0])
        else:
            axis.plot(time, realized_thrust[:, motor], color="#344b62",
                      lw=1.3, label="Realized plant thrust")
            axis.plot(time, command_thrust[:, motor], color="#78909c",
                      lw=1.0, alpha=0.9, label="Commanded plant thrust")
            input_lines[panel] = [
                axis.plot([], [], color="#ea7d23", lw=1.8, label=
                          "Primal input prediction")[0],
                axis.plot([], [], color="#7c4dcc", lw=1.8, ls="-.",
                          label="Projected input prediction")[0]]
            dots.append(axis.plot(
                [], [], "o", color="#d32f45", ms=4)[0])
        cursors.append(axis.axvline(
            time[0], color="#d32f45", lw=1, alpha=0.8))
        axis.set_title(titles[panel], loc="left", fontsize=10.5)
        axis.ticklabel_format(axis="y", style="plain", useOffset=False)
        if panel >= 12:
            axis.set_xlabel("Simulation time [s]")

    handles, labels = [], []
    for axis in (axes[0, 0], axes[0, 3]):
        axis_handles, axis_labels = axis.get_legend_handles_labels()
        for handle, label in zip(axis_handles, axis_labels):
            if label not in labels:
                handles.append(handle)
                labels.append(label)
    figure.legend(handles, labels, loc="upper center",
                  bbox_to_anchor=(0.5, 0.91), ncol=4, frameon=False,
                  fontsize=9)
    title = figure.suptitle("", fontsize=15, y=0.98)
    figure.text(
        0.5, 0.018,
        "World NWU | ZYX Euler attitude | Angular velocity in world axes | "
        "Visible window: current time - 1 s to current time + 3 s",
        ha="center", fontsize=9.5)

    writer = FFMpegWriter(
        fps=args.fps, codec="libx264", bitrate=5000,
        extra_args=["-pix_fmt", "yuv420p"])
    with writer.saving(figure, str(output), 100):
        for solve_index, solve_id in enumerate(solve_ids):
            current = available[solve_index]
            state_time = snapshot[solve_index] + dt * np.arange(STATE_KNOTS)
            input_time = snapshot[solve_index] + dt * np.arange(INPUT_KNOTS)
            left = current - args.history_seconds
            right = current + args.future_seconds
            window = (time >= left) & (time <= right)
            for panel, axis in enumerate(axes.flat):
                state_index = state_indices[panel]
                motor = motor_indices[panel]
                axis.set_xlim(left, right)
                cursors[panel].set_xdata([current, current])
                if state_index is not None:
                    for line, horizon in zip(
                            state_lines[panel], state_horizons):
                        line.set_data(
                            state_time, horizon[solve_index, :, state_index])
                    current_value = np.interp(
                        current, time, truth[:, state_index])
                    dots[panel].set_data([current], [current_value])
                    update_limits(axis, [
                        truth[window, state_index],
                        *[horizon[solve_index, :, state_index]
                          for horizon in state_horizons]])
                else:
                    for line, horizon in zip(
                            input_lines[panel], input_horizons):
                        line.set_data(
                            input_time, horizon[solve_index, :, motor])
                    current_value = np.interp(
                        current, time, realized_thrust[:, motor])
                    dots[panel].set_data([current], [current_value])
                    update_limits(axis, [
                        realized_thrust[window, motor],
                        command_thrust[window, motor],
                        *[horizon[solve_index, :, motor]
                          for horizon in input_horizons]])
            title.set_text(
                f"MPC prediction vs closed-loop simulation | "
                f"Current time: {current:.3f} s\n"
                f"Solve {solve_id} / {solve_ids[-1]} | "
                f"Snapshot: {snapshot[solve_index]:.3f} s | "
                f"Prediction horizon: {(STATE_KNOTS - 1) * dt:.2f} s")
            writer.grab_frame()
            if solve_index == len(solve_ids) // 2:
                figure.savefig(output.with_suffix(".png"), dpi=100)
            if solve_index % 100 == 0:
                print(
                    f"Rendered {solve_index + 1}/{len(solve_ids)}",
                    flush=True)

    report = {
        "run": str(args.run), "video": str(output),
        "frames": len(solve_ids), "fps": args.fps, "dt_s": dt,
        "window_seconds": {
            "before_current": args.history_seconds,
            "after_current": args.future_seconds},
        "inputs": {
            "prediction": "solver motor thrust correction plus 0.1054575 N hover",
            "closed_loop_command": "CrazySim rpm_ref converted with archived rpm_to_thrust",
            "closed_loop_realized": "CrazySim rpm converted with archived rpm_to_thrust"},
        "first_solve": int(solve_ids[0]),
        "last_solve": int(solve_ids[-1]),
        "conversion_checks": "passed",
        "time_mapping":
            "plan_tick->stabilizer_sequence; finish_tick->firmware_tick"}
    output.with_suffix(".json").write_text(
        json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
