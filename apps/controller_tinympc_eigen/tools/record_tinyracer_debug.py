#!/usr/bin/env python3
"""Record and plot TinyRacer flight, perception, and constraint telemetry."""

import argparse
import csv
import math
import os
import threading
import time
from datetime import datetime
from pathlib import Path


PERIOD_MS = 50
LOG_GROUPS = {
    "state": [
        "stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
        "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz",
    ],
    "reference": [
        "trRef.x", "trRef.y", "trRef.z",
        "trRef.hx", "trRef.hy", "trRef.hz",
    ],
    "cylinder": [
        "trCyl.x", "trCyl.y", "trCyl.r",
        "trCyl.nx", "trCyl.ny", "trCyl.b",
    ],
    "controller": [
        "trCtl.lateral", "trCtl.violation", "trCtl.consensus", "trCtl.slack",
        "trCtl.solveUs", "trCtl.mode", "trCtl.cyl", "trCtl.side",
    ],
    "clearance": ["seqRx.d0", "seqRx.d1", "seqRx.d2", "seqRx.d3"],
    "confidence": ["seqRx.c0", "seqRx.c1", "seqRx.c2", "seqRx.c3"],
}
FIELDS = ["time_s"] + [f"{name}_timestamp_ms" for name in LOG_GROUPS]
FIELDS += [variable for variables in LOG_GROUPS.values() for variable in variables]


def plot_csv(csv_path, output_path=None):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    with open(csv_path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"No samples in {csv_path}")

    def values(name):
        return [float(row[name]) if row.get(name, "") else math.nan for row in rows]

    t = values("time_s")
    x, y = values("stateEstimate.x"), values("stateEstimate.y")
    rx, ry = values("trRef.x"), values("trRef.y")
    hx, hy = values("trRef.hx"), values("trRef.hy")
    active = values("trCtl.cyl")
    cx, cy, radius = values("trCyl.x"), values("trCyl.y"), values("trCyl.r")
    nx, ny, boundary = values("trCyl.nx"), values("trCyl.ny"), values("trCyl.b")

    fig, axes = plt.subplots(3, 1, figsize=(11, 13), constrained_layout=True)
    ax = axes[0]
    ax.plot(x, y, color="black", linewidth=2, label="estimated flight path")
    ax.plot(rx, ry, color="tab:blue", linewidth=1.2, label="immediate MPC reference")
    ax.plot(hx, hy, color="tab:cyan", linewidth=1, alpha=0.75,
            label="terminal horizon reference")
    previous = False
    for i, is_active in enumerate(active):
        current = is_active > 0.5
        if current and (not previous or (i > 0 and
                (abs(cx[i] - cx[i - 1]) > 1e-3 or abs(cy[i] - cy[i - 1]) > 1e-3))):
            ax.add_patch(Circle((cx[i], cy[i]), radius[i], color="tab:red",
                                alpha=0.18, label="inferred cylinder" if not previous else None))
            ax.scatter([cx[i]], [cy[i]], color="tab:red", s=18)
        if current and i % 10 == 0 and all(math.isfinite(v) for v in
                                           (nx[i], ny[i], boundary[i], radius[i])):
            norm_sq = nx[i] * nx[i] + ny[i] * ny[i]
            if norm_sq > 1e-6:
                px, py = boundary[i] * nx[i] / norm_sq, boundary[i] * ny[i] / norm_sq
                tx, ty = -ny[i], nx[i]
                scale = max(0.25, radius[i] * 1.4) / math.sqrt(norm_sq)
                ax.plot([px - scale * tx, px + scale * tx],
                        [py - scale * ty, py + scale * ty],
                        color="tab:red", alpha=0.12, linewidth=0.8)
        previous = current
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_title("TinyRacer trajectory and inferred obstacle constraints")
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc="best")
    ax.grid(True, alpha=0.25)

    for sector in range(4):
        axes[1].plot(t, values(f"seqRx.d{sector}"), label=f"sector {sector}")
    axes[1].axhline(0.30, color="black", linestyle="--", linewidth=1,
                    label="activation threshold")
    axes[1].set_ylabel("clearance [m]")
    axes[1].set_title("Vision clearances")
    axes[1].legend(ncol=3)
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(t, values("trCtl.violation"), label="plane violation")
    axes[2].plot(t, values("trCtl.consensus"), label="consensus error")
    axes[2].plot(t, values("trCtl.slack"), label="half-space slack")
    axes[2].step(t, values("trCtl.mode"), where="post", alpha=0.65,
                 label="mode (0 track, 1 blocked, 2 recover)")
    axes[2].set_xlabel("host time [s]")
    axes[2].set_ylabel("solver diagnostic / mode")
    axes[2].set_title("Constraint and solver behavior")
    axes[2].legend(ncol=2)
    axes[2].grid(True, alpha=0.25)

    output_path = Path(output_path) if output_path else Path(csv_path).with_suffix(".png")
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    print(f"Plot: {output_path}")
    return output_path


def record(args):
    try:
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.log import LogConfig
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        from cflib.utils import uri_helper
    except ImportError as error:
        raise SystemExit("Install dependencies with: pip install cflib matplotlib") from error

    uri = args.uri or uri_helper.uri_from_env(
        default="radio://0/80/2M/E7E7E7E7E8")
    run_dir = Path(args.output_dir) / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=False)
    csv_path, console_path = run_dir / "telemetry.csv", run_dir / "console.txt"
    latest, lock, stop = {}, threading.Lock(), threading.Event()
    start = time.monotonic()

    def pid_takeoff(cf):
        cf.param.set_value("stabilizer.controller", "1")
        time.sleep(0.1)
        started, stable_since = time.monotonic(), None
        while True:
            now = time.monotonic()
            elapsed = now - started
            target = args.takeoff_height * min(elapsed / args.takeoff_ramp, 1.0)
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, target)
            with lock:
                z = latest.get("stateEstimate.z")
                vx = latest.get("stateEstimate.vx")
                vy = latest.get("stateEstimate.vy")
                vz = latest.get("stateEstimate.vz")
            stable = (z is not None and vx is not None and vy is not None and
                      vz is not None and abs(z - args.takeoff_height) < 0.04 and
                      math.sqrt(vx * vx + vy * vy + vz * vz) < 0.10)
            stable_since = stable_since or now if stable else None
            if stable_since is not None and now - stable_since >= args.hover_settle:
                print(f"PID hover established at z={z:.2f} m.")
                return
            if elapsed >= args.takeoff_timeout:
                raise RuntimeError(f"PID takeoff did not settle at {args.takeoff_height:.2f} m")
            time.sleep(0.05)

    def log_callback(group):
        def callback(timestamp, data, _config):
            with lock:
                latest.update(data)
                latest[f"{group}_timestamp_ms"] = timestamp
        return callback

    with open(csv_path, "w", newline="", buffering=1) as csv_stream, \
            open(console_path, "w", buffering=1) as console_stream:
        writer = csv.DictWriter(csv_stream, fieldnames=FIELDS)
        writer.writeheader()

        def console_callback(text):
            elapsed = time.monotonic() - start
            line = f"{elapsed:10.3f} {text}"
            print(text, end="")
            console_stream.write(line)

        def sampler():
            next_sample = time.monotonic()
            while not stop.is_set():
                next_sample += PERIOD_MS / 1000.0
                with lock:
                    row = {field: latest.get(field, "") for field in FIELDS}
                row["time_s"] = time.monotonic() - start
                writer.writerow(row)
                stop.wait(max(0.0, next_sample - time.monotonic()))

        def commander_keepalive(cf):
            while not stop.is_set():
                cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, args.takeoff_height)
                stop.wait(0.1)

        cflib.crtp.init_drivers(enable_debug_driver=False)
        cache = str(Path.home() / ".cache" / "crazyflie")
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache)) as scf:
            scf.cf.console.receivedChar.add_callback(console_callback)
            configs = []
            armed = False
            keepalive_thread = None
            try:
                for group, variables in LOG_GROUPS.items():
                    config = LogConfig(name=f"TinyRacer {group}", period_in_ms=PERIOD_MS)
                    for variable in variables:
                        config.add_variable(variable)
                    scf.cf.log.add_config(config)
                    if not config.valid:
                        raise RuntimeError(
                            f"Firmware is missing telemetry for {group}; flash the rebuilt firmware")
                    config.data_received_cb.add_callback(log_callback(group))
                    config.start()
                    configs.append(config)

                thread = threading.Thread(target=sampler, daemon=True)
                thread.start()
                if not args.no_arm:
                    if not args.no_set_controller:
                        scf.cf.param.set_value("stabilizer.controller", "1")
                    scf.cf.platform.send_arming_request(True)
                    armed = True
                    print("Armed.")
                else:
                    print("Recording without arming.")
                if not args.no_set_controller:
                    if args.no_arm:
                        raise RuntimeError("PID takeoff requires arming")
                    pid_takeoff(scf.cf)
                    scf.cf.param.set_value("stabilizer.controller", "6")
                    keepalive_thread = threading.Thread(
                        target=commander_keepalive, args=(scf.cf,), daemon=True)
                    keepalive_thread.start()
                    print("Handed off to OutOfTree (6) at trajectory knot zero; Ctrl-C to stop.")
                else:
                    print("Recording without changing controller; Ctrl-C to stop.")
                deadline = time.monotonic() + args.duration if args.duration > 0 else math.inf
                try:
                    while time.monotonic() < deadline:
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    pass
            finally:
                stop.set()
                if keepalive_thread is not None:
                    keepalive_thread.join(timeout=1.0)
                if armed:
                    scf.cf.platform.send_arming_request(False)
                if "thread" in locals():
                    thread.join(timeout=1.0)
                for config in configs:
                    config.stop()

    print(f"CSV: {csv_path}")
    print(f"Console: {console_path}")
    if not args.no_plot:
        plot_csv(csv_path)
    return csv_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", help="Crazyflie URI (or use CFLIB_URI)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="recording duration in seconds; default records until Ctrl-C")
    parser.add_argument("--takeoff-height", type=float, default=0.4)
    parser.add_argument("--takeoff-ramp", type=float, default=2.0,
                        help="PID climb duration in seconds")
    parser.add_argument("--takeoff-timeout", type=float, default=8.0)
    parser.add_argument("--hover-settle", type=float, default=0.4,
                        help="required stable hover time before MPC handoff")
    parser.add_argument("--output-dir", default=str(Path(__file__).parent / "debug_runs"))
    parser.add_argument("--no-set-controller", action="store_true",
                        help="do not set stabilizer.controller=6")
    parser.add_argument("--no-arm", action="store_true",
                        help="do not send the supervisor arming request")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--plot-only", metavar="CSV", help="plot an existing recording")
    args = parser.parse_args()
    if args.plot_only:
        plot_csv(args.plot_only)
    else:
        record(args)


if __name__ == "__main__":
    main()
