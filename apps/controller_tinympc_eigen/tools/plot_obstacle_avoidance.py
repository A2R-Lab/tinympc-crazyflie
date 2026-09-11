#!/usr/bin/env python3
"""Render logged world-frame avoidance constraints in a top-down video.

Usage: python tools/plot_obstacle_avoidance.py RUN_DIR [--output flight.mp4]
       python tools/plot_obstacle_avoidance.py --demo /tmp/avoid-demo

Requires matplotlib and FFmpeg for MP4 (imageio-ffmpeg can supply FFmpeg),
or matplotlib and Pillow for --output flight.gif. Demo data are SYNTHETIC.
Each telemetry.csv row is host_s,firmware_ms,group,values_json. Only planes
whose sample exactly matches the latest as-of dgAvoid.sample are displayed.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def read_run(directory):
    directory = Path(directory)
    metadata = json.loads((directory / 'metadata.json').read_text())
    summary = directory / 'summary.json'
    if summary.exists():
        metadata.update(json.loads(summary.read_text()))
    rows = []
    with (directory / 'telemetry.csv').open(newline='') as handle:
        for line, row in enumerate(csv.DictReader(handle), 2):
            try:
                stamp = float(row['host_s'])
                values = json.loads(row['values_json'])
                if not math.isfinite(stamp) or not isinstance(values, dict):
                    raise ValueError('invalid timestamp or values')
                rows.append((stamp, row['group'], values))
            except (KeyError, ValueError, TypeError) as error:
                raise ValueError(f'telemetry.csv:{line}: {error}') from error
    rows.sort(key=lambda row: row[0])
    if not rows:
        raise ValueError('telemetry.csv contains no samples')
    return metadata, rows


def number(values, key, default=None):
    value = values.get(key, default)
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else default


def snapshot(rows, stamp):
    """As-of telemetry; missing blocks never borrow a different generation."""
    latest = {}
    planes = {}
    for time, group, values in rows:
        if time > stamp:
            break
        latest[group] = (time, values)
        if group in ('plane0', 'plane1'):
            prefix = 'dgPlane' + group[-1] + '.'
            sample = number(values, prefix + 'sample')
            planes[(group, sample)] = values
    avoid = latest.get('avoid', (None, {}))[1]
    sample = number(avoid, 'dgAvoid.sample')
    active, missing = [], []
    count = int(number(avoid, 'dgAvoid.count', 0))
    # New firmware reports actual controller/motor activation separately from
    # the persistent enabled configuration. Older logs only have enabled.
    active_flag = number(avoid, 'dgAvoid.active')
    if active_flag is None:
        active_flag = number(avoid, 'dgAvoid.enabled', 0)
    if active_flag and count > 0:
        for index in range(min(count, 2)):
            values = planes.get(('plane' + str(index), sample), {}) if sample is not None else {}
            prefix = f'dgPlane{index}.'
            plane = tuple(number(values, prefix + field) for field in ('nx', 'ny', 'b'))
            if any(value is None for value in plane) or math.hypot(*(plane[:2])) < 1e-8:
                missing.append(index)
            else:
                active.append((index, plane))
    return latest, active, missing


def clip_polygon(polygon, nx, ny, b):
    """Clip a convex polygon to nx*x+ny*y >= b (the excluded side)."""
    result = []
    if not polygon:
        return result
    previous = polygon[-1]
    prev_d = nx * previous[0] + ny * previous[1] - b
    for point in polygon:
        distance = nx * point[0] + ny * point[1] - b
        if (distance >= 0) != (prev_d >= 0):
            fraction = prev_d / (prev_d - distance)
            result.append((previous[0] + fraction * (point[0] - previous[0]),
                           previous[1] + fraction * (point[1] - previous[1])))
        if distance >= 0:
            result.append(point)
        previous, prev_d = point, distance
    return result


def boundary_segment(nx, ny, b, bounds):
    xmin, xmax, ymin, ymax = bounds
    points = []
    if abs(ny) > 1e-12:
        points.extend((x, (b - nx*x)/ny) for x in (xmin, xmax))
    if abs(nx) > 1e-12:
        points.extend(((b - ny*y)/nx, y) for y in (ymin, ymax))
    inside = []
    for x, y in points:
        if xmin-1e-9 <= x <= xmax+1e-9 and ymin-1e-9 <= y <= ymax+1e-9:
            if not any(math.hypot(x-a, y-c) < 1e-9 for a, c in inside):
                inside.append((x, y))
    return inside[:2]


def create_demo(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if any((directory / name).exists() for name in ('metadata.json', 'telemetry.csv', 'events.csv')):
        raise ValueError('demo directory already contains run files; choose an empty directory')
    (directory / 'metadata.json').write_text(json.dumps({'synthetic': True, 'speed': 0.1,
        'distance': 0.7, 'mission_origin': [0, 0], 'heading_rad': 0.0}, indent=2))
    with (directory / 'telemetry.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['host_s', 'firmware_ms', 'group', 'values_json'])
        for index in range(81):
            t = index / 10
            x = min(0.1*t, 0.57)
            count = 0 if t < 2 else (1 if t < 4 else 2)
            groups = {
                'position': {'stateEstimate.x': x, 'stateEstimate.y': 0.0, 'stateEstimate.z': 0.5,
                             'stateEstimate.vx': 0.1 if t < 5.7 else 0.0, 'stateEstimate.vy': 0.0},
                'attitude': {'stateEstimate.yaw': 0.0},
                'avoid': {'dgAvoid.sample': index, 'dgAvoid.count': count, 'dgAvoid.enabled': 1, 'dgAvoid.active': 1,
                          'dgAvoid.fresh': int(t < 6), 'dgAvoid.fault': int(t >= 7),
                          'dgAvoid.cmdSpeed': 0.1, 'dgAvoid.mode': 1},
                'plane0': {'dgPlane0.sample': index, 'dgPlane0.nx': 1.0, 'dgPlane0.ny': 0.0, 'dgPlane0.b': 0.59},
                'plane1': {'dgPlane1.sample': index, 'dgPlane1.nx': 0.6, 'dgPlane1.ny': 0.8, 'dgPlane1.b': 0.39},
            }
            for group, values in groups.items():
                writer.writerow([t, index*100, group, json.dumps(values)])
    (directory / 'events.csv').write_text('host_s,event\n0,SYNTHETIC demo starts\n')
    return directory


def render(directory, output, fps=15, playback_speed=1.0):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
    from matplotlib.patches import Polygon
    metadata, rows = read_run(directory)
    positions = [(t, number(v, 'stateEstimate.x'), number(v, 'stateEstimate.y'))
                 for t, g, v in rows if g == 'position']
    positions = [(t, x, y) for t, x, y in positions if x is not None and y is not None]
    if not positions:
        raise ValueError('run has no valid world XY position telemetry')
    xs, ys = [p[1] for p in positions], [p[2] for p in positions]
    origin = metadata.get('mission_origin', [xs[0], ys[0]])
    options = metadata.get('options', metadata)
    distance = options.get('distance')
    heading = metadata.get('heading_rad')
    target = None
    if distance is not None and heading is not None:
        target = (origin[0] + float(distance)*math.cos(heading), origin[1] + float(distance)*math.sin(heading))
    plot_x, plot_y = xs + ([target[0]] if target else []), ys + ([target[1]] if target else [])
    margin = max(0.5, (max(plot_x)-min(plot_x))*0.2, (max(plot_y)-min(plot_y))*0.2)
    bounds = (min(plot_x)-margin, max(plot_x)+margin, min(plot_y)-margin, max(plot_y)+margin)
    xmin, xmax, ymin, ymax = bounds
    rectangle = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
    fig, ax = plt.subplots(figsize=(9, 6))
    start, end = rows[0][0], rows[-1][0]
    frames = max(1, math.ceil((end-start)*fps/playback_speed)+1)
    times = [p[0] for p in positions]
    def draw(frame):
        stamp = min(end, start + frame*playback_speed/fps)
        latest, planes, missing = snapshot(rows, stamp)
        ax.clear()
        ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel='World X (m)', ylabel='World Y (m)')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=0.2)
        prefix = 'SYNTHETIC DEMO — ' if metadata.get('synthetic') else ''
        ax.set_title(prefix + 'Obstacle avoidance · excluded regions shaded')
        if target:
            ax.plot([origin[0], target[0]], [origin[1], target[1]], '--', color='gray', label='Requested straight line')
            ax.plot(*target, marker='x', color='black')
        stop = bisect.bisect_right(times, stamp)
        ax.plot(xs[:stop], ys[:stop], color='#1665a8', label='Measured trajectory')
        for index, (nx, ny, b) in planes:
            color = ('#df7622', '#b23c77')[index]
            polygon = clip_polygon(rectangle, nx, ny, b)
            if len(polygon) >= 3:
                ax.add_patch(Polygon(polygon, color=color, alpha=0.18))
            segment = boundary_segment(nx, ny, b, bounds)
            if len(segment) == 2:
                ax.plot(*zip(*segment), color=color, linewidth=2, label=f'Applied plane {index}')
        position_time, position = latest.get('position', (None, {}))
        yaw = number(latest.get('attitude', (None, {}))[1], 'stateEstimate.yaw')
        if stop:
            x, y = xs[stop-1], ys[stop-1]
            ax.plot(x, y, 'o', color='#1665a8')
            if yaw is not None:
                angle = math.radians(yaw)
                ax.arrow(x, y, 0.15*math.cos(angle), 0.15*math.sin(angle), width=0.015, color='#1665a8')
        vx, vy = number(position, 'stateEstimate.vx'), number(position, 'stateEstimate.vy')
        speed = f'{math.hypot(vx, vy):.2f} m/s' if vx is not None and vy is not None else 'unavailable'
        avoid_time, avoid = latest.get('avoid', (None, {}))
        def status(key):
            return str(avoid.get('dgAvoid.' + key, '?'))
        text = f't = {stamp-start:.2f} s   speed = {speed}\nenabled={status("enabled")} active={status("active")} fresh={status("fresh")} fault={status("fault")} count={status("count")} sample={status("sample")}'
        if missing:
            text += '\nPlane blocks unavailable for current sample: ' + ', '.join(map(str, missing))
        if avoid_time is None:
            text += '\nAvoidance telemetry unavailable'
        elif stamp - avoid_time > 0.5:
            text += f'\nAvoidance telemetry age: {stamp-avoid_time:.2f} s (last reported state)'
        if position_time is None or stamp-position_time > 0.5:
            text += '\nPosition telemetry missing/stale'
        ax.text(0.02, 0.98, text, transform=ax.transAxes, va='top', fontsize=9,
                bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': 'none'})
        ax.legend(loc='lower left', fontsize=8)
        fig.tight_layout()
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.suffix.lower() == '.gif':
        writer = PillowWriter(fps=fps)
    elif output.suffix.lower() == '.mp4':
        if not FFMpegWriter.isAvailable():
            try:
                import imageio_ffmpeg
                matplotlib.rcParams['animation.ffmpeg_path'] = imageio_ffmpeg.get_ffmpeg_exe()
            except ImportError as error:
                raise RuntimeError('MP4 needs FFmpeg or imageio-ffmpeg; alternatively choose --output video.gif') from error
        writer = FFMpegWriter(fps=fps, codec='libx264', extra_args=['-pix_fmt', 'yuv420p'])
    else:
        raise ValueError('output must end in .mp4 or .gif')
    try:
        FuncAnimation(fig, draw, frames=frames, interval=1000/fps).save(str(output), writer=writer)
    finally:
        plt.close(fig)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_dir', nargs='?', type=Path)
    parser.add_argument('--demo', type=Path, metavar='DIRECTORY', help='create SYNTHETIC data and render it')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--playback-speed', type=float, default=1.0)
    args = parser.parse_args()
    if bool(args.run_dir) == bool(args.demo):
        parser.error('provide either RUN_DIR or --demo DIRECTORY')
    if args.fps <= 0 or not math.isfinite(args.playback_speed) or args.playback_speed <= 0:
        parser.error('fps and playback-speed must be positive')
    directory = create_demo(args.demo) if args.demo else args.run_dir
    output = args.output or directory / 'obstacle_avoidance.mp4'
    print(render(directory, output, args.fps, args.playback_speed))


if __name__ == '__main__':
    main()
