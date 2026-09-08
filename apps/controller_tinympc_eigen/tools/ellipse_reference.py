"""Configurable ellipse geometry, starting along body heading."""
import math


def semiaxes(major=3.0, minor=2.0):
    if not all(math.isfinite(v) for v in (major, minor)) or not 0 < minor <= major:
        raise ValueError('axes must be finite full lengths with 0 < minor <= major')
    return major / 2, minor / 2


def sample(theta, speed=1.0, origin=(0.0, 0.0), heading=0.0, major=3.0, minor=2.0):
    """Return world x, y, vx, vy, tangent yaw (radians), and yaw rate.

    Axes are full lengths in meters. The center is minor/2 meters to the
    left of the starting position; initial velocity follows heading.
    """
    a, b = semiaxes(major, minor)
    st, ct = math.sin(theta), math.cos(theta)
    c, s = math.cos(heading), math.sin(heading)
    u, v = a * st, b * (1 - ct)
    du, dv = a * ct, b * st
    metric = math.hypot(du, dv)
    tx, ty = c * du - s * dv, s * du + c * dv
    return (origin[0] + c * u - s * v,
            origin[1] + s * u + c * v,
            speed * tx / metric, speed * ty / metric,
            math.atan2(ty, tx), speed * a * b / metric**3)


def advance(theta, speed, dt, major=3.0, minor=2.0):
    """Integrate angular progress at constant linear speed using midpoint steps."""
    a, b = semiaxes(major, minor)
    for _ in range(4):
        ds = speed * dt / 4
        metric = math.hypot(a * math.cos(theta), b * math.sin(theta))
        midpoint = theta + ds / (2 * metric)
        theta += ds / math.hypot(a * math.cos(midpoint), b * math.sin(midpoint))
    return theta
