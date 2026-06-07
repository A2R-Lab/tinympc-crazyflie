#!/usr/bin/env python3
"""
Compute the Crazyflie moment-of-inertia tensor for a stack of decks, via the
composite-body method (new CoM + parallel-axis theorem). Outputs J for the OLD
config (CF + Flow deck) and the NEW config (CF + Flow deck + AI deck), plus the
constants to paste into regen_params.py.

Each part is modeled as a thin rectangular PCB plate (dims a in x, b in y) whose
center sits at (x, y, z) relative to the CF main-board center (z up).

Plate inertia about its own center (thickness neglected):
    Ixx = (1/12) m b^2      Iyy = (1/12) m a^2      Izz = (1/12) m (a^2 + b^2)

Parallel axis to the composite CoM at height zc (parts are centered in x,y):
    Ixx += Ixx_own + m (z - zc)^2
    Iyy += Iyy_own + m (z - zc)^2
    Izz += Izz_own                     # no z lever arm for yaw

>>> MEASURE THESE <<<  the deck mount HEIGHT (z) is the dominant uncertainty for
Ixx/Iyy because it enters as z^2. Put a caliper on the gap from the main board to
each deck PCB. Masses are from Bitcraze datasheets; override with a scale reading.
"""
import numpy as np

g = 1e-3  # grams -> kg
mm = 1e-3  # millimeters -> meters

# ---------------------------------------------------------------------------
# Part table. Each: name, mass[kg], a[m](x), b[m](y), z[m](height above board).
# The CF "base" lumps board+motors+battery; its inertia is the bare-CF system-ID
# value (Forster 2015), treated as a solid block about its own CoM (z~0), NOT a
# plate — so we pass its J directly instead of the plate formula.
# ---------------------------------------------------------------------------
# Total measured flying mass = 36.3 g. AI deck = 4.4 g, Flow deck = 1.6 g, so the
# CF base (board+motors+battery+wiring) = 36.3 - 4.4 - 1.6 = 30.3 g.
CF_BASE = dict(
    name="CF2.1 base (board+motors+battery)",
    m=30.3 * g,
    z=0.0 * mm,
    J=np.array([1.66e-5, 1.66e-5, 2.93e-5]),  # bare-CF system-ID, about its CoM
)

# Flow deck v2 — mounts on the BOTTOM (z negative). ~1.6 g, thin.
FLOW_DECK = dict(
    name="Flow deck v2 (bottom)",
    m=1.6 * g,
    a=28.0 * mm, b=30.0 * mm, t=2.0 * mm,
    z=-8.0 * mm,            # <-- MEASURE: deck-center height below board
)

# AI deck 1.1 — mounts on TOP. Measured 30 x 52 x 8 mm, 4.4 g.
# AXIS ASSUMPTION: long 52 mm side runs fore-aft (body x); 30 mm lateral (body y).
# If the deck's long side is actually lateral, swap a<->b (and IXX_NEW<->IYY_NEW).
AI_DECK = dict(
    name="AI deck 1.1 (top, 30x52x8)",
    m=4.4 * g,
    a=52.0 * mm,           # x (fore-aft) extent
    b=30.0 * mm,           # y (lateral) extent
    t=8.0 * mm,            # z (thickness) extent
    z=+9.0 * mm,           # <-- MEASURE: deck-center height = mount gap + t/2
)


def plate_inertia(part):
    """Own-center inertia of a solid rectangular box (a x b x t) -> diag Ixx,Iyy,Izz.
    Ixx uses the extents perpendicular to x (b,t); Iyy uses (a,t); Izz uses (a,b)."""
    if "J" in part:
        return part["J"].copy()
    m, a, b = part["m"], part["a"], part["b"]
    t = part.get("t", 0.0)   # thickness (z extent); 0 = thin plate
    return np.array([
        (1.0 / 12.0) * m * (b * b + t * t),
        (1.0 / 12.0) * m * (a * a + t * t),
        (1.0 / 12.0) * m * (a * a + b * b),
    ])


def composite_J(parts):
    """Composite mass, CoM height, and inertia about the composite CoM."""
    M = sum(p["m"] for p in parts)
    zc = sum(p["m"] * p["z"] for p in parts) / M
    J = np.zeros(3)
    for p in parts:
        Jown = plate_inertia(p)
        d2 = (p["z"] - zc) ** 2           # x,y offsets are 0 -> only z matters
        J[0] += Jown[0] + p["m"] * d2     # Ixx
        J[1] += Jown[1] + p["m"] * d2     # Iyy
        J[2] += Jown[2]                   # Izz: no z lever arm
    return M, zc, J


def report(label, parts):
    M, zc, J = composite_J(parts)
    print(f"\n=== {label} ===")
    for p in parts:
        print(f"   {p['name']:38s} m={p['m']*1e3:5.2f} g  z={p['z']*1e3:+5.1f} mm")
    print(f"   total mass = {M*1e3:.2f} g   CoM height zc = {zc*1e3:+.2f} mm")
    print(f"   Ixx={J[0]:.4e}  Iyy={J[1]:.4e}  Izz={J[2]:.4e}  kg*m^2")
    return M, J


def main():
    M_old, J_old = report("OLD: CF + Flow deck", [CF_BASE, FLOW_DECK])
    M_new, J_new = report("NEW: CF + Flow deck + AI deck", [CF_BASE, FLOW_DECK, AI_DECK])

    d = (J_new - J_old) / J_old * 100.0
    print("\n=== change from adding the AI deck ===")
    print(f"   dmass = {(M_new-M_old)*1e3:+.2f} g")
    print(f"   Ixx {d[0]:+.1f}%   Iyy {d[1]:+.1f}%   Izz {d[2]:+.1f}%")

    print("\n=== paste into regen_params.py ===")
    print(f"M_OLD   = {M_old:.4f}")
    print(f"IXX_OLD = {J_old[0]:.4e}")
    print(f"IYY_OLD = {J_old[1]:.4e}")
    print(f"IZZ_OLD = {J_old[2]:.4e}")
    print(f"M_NEW   = {M_new:.4f}")
    print(f"IXX_NEW = {J_new[0]:.4e}")
    print(f"IYY_NEW = {J_new[1]:.4e}")
    print(f"IZZ_NEW = {J_new[2]:.4e}")


if __name__ == "__main__":
    main()
