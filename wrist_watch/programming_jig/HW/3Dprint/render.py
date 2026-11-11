"""Presentation renders of the 3-piece watch programming jig (tray / base / twist-lock lid) on the programmer
board, with the watch PCB (dial): unlocked (lid dropped in at the slots), locked (twisted clockwise),
exploded, and a locked cutaway through both M3 screws (x = 0 plane).

Rasteriser: numpy z-buffer, 2x supersampled, flat shaded, outlines on B-rep face edges (same approach as
../watch_holder/render.py). Context models are simplified stand-ins placed from the KiCad / gerber data:
programmer outline, USB-C, CP2102N, AMS1117, 0805s, J13/J15 headers; dial PCB with TQFP-32, SOIC-16,
LED grid, side button (ESTIMATED size, as in the jig script) and a CR2032 underneath.

    python render.py  ->  renders/render_unlocked / render_locked / render_exploded / render_cutaway .png
"""
import csv
from math import cos, radians, sin

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from build123d import Align, Box, Cylinder, Pos, Rot, Sphere, fillet

import watch_programmer_jig as J

HERE = J.HERE
W_PX, H_PX, SS = 1800, 1250, 2
BG_TOP, BG_BOT = np.array([0.97, 0.97, 0.96]), np.array([0.86, 0.87, 0.88])
EDGE_C = np.array([0.10, 0.10, 0.12])
BASE_C = (0.80, 0.80, 0.82)        # PLA (colour not chosen yet)
LID_C = (0.95, 0.52, 0.16)
BOARD_C = (0.10, 0.42, 0.22)       # programmer solder mask
DIAL_C = (0.13, 0.13, 0.16)        # watch PCB
GOLD_C = (0.88, 0.72, 0.30)
BLACK_C = (0.12, 0.12, 0.13)
METAL_C = (0.78, 0.79, 0.82)
LED_C = (0.96, 0.95, 0.90)
SMD_C = (0.62, 0.52, 0.38)
CELL_C = (0.72, 0.73, 0.76)
TRAY_C = (0.30, 0.31, 0.34)        # PLA (colour not chosen yet)
SCREW_C = (0.16, 0.16, 0.18)       # black-oxide socket caps

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


# ---------- scene parts ----------
def box(x, y, z0, sx, sy, sz, rot=0.0):
    return Pos(x, y, z0) * Rot(0, 0, rot) * Box(sx, sy, sz, align=MIN)


def cyl(x, y, z0, d, h):
    return Pos(x, y, z0) * Cylinder(d / 2, h, align=MIN)


def programmer():
    """Board (top copper at z = 0) + main parts, from programmer.kicad_pcb / positions.csv."""
    x0, y1 = J.to_jig((104.14, 80.01))
    x1, y0 = J.to_jig((164.084, 126.746))
    board = Pos((x0 + x1) / 2, (y0 + y1) / 2, -1.6) * Box(x1 - x0, y1 - y0, 1.6, align=MIN)
    board = fillet(board.edges().filter_by(lambda e: abs(e.tangent_at(0).Z) > 0.9), 1.27)
    for x, y in J.holes:                                            # H1-H4
        board -= cyl(x, y, -2, 3.2, 3)
    items = [(board, BOARD_C)]
    pads = [cyl(x, y, 0, 2.0, 0.04) for x, y in J.pins]
    items.append((sum(pads[1:], pads[0]), GOLD_C))
    parts = []
    pos = [p for p in (HERE.parent / "PCB/production/positions.csv",            # copy in the watches repo (HW/3Dprint)
                       HERE.parents[1] / "watches/wrist_watch/programming_jig/HW/PCB/production/positions.csv")
           if p.exists()][0]
    with open(pos, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            x, y = J.to_jig((float(row["Mid X"]), -float(row["Mid Y"])))
            rot = float(row["Rotation"])
            ref = row["Designator"]
            if ref[0] in "CR":
                parts.append((box(x, y, 0, 2.0, 1.25, 0.6, rot), SMD_C if ref[0] == "C" else BLACK_C))
            elif ref == "U1":
                parts.append((box(x, y, 0, 3.0, 3.0, 0.9, rot), BLACK_C))
            elif ref == "U2":
                parts += [(box(x, y, 0, 6.5, 3.5, 1.6, rot), BLACK_C), (box(x + 3.15, y, 0, 1.0, 3.2, 0.3), METAL_C)]
            elif ref == "J14":
                parts.append((box(x - 0.2, y, 0, 7.3, 8.9, 3.2, 0), METAL_C))
    for pads in J.HEADERS.values():                                 # 2x3 male headers, 45 deg
        P = np.array([J.to_jig(p) for p in pads])
        c = P.mean(0)
        a = np.degrees(np.arctan2(*(P[1] - P[0])[::-1]))
        parts.append((box(c[0], c[1], 0, 7.62, 5.08, 2.5, a + 90), BLACK_C))
        pins = [box(px, py, -1.6 - 3.0, 0.64, 0.64, 8.5 + 4.6, a) for px, py in P]   # 3 mm below the board
        parts.append((sum(pins[1:], pins[0]), GOLD_C))
    return items + parts


def pogo_pins(compress):
    """Mill-Max 0907 stand-ins, plunger pushed down by `compress`."""
    out = []
    for x, y in J.pins:
        top = J.PIN_HEIGHT - compress
        body = (cyl(x, y, 0, J.FLANGE_D, J.FLANGE_H) + cyl(x, y, 0, 1.63, 0.71) + cyl(x, y, 0, J.BARREL_D, 5.03))
        plunger = cyl(x, y, 5.0, 1.07, top - 5.0 - 0.5) + Pos(x, y, top - 0.53) * Sphere(0.535)
        out.append(body + plunger)
    return [(sum(out[1:], out[0]), GOLD_C)]


def dial(z_bot):
    """Watch PCB: back face at z_bot, LED side up (layout from the EAGLE gerbers)."""
    zt = z_bot + J.PCB_T
    items = [(cyl(0, 0, z_bot, 2 * J.PCB_R, J.PCB_T), DIAL_C)]
    items.append((box(0, 11.38, zt, 7.0, 7.0, 1.0), BLACK_C))                               # TQFP-32
    items.append((box(0, -10.76, zt, 7.5, 10.3, 2.4), BLACK_C))                             # SOIC-16
    items.append((box(-7.7, 11.0, zt, 2.0, 3.6, 0.8), SMD_C))
    leds = [(-2.3, 4.5), (6.9, 4.5), (-2.3, 2.0), (2.3, 2.0), (6.9, 2.0)] + \
           [(x, y) for y in (-0.5, -3.0) for x in (-6.9, -2.3, 2.3, 6.9)]
    l = [box(x, y, zt, 1.6, 0.8, 0.6) for x, y in leds]
    items.append((sum(l[1:], l[0]), LED_C))
    r_in = 14.3 - 1.6                                                                       # side button (ESTIMATED)
    btn = box((r_in + J.PCB_R) / 2, 0, zt, J.PCB_R - r_in, J.BTN_W, J.BTN_H - 0.6)
    act = Pos(J.PCB_R, 0, zt + 1.5) * Rot(0, 90, 0) * Cylinder(1.0, J.BTN_PROUD, align=MIN)
    items += [(btn, METAL_C), (act, BLACK_C)]
    items.append((cyl(-0.66, -0.83, z_bot - 3.6, 20.0, 3.2), CELL_C))                       # CR2032 (holder omitted)
    return items


def hardware():
    """2x M3 x 10 socket caps seated in the base counterbores, 2x M3 nuts in the tray pockets."""
    screws, nuts = [], []
    top = J.SEAT_Z + J.SCREW_HEAD_H
    for x, y in J.screws:
        head = cyl(x, y, J.SEAT_Z, J.SCREW_HEAD_D, J.SCREW_HEAD_H) - J.hexagon(x, y, 2.5, top - 1.5, top + 1)
        screws.append((head + cyl(x, y, J.SEAT_Z - J.SCREW_LEN, 3.0, J.SCREW_LEN), SCREW_C))
        nut = J.hexagon(x, y, 5.5, J.NUT_CEIL - J.NUT_H, J.NUT_CEIL) - cyl(x, y, J.NUT_CEIL - J.NUT_H - 1, 3.0, J.NUT_H + 2)
        nuts.append((nut, METAL_C))
    return screws, nuts


def jig(lid_rot, lid_dz, lid_extra=0.0):
    base = J.base_part()
    lid = Pos(0, 0, lid_dz + lid_extra) * (Rot(0, 0, lid_rot) * J.lid_part())
    return [(base, BASE_C)], [(lid, LID_C)]


def lifted(items, dz):
    return [(Pos(0, 0, dz) * p, c) for p, c in items]


# ---------- rasteriser (from ../watch_holder/render.py) ----------
def tessellate(items):
    tris, cols, fids, fid = [], [], [], 0
    for part, col in items:
        for f in part.faces():
            v, t = f.tessellate(0.03, 0.15)
            if not t:
                continue
            v = np.array([[q.X, q.Y, q.Z] for q in v])
            tr = v[np.array(t)]
            tris.append(tr)
            cols.append(np.tile(col, (len(tr), 1)))
            fids.append(np.full(len(tr), fid))
            fid += 1
    return np.concatenate(tris), np.concatenate(cols), np.concatenate(fids)


def camera(centre, radius, azim, elev, dist):
    a, e = np.radians(azim), np.radians(elev)
    d = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    eye = centre + d * radius * dist
    fwd = -d
    right = np.cross(fwd, [0, 0, 1.0]); right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return eye, right, up, fwd


def box_blur(a, r):
    c = np.cumsum(np.pad(a, ((r + 1, r), (0, 0)), mode="edge"), 0)
    a = (c[2 * r + 1:] - c[:-2 * r - 1]) / (2 * r + 1)
    c = np.cumsum(np.pad(a, ((0, 0), (r + 1, r)), mode="edge"), 1)
    return (c[:, 2 * r + 1:] - c[:, :-2 * r - 1]) / (2 * r + 1)


def render(items, path, azim=-60, elev=32, title=None, sub=None, dist=3.4, frame=None):
    T, C, F = tessellate(items)
    pts = T.reshape(-1, 3) if frame is None else np.array(frame)
    centre = (pts.min(0) + pts.max(0)) / 2
    radius = np.linalg.norm(pts.max(0) - pts.min(0)) / 2
    eye, right, up, fwd = camera(centre, radius, azim, elev, dist)

    rel = T - eye
    x, y, z = rel @ right, rel @ up, rel @ fwd
    u, v = x / z, y / z
    W2, H2 = W_PX * SS, H_PX * SS
    top_margin = 0.14 if title else 0.05
    fr = (np.array(pts) - eye)
    fu, fv = (fr @ right) / (fr @ fwd), (fr @ up) / (fr @ fwd)
    umin, umax, vmin, vmax = fu.min(), fu.max(), fv.min(), fv.max()
    s = min(W2 * 0.90 / (umax - umin), H2 * (0.92 - top_margin) / (vmax - vmin))
    px = (u - (umin + umax) / 2) * s + W2 / 2
    py = H2 * (top_margin + 0.95) / 2 - (v - (vmin + vmax) / 2) * s
    iz = 1.0 / z

    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
    to_cam = eye - T.mean(1)
    n[(n * to_cam).sum(1) < 0] *= -1
    view = to_cam / np.linalg.norm(to_cam, axis=1)[:, None]
    key = -0.45 * right + 0.65 * up - 0.60 * fwd; key /= np.linalg.norm(key)
    fill = 0.70 * right + 0.10 * up - 0.70 * fwd; fill /= np.linalg.norm(fill)
    lam = 0.38 + 0.55 * np.clip(n @ key, 0, 1) + 0.20 * np.clip(n @ fill, 0, 1)
    h = (key[None, :] + view); h /= np.linalg.norm(h, axis=1)[:, None]
    spec = 0.22 * np.clip((n * h).sum(1), 0, 1) ** 40
    shade = np.clip(C * lam[:, None] + spec[:, None], 0, 1)

    zbuf = np.zeros((H2, W2))
    img = np.zeros((H2, W2, 3))
    ids = np.full((H2, W2), -1)
    for k in range(len(T)):
        xs, ys, ws = px[k], py[k], iz[k]
        x0, x1 = int(max(np.floor(xs.min()), 0)), int(min(np.ceil(xs.max()), W2 - 1))
        y0, y1 = int(max(np.floor(ys.min()), 0)), int(min(np.ceil(ys.max()), H2 - 1))
        if x1 < x0 or y1 < y0:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        d = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
        if abs(d) < 1e-9:
            continue
        b0 = ((ys[1] - ys[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gy - ys[2])) / d
        b1 = ((ys[2] - ys[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gy - ys[2])) / d
        b2 = 1 - b0 - b1
        inside = (b0 >= -1e-6) & (b1 >= -1e-6) & (b2 >= -1e-6)
        if not inside.any():
            continue
        wz = b0 * ws[0] + b1 * ws[1] + b2 * ws[2]
        zb = zbuf[y0:y1 + 1, x0:x1 + 1]
        m = inside & (wz > zb)
        zb[m] = wz[m]
        img[y0:y1 + 1, x0:x1 + 1][m] = shade[k]
        ids[y0:y1 + 1, x0:x1 + 1][m] = F[k]

    grad = np.linspace(0, 1, H2)[:, None, None]
    bg = BG_TOP * (1 - grad) + BG_BOT * grad
    mask = (ids >= 0).astype(float)
    dy, dx = int(0.012 * H2), int(0.008 * W2)                    # shifted copy, no wrap-around
    sh = np.zeros_like(mask)
    sh[dy:, dx:] = mask[:-dy, :-dx]
    for r in (24, 24, 24):
        sh = box_blur(sh, r)
    bg = bg * (1 - 0.28 * sh[:, :, None])
    out = np.where(mask[:, :, None] > 0, img, bg)

    e = np.zeros((H2, W2), bool)
    e[:, :-1] |= ids[:, :-1] != ids[:, 1:]
    e[:-1, :] |= ids[:-1, :] != ids[1:, :]
    e[:, 1:] |= e[:, :-1].copy()
    e[1:, :] |= e[:-1, :].copy()
    lum = out[..., :3].mean(-1)
    dark = e & (lum < 0.35)
    light = e & ~dark
    out[light] = out[light] * 0.25 + EDGE_C * 0.75
    out[dark] = out[dark] * 0.45 + 0.55 * np.array([0.52, 0.52, 0.55])

    out = out.reshape(H_PX, SS, W_PX, SS, 3).mean((1, 3))
    fig = plt.figure(figsize=(W_PX / 100, H_PX / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1]); ax.imshow(out); ax.set_axis_off()
    if title:
        fig.text(0.035, 0.95, title, fontsize=26, weight="bold", color="#2b2b2b", va="top")
    if sub:
        fig.text(0.035, 0.895, sub, fontsize=15, color="#555", va="top")
    fig.savefig(path, dpi=100)
    plt.close(fig)
    print("wrote", path.name, f"({len(T)} tris)")


def cut(items, x0):
    """Keep the x < x0 half (cutaway)."""
    keep = Pos(x0 - 100, 0, -50) * Box(200, 400, 200, align=(Align.CENTER, Align.CENTER, Align.MIN))
    out = []
    for p, c in items:
        q = p & keep
        if q.faces():
            out.append((q, c))
    return out


def main():
    board = programmer()
    tray = [(J.tray_part(), TRAY_C)]
    screws, nuts = hardware()
    base, lid_locked = jig(0.0, 0.0)
    _, lid_open = jig(J.LOCK_ANGLE, J.LOCK_DROP)
    lock = f"{J.LOCK_ANGLE:.0f}°"
    frame = [(-28, -28, -8), (41, 28, 16)]                          # tray outline: same for open / locked
    view = dict(azim=-62, elev=34, dist=3.0, frame=frame)
    stack = tray + nuts + board + base + screws

    render(stack + pogo_pins(J.COMPRESS - J.LOCK_DROP) + dial(J.PCB_BOT + J.LOCK_DROP) + lid_open,
           HERE / "renders" / "render_unlocked.png", **view,
           title="Unlocked: lid dropped in at the slots",
           sub=f"lid ticks line up with the base ticks; the dial rests on the pins (just touching). "
               f"Turn the lid {lock} clockwise to lock.")
    render(stack + pogo_pins(J.COMPRESS) + dial(J.PCB_BOT) + lid_locked,
           HERE / "renders" / "render_locked.png", **view,
           title=f"Locked: twisted {lock} clockwise, clicked into the end pocket",
           sub=f"lip holds the dial rim down: pogo pins compressed {J.COMPRESS} mm "
               f"({J.COMPRESS / J.PIN_STROKE:.0%} of stroke). LEDs stay visible.")
    render(lifted(tray, -16) + lifted(nuts, -30) + board + pogo_pins(0.0) + lifted(base, 14) + lifted(screws, 38)
           + lifted(dial(J.PCB_BOT + J.LOCK_DROP), 28) + lifted(lid_open, 48),
           HERE / "renders" / "render_exploded.png", azim=-62, elev=26, dist=3.0,
           title="Exploded: 3 printed pieces",
           sub="nuts into the tray | tray (2 pegs) holds the programmer | base on the pegs, 2x M3 x 10 from the top | "
               "dial | twist-lock lid")
    x_cut = J.screws[0][0]                                          # through both screw axes (H1 / H3)
    render(cut(stack + pogo_pins(J.COMPRESS) + dial(J.PCB_BOT) + lid_locked, x_cut),
           HERE / "renders" / "render_cutaway.png", azim=0, elev=14, dist=3.2,
           frame=[(x_cut - 6, -28, -8), (x_cut, 28, 15)],
           title="Cutaway through the screws, locked",
           sub="M3 x 10 from the top: base ear -> programmer -> nut in the tray | lip on the dial rim | "
               "pegs and pogo pins behind")


if __name__ == "__main__":
    main()
