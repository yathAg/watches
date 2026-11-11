"""Programming jig V2 for the binary wrist watch PCB ("dial"), 3 pieces:
  TRAY  - holds the programmer board; 2 pegs up through H2/H4, M3 hex-nut pockets under H1/H3.
  BASE  - sits on the board, guides the pogo pins, centres + keys the dial; 2 M3 x 10 socket-cap screws
          from the top (counterbores in two ears) through H1/H3 into the tray nuts clamp base+board+tray.
  LID   - twist-lock clamp ring that pulls the dial down onto the pins.
V1 (2 pieces, base self-tapped from below with 4 screws): archive/V1/ (watch_programmer_jig_V1.py, jig_*_V1.*).

Sources
  Programmer : watches/wrist_watch/programming_jig/HW/PCB/programmer.kicad_pcb (KiCad, y down). Dial silk
               circle centre (127.4318, 103.2808) r 17.62 = jig origin. M3 NPTH H1-H4 (3.2) on r 20.574.
               Board edges are tangent to r 23.25 on the left/top/bottom -> jig OD 46.5.
               Pogo pads: serial row J1-J6 (RTS TXD RXD 3V3 CTS GND, CP2102N) and ISP group J7-J12 (J13).
  Watch PCB  : watches/wrist_watch/instructibles_archive/FLVZX1QJUSNBBNV.zip (EAGLE gerbers): round, D 35.2, 1.57 thick.
               Its back-side pads match both programmer groups (<= 0.05 mm, not mirrored), so the dial
               sits LED side up at 0 deg. Top-side parts nearest the edge: TQFP-32 / SOIC-16 pads reach
               r 16.65, SOIC body r 16.3 x 2.65 tall. Side button (2 THT pins at (14.3, +-1.27)) = 3 o'clock.
  Pogo pins  : Mill-Max 0907-2-15-20-75-14-11-0 (programming_jig/HW/PCB/*.pdf): height 7.493, stroke 1.397,
               plunger D 1.07, barrel D 1.50 up to 5.03, collar D 1.63 up to 0.71, flange D 1.83 x 0.41,
               #75 spring 60 gf @ 0.7 mm (~90 gf at the 0.89 mm used here).
  ESTIMATED  : side button size (BTN_*), measure it; coin-cell holder assumed < 6 mm tall.

Frame: jig origin = dial centre, +x right, +y toward the programmer's top edge (KiCad -y), z = 0 on the
programmer's top copper. Lid modelled in its LOCKED position. Twist CLOCKWISE (seen from above) to lock.

How it works: the lid lip sets the dial's top face at PCB_TOP, so the pins are compressed PIN_HEIGHT -
PCB_BOT = 0.89 mm (64 % of stroke). Dropped on at the entry slots, the lid sits 0.9 mm higher (pins just
touching); a ramp in each base groove pulls it down while twisting, then the lugs ride 0.3 mm lower
(DETENT) and pop up into the end pocket = click + end stop.

Print (exported files already oriented, min Z = 0), PLA, 0.2 mm layers, 3-4 walls, no supports:
  jig_tray - floor on the bed. Nut pockets open on the bed face (6 mm bridge over each).
  jig_base - programmer face on the bed. Flange pockets are tiny bridges, groove roofs 1.0 mm overhangs.
  jig_lid  - top face on the bed, lip and 3 lug tabs up. Lugs are 0.9 mm overhangs. Engraved arrow on the bed face.
Lid lugs are unevenly spaced (fits one way only, button cut-out over the button); the base has flats
facing the J13 / J15 headers so a 2x3 IDC plug clears it, and the lug tabs never sweep over them or the ears.
Hardware: 2x M3 x 10 socket cap (DIN 912) + 2x M3 hex nuts (pressed into the tray from below).
6 or 12 Mill-Max pins, soldered to the pads (or just dropped into the base pockets and clamped by it).
Assembly: nuts into the tray -> programmer board onto the pegs -> base onto the pegs -> 2 screws from the top.

    python watch_programmer_jig.py  ->  jig_tray_V1, jig_base_V2, jig_lid_V2 (print/*.stl, cad/*.step),
                                        print/jig_plate_V2.3mf (all three on one bed, grey), renders/preview.png
"""
from math import atan2, cos, degrees, hypot, radians, sin
from pathlib import Path

import numpy as np
from build123d import (Axis, Compound, Polyline, Pos, Rot, chamfer, export_step, export_stl, extrude,
                       make_face, revolve)
from manifold3d import CrossSection, JoinType, Manifold, Mesh

HERE = Path(__file__).parent
SEG = 120

# --- programmer board (KiCad coordinates, mm, y down) ---
ORIGIN = (127.4318, 103.2808)                     # F.SilkS dial circle centre
MOUNT_HOLES = [(127.381, 82.677), (106.807, 103.251), (127.381, 123.825), (147.955, 103.251)]  # H1 H2 H3 H4
PADS_SERIAL = [(113.9698, y) for y in (96.266, 99.06, 101.854, 104.648, 107.442, 110.236)]       # J1..J6
PADS_ISP = [(134.2898, 92.3798), (136.271, 90.424), (136.271, 94.361),                            # J7 J8 J9
            (138.2522, 92.3798), (138.2522, 96.3422), (140.208, 94.361)]                          # J10 J11 J12
HEADERS = {   # 2x3 THT headers (PTH drill file): ISP J13 and serial J15, both at 45 deg
    "J13": [(144.379, 84.46), (146.175, 82.664), (146.175, 86.256), (147.971, 84.46), (147.971, 88.052), (149.767, 86.256)],
    "J15": [(106.152, 121.036), (107.948, 119.24), (107.948, 122.832), (109.744, 121.036), (109.744, 124.628), (111.54, 122.832)]}
PLUG_MARGIN = (2.55, 2.45)                         # 2x3 IDC plug outline beyond the pad grid (rows, columns)
BOARD_EDGES = (104.14, 80.01, 164.084, 126.746)    # Edge.Cuts x0 y0 x1 y1, R1.27 corners
BOARD_T, BOARD_CORNER_R = 1.6, 1.27
USB_C = (158.8599, 107.696)                        # J14 (positions.csv); receptacle opens at the right edge
PTH_PINS = [p for pads in HEADERS.values() for p in pads]                                       # 1.0 drills
USB_SLOTS = [((156.689, 103.376), (157.639, 103.376)), ((156.689, 112.016), (157.639, 112.016)),
             ((160.994, 103.376), (161.694, 103.376)), ((160.994, 112.016), (161.694, 112.016))]  # shell legs
SCREW_HOLES, PEG_HOLES = (0, 2), (1, 3)            # indices into MOUNT_HOLES: screws H1/H3, pegs H2/H4

# --- watch PCB ---
PCB_R = 17.6                                       # D 35.2 (gerber job file)
PCB_T = 1.57

# --- pogo pin (Mill-Max 0907-2-15-20-75-14-11-0) ---
PIN_HEIGHT = 7.493                                 # free height above the pad
PIN_STROKE = 1.397
BARREL_D = 1.50
FLANGE_D, FLANGE_H = 1.83, 0.41
COMPRESS = 0.89                                    # working compression (~2/3 stroke)

# --- stack (z) ---
PCB_BOT = round(PIN_HEIGHT - COMPRESS, 2)          # 6.60 dial back face when locked
PCB_TOP = PCB_BOT + PCB_T                          # 8.17 lip contact plane
Z_TOP = 11.0                                       # base top
LID_GAP = 0.6                                      # base top -> lid plate when locked (> DETENT)
PLATE_T = 2.0

# --- base ---
BASE_R = 23.25                                     # tangent to the programmer's left/top/bottom edges
BORE_R = PCB_R + 0.2                               # centring band around the dial
BAND_TOP = PCB_TOP - 0.3                           # band stops below the lip
UPPER_R = 18.2                                     # bore above the band (lip OD + 0.3)
GUIDE_D = BARREL_D + 0.4                           # small vertical holes print ~0.2 undersize -> ~0.2 clr
POCKET_D, POCKET_H = 2.6, 0.8                      # flange + solder fillet pocket (pad is D 2.0)
GUIDE_TOP = 4.8                                    # guide islands top; 1.8 mm below the dial's back
ISLAND_WALL = 1.6                                  # plastic around each guide hole
CHAMFER = 0.4                                      # bed-side edge chamfer

# --- screws (H1/H3, from the top) and pegs (H2/H4) ---
SCREW_LEN = 10.0                                   # M3 x 10 socket cap, DIN 912
SCREW_HEAD_D, SCREW_HEAD_H = 5.5, 3.0
CLEAR_D = 3.4                                      # M3 clearance
CBORE_D = 6.2                                      # head counterbore
SEAT_Z = 3.2                                       # head seat in the base; head top 6.2 < dial back 6.6
EAR_R = 4.4                                        # boss around each counterbore (closes the outer wall)
PEG_D = 3.0                                        # programmer NPTH is 3.2
PEG_TOP = 3.0                                      # peg reaches 3 mm into the base
SOCKET_D = 3.3
NUT_AF, NUT_H = 5.75, 2.4                          # M3 nut 5.5 AF: snug press fit so it stays in the pocket

# --- tray (holds the programmer board) ---
TRAY_CLR = 0.3                                     # board -> pocket wall, per side
TRAY_WALL = 2.0
TRAY_GAP = 3.6                                     # under the board: THT pins stick out ~3 mm
TRAY_FLOOR = 2.0
TRAY_Z0 = -BOARD_T - TRAY_GAP - TRAY_FLOOR         # -7.2 tray bottom
LEDGE_W = 1.0                                      # perimeter ledge under the board edge
PTH_RELIEF_D = 3.0                                 # ledge/posts relieved around every through-hole joint
BOSS_D = 9.6                                       # under H1-H4 (holds the nut pocket)
POST_D = 5.0                                       # support posts under the two pogo groups
NUT_CEIL = -4.0                                    # nut pocket ceiling (2.4 mm of plastic up to the board)
USB_CUT_W, USB_CUT_Z = 14.0, -3.5                  # right-wall cut-out for the USB-C plug overmould

# --- side button notch = rotation key (ESTIMATED: measure the button) ---
BTN_W = 6.0                                        # width along the PCB edge
BTN_PROUD = 1.5                                    # sticks out past the PCB edge
BTN_H = 3.5                                        # height above the PCB top
BTN_CLR = 0.5                                      # per side
BTN_Z0 = BAND_TOP                                  # notch starts here (button is on the top side)
BTN_R_OUT = PCB_R + BTN_PROUD + 0.4

# --- bayonet (3 lugs on the lid, L-grooves in the base) ---
LOCK_ANGLE = 28.0                                  # twist from entry to locked (clockwise)
LUG_ANGLES = (113.5, 164.5, 319.0)                 # lug centres, locked (deg). Uneven = lid fits one way only;
                                                   # twist sweeps stay clear of the J13/J15 plugs and the ears
SLOT_ANGLES = tuple(a + LOCK_ANGLE for a in LUG_ANGLES)   # entry slot centres (base frame)
LUG_ARC = 6.0                                      # lug length along the circumference
LUG_H = 1.6
LUG_IN = 0.9                                       # radial reach of the lug inside the skirt
GROOVE_DEPTH = 1.0
GROOVE_R = BASE_R - GROOVE_DEPTH                   # 22.25 groove root
LID_CLR = 0.25                                     # skirt inner radius over the base
SKIRT_T = 1.6
LUG_TOP_LOCK = 9.4                                 # lug top = pocket roof when locked
LOCK_DROP = 0.9                                    # entry height above locked (pins just touching)
DETENT = 0.3                                       # extra dip before the pocket
GROOVE_FLOOR = 7.2
SLOT_CLR = 0.5                                     # mm, each side of the lug in slot and pocket

# --- lid ---
LIP_IN = 16.9                                      # clears TQFP/SOIC pads (r 16.65)
LIP_OUT = UPPER_R - 0.3                            # 17.9
SKIRT_IN = BASE_R + LID_CLR                        # 23.5
LID_R = SKIRT_IN + SKIRT_T                         # 25.1 (lug tabs only)
LID_PLATE_R = 22.2                                 # round plate: inside the J13/J15 plug keep-outs
TAB_MARGIN = 1.5                                   # deg, tab beyond the lug each side
PLATE_Z0 = Z_TOP + LID_GAP
PLATE_Z1 = PLATE_Z0 + PLATE_T
LUG_Z0 = LUG_TOP_LOCK - LUG_H                      # 7.8 skirt bottom
BTN_CUT_R = BTN_R_OUT + 0.5                        # lid cut-out clears the button through the twist
ENGRAVE = 0.6

PRINT, CAD = HERE / "print", HERE / "cad"
OUT_BASE, OUT_LID, OUT_TRAY = PRINT / "jig_base_V2", PRINT / "jig_lid_V2", PRINT / "jig_tray_V1"  # .step in CAD
OUT_PNG = HERE / "renders" / "preview.png"
OUT_PLATE = PRINT / "jig_plate_V2.3mf"               # all 3 parts on one bed, Bambu Studio project, one colour
PLATE_COLOUR = "#8E9094"                            # grey PLA
PLATE_GAP = 8.0


# ---------- helpers ----------
def to_jig(p):
    """KiCad (y down) -> jig frame."""
    return (p[0] - ORIGIN[0], -(p[1] - ORIGIN[1]))


def face_from_xy(pts):
    return make_face(Polyline(*[(x, y, 0) for x, y in pts], close=True))


def prism(pts, z0, z1):
    return Pos(0, 0, z0) * extrude(face_from_xy(pts), z1 - z0)


def sector(r0, r1, a0, a1, z0, z1):
    """Annular sector, angles in degrees (a0 < a1)."""
    n = max(3, int(abs(a1 - a0) / 1.5) + 2)
    ang = np.radians(np.linspace(a0, a1, n))
    pts = [(r1 * cos(a), r1 * sin(a)) for a in ang] + [(r0 * cos(a), r0 * sin(a)) for a in ang[::-1]]
    return prism(pts, z0, z1)


def disc(x, y, d, z0, z1, n=48):
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return prism([(x + d / 2 * cos(a), y + d / 2 * sin(a)) for a in ang], z0, z1)


def revolve_rz(profile):
    return revolve(make_face(Polyline(*[(r, 0, z) for r, z in profile], close=True)), Axis.Z)


def hull_pts(circles):
    """2D convex hull of circles [(x, y, r)] -> outline points (via manifold CrossSection)."""
    cs = CrossSection.batch_hull([CrossSection.circle(r, 48).translate((x, y)) for x, y, r in circles])
    return [tuple(p) for p in cs.to_polygons()[0]]


def ang_w(length, r):
    return degrees(length / r)


def plug_keepout(pads, margin=PLUG_MARGIN, n=60):
    """Sample points of a 2x3 IDC plug outline (jig frame) around the header pads."""
    P = np.array([to_jig(p) for p in pads])
    c = P.mean(0)
    u = (P[1] - P[0]) / np.linalg.norm(P[1] - P[0])
    v = (P[2] - P[0]) / np.linalg.norm(P[2] - P[0])
    pu, pv = (P - c) @ u, (P - c) @ v
    return np.array([c + a * u + b * v for a in np.linspace(pu.min() - margin[0], pu.max() + margin[0], n)
                     for b in np.linspace(pv.min() - margin[1], pv.max() + margin[1], n)])


# ---------- geometry ----------
pins_serial = [to_jig(p) for p in PADS_SERIAL]
pins_isp = [to_jig(p) for p in PADS_ISP]
pins = pins_serial + pins_isp
holes = [to_jig(p) for p in MOUNT_HOLES]
screws = [holes[i] for i in SCREW_HOLES]
pegs = [holes[i] for i in PEG_HOLES]
LUG_W = ang_w(LUG_ARC, BASE_R)                     # lug angular width
SLOT_HW = LUG_W / 2 + ang_w(SLOT_CLR, BASE_R)
BTN_HW = ang_w(BTN_W / 2 + BTN_CLR, PCB_R)
TAB_HW = LUG_W / 2 + TAB_MARGIN
KEEPOUTS = {k: plug_keepout(v) for k, v in HEADERS.items()}
FLATS = {k: (atan2(*k_.mean(0)[::-1]), (k_ @ (k_.mean(0) / np.linalg.norm(k_.mean(0)))).min() - 0.3)
         for k, k_ in KEEPOUTS.items()}             # header direction (rad), flat distance from centre


def roof(theta, t0):
    """Groove roof z at base angle theta for the groove whose slot is centred at t0."""
    lock_c = t0 - LOCK_ANGLE
    pocket_hi = lock_c + LUG_W / 2 + ang_w(0.3, BASE_R)
    ramp_hi = t0 - SLOT_HW
    if theta <= pocket_hi:
        return LUG_TOP_LOCK
    f = (theta - pocket_hi) / (ramp_hi - pocket_hi)
    return LUG_TOP_LOCK - DETENT + f * (LOCK_DROP + DETENT)


def base_part():
    body = revolve_rz([(BORE_R + CHAMFER, 0), (BASE_R - CHAMFER, 0), (BASE_R, CHAMFER),
                       (BASE_R, Z_TOP - 0.6), (BASE_R - 0.6, Z_TOP), (UPPER_R + 0.4, Z_TOP),
                       (UPPER_R, Z_TOP - 0.4), (UPPER_R, BAND_TOP), (BORE_R + 0.2, BAND_TOP),
                       (BORE_R, BAND_TOP - 0.2), (BORE_R, CHAMFER)])
    # ears around the screw counterbores (overhang the board edge onto the tray wall), kept out of the bore
    void = revolve_rz([(0, -1), (BORE_R, -1), (BORE_R, BAND_TOP), (UPPER_R, BAND_TOP), (UPPER_R, Z_TOP + 1),
                       (0, Z_TOP + 1)])
    body = body + (Compound([disc(x, y, 2 * EAR_R, 0, Z_TOP, SEG) for x, y in screws]) - void)
    # guide islands, each merged into the wall
    r_isl = GUIDE_D / 2 + ISLAND_WALL
    row = [(x, y, r_isl) for x, y in pins_serial] + [(-BORE_R - 0.5, y, r_isl) for _, y in pins_serial[::5]]
    cx, cy = np.mean([p[0] for p in pins_isp]), np.mean([p[1] for p in pins_isp])
    a = atan2(cy, cx)
    isp = [(x, y, r_isl) for x, y in pins_isp] + [((BORE_R + 0.5) * cos(a + s), (BORE_R + 0.5) * sin(a + s), 3.0)
                                                  for s in (-0.12, 0.12)]
    body = body + prism(hull_pts(row), 0, GUIDE_TOP) + prism(hull_pts(isp), 0, GUIDE_TOP)
    cuts = []
    for x, y in pins:
        cuts += [disc(x, y, GUIDE_D, -1, GUIDE_TOP + 1, 32), disc(x, y, POCKET_D, -1, POCKET_H, 32)]
    for x, y in screws:                             # M3 clearance + head counterbore from the top
        cuts += [disc(x, y, CLEAR_D, -1, Z_TOP + 1, 48), disc(x, y, CBORE_D, SEAT_Z, Z_TOP + 1, 64)]
    for x, y in pegs:                               # sockets for the tray pegs
        cuts.append(disc(x, y, SOCKET_D, -1, PEG_TOP + 0.4, 48))
    # button notch / rotation key at 0 deg
    cuts.append(sector(BORE_R - 1.0, BTN_R_OUT, -BTN_HW, BTN_HW, BTN_Z0, Z_TOP + 1))
    # bayonet: entry slot + ramped groove (1 deg steps; prints in 0.2 mm layers anyway)
    for t0 in SLOT_ANGLES:
        cuts.append(sector(GROOVE_R, BASE_R + 1, t0 - SLOT_HW, t0 + SLOT_HW, GROOVE_FLOOR, Z_TOP + 1))
        end = t0 - LOCK_ANGLE - SLOT_HW
        edges = np.arange(end, t0 - SLOT_HW, 1.0).tolist() + [t0 - SLOT_HW + 0.01]
        for a0, a1 in zip(edges[:-1], edges[1:]):
            cuts.append(sector(GROOVE_R, BASE_R + 1, a0, a1 + 0.01, GROOVE_FLOOR, roof(a0, t0)))
        # alignment tick below the slot (outer wall)
        cuts.append(sector(BASE_R - 0.5, BASE_R + 1, t0 - 1.0, t0 + 1.0, 1.0, GROOVE_FLOOR - 1.2))
    # flats facing J13 / J15 so a plugged-in IDC cable clears the base
    for a, d in FLATS.values():
        u, v = np.array([cos(a), sin(a)]), np.array([-sin(a), cos(a)])
        pts = [tuple(d * u + s * v) for s in (-30, 30)] + [tuple((d + 10) * u + s * v) for s in (30, -30)]
        cuts.append(prism(pts, -1, Z_TOP + 1))
    return body - Compound(cuts)


def lid_part():
    body = revolve_rz([(LIP_IN, PCB_TOP), (PCB_R, PCB_TOP), (LIP_OUT, PCB_TOP + 0.3), (LIP_OUT, PLATE_Z0),
                       (LID_PLATE_R, PLATE_Z0), (LID_PLATE_R, PLATE_Z1 - CHAMFER),
                       (LID_PLATE_R - CHAMFER, PLATE_Z1), (LIP_IN + CHAMFER, PLATE_Z1), (LIP_IN, PLATE_Z1 - CHAMFER)])
    lugs, cuts = [], []
    for c in LUG_ANGLES:                            # lug centres in the locked lid frame
        lugs += [sector(LID_PLATE_R - 0.5, LID_R, c - TAB_HW, c + TAB_HW, PLATE_Z0, PLATE_Z1),   # plate tab
                 sector(SKIRT_IN, LID_R, c - TAB_HW, c + TAB_HW, LUG_Z0, PLATE_Z0 + 0.01),      # skirt
                 sector(SKIRT_IN - LUG_IN, SKIRT_IN + 0.2, c - LUG_W / 2, c + LUG_W / 2, LUG_Z0, LUG_TOP_LOCK)]
        cuts.append(sector(LID_R - 0.5, LID_R + 1, c - 1.0, c + 1.0, LUG_Z0 - 1, PLATE_Z1 + 1))  # tick
    body = body + Compound(lugs)
    # button cut-out: lip + inner plate over the button for the whole twist
    m = 3.0
    cuts.append(sector(LIP_IN - 1, BTN_CUT_R, -LOCK_ANGLE - BTN_HW - m, BTN_HW + m, PCB_TOP - 1, PLATE_Z1 + 1))
    # engraved clockwise arrow on the top face (left side)
    ra, w = (LIP_IN + LID_PLATE_R) / 2, 1.2
    a_tail, a_head = 160.0, 115.0
    cuts.append(sector(ra - w / 2, ra + w / 2, a_head, a_tail, PLATE_Z1 - ENGRAVE, PLATE_Z1 + 1))
    h = radians(a_head)
    tip = (ra * cos(h - 0.13), ra * sin(h - 0.13))
    b1 = ((ra - 1.8) * cos(h + 0.02), (ra - 1.8) * sin(h + 0.02))
    b2 = ((ra + 1.8) * cos(h + 0.02), (ra + 1.8) * sin(h + 0.02))
    cuts.append(prism([tip, b2, b1], PLATE_Z1 - ENGRAVE, PLATE_Z1 + 1))
    return body - Compound(cuts)


def board_cs(grow=0.0):
    """Programmer outline (jig frame) grown by `grow`, as a manifold CrossSection."""
    x0, y1 = to_jig(BOARD_EDGES[:2])
    x1, y0 = to_jig(BOARD_EDGES[2:])
    r = BOARD_CORNER_R
    cs = CrossSection.square((x1 - x0 - 2 * r, y1 - y0 - 2 * r)).translate((x0 + r, y0 + r))
    return cs.offset(r + grow, JoinType.Round, circular_segments=SEG)


def cs_pts(cs):
    return [tuple(p) for p in cs.to_polygons()[0]]


def hexagon(x, y, af, z0, z1):
    r = af / 2 / cos(radians(30))                   # corners along x, flats face +-y (toward the wall)
    return prism([(x + r * cos(radians(60 * k)), y + r * sin(radians(60 * k))) for k in range(6)], z0, z1)


def tray_part():
    z_floor, z_board = TRAY_Z0 + TRAY_FLOOR, -BOARD_T
    outer = prism(cs_pts(board_cs(TRAY_CLR + TRAY_WALL)), TRAY_Z0, 0)
    outer = chamfer(outer.edges().group_by(Axis.Z)[0], CHAMFER)
    pocket = board_cs(TRAY_CLR)
    body = outer - prism(cs_pts(pocket), z_floor, 1)
    # supports up to the board's underside: perimeter ledge, bosses under H1-H4, posts under the pogo groups
    ledge = pocket - pocket.offset(-LEDGE_W - TRAY_CLR, JoinType.Round, circular_segments=SEG)
    sup = [prism(p, z_floor - 0.01, z_board) for p in ([tuple(q) for q in poly] for poly in ledge.to_polygons())]
    ledge_part = sup[0]
    for s in sup[1:]:                                # outer loop minus inner loop
        ledge_part = ledge_part - s if s.volume < ledge_part.volume else s - ledge_part
    groups = [np.mean(pins_serial, 0), np.mean(pins_isp, 0)]
    supports = ledge_part + Compound([disc(x, y, BOSS_D, z_floor - 0.01, z_board, SEG) for x, y in holes] +
                                     [disc(x, y, POST_D, z_floor - 0.01, z_board, 64) for x, y in groups])
    relief = [disc(*to_jig(p), PTH_RELIEF_D, z_floor + 0.01, z_board + 1, 32) for p in PTH_PINS]
    for a, b in USB_SLOTS:
        relief += [disc(*to_jig(a), PTH_RELIEF_D + 0.6, z_floor + 0.01, z_board + 1, 32),
                   disc(*to_jig(b), PTH_RELIEF_D + 0.6, z_floor + 0.01, z_board + 1, 32)]
    body = body + (supports - Compound(relief))
    # pegs through H2/H4 into the base sockets (0.3 mm tip chamfer)
    for x, y in pegs:
        body = body + Pos(x, y, 0) * revolve_rz([(0, z_board - 0.01), (PEG_D / 2, z_board - 0.01),
                                                 (PEG_D / 2, PEG_TOP - 0.3), (PEG_D / 2 - 0.3, PEG_TOP), (0, PEG_TOP)])
    cuts = []
    for x, y in screws:                              # screw clearance + hex nut pocket from below
        cuts += [disc(x, y, CLEAR_D, TRAY_Z0 - 1, 1, 48), hexagon(x, y, NUT_AF, TRAY_Z0 - 1, NUT_CEIL)]
    ux, uy = to_jig(USB_C)                           # USB-C plug overmould
    cuts.append(prism([(ux, uy - USB_CUT_W / 2), (ux + 20, uy - USB_CUT_W / 2), (ux + 20, uy + USB_CUT_W / 2),
                       (ux, uy + USB_CUT_W / 2)], USB_CUT_Z, 1))
    return body - Compound(cuts)


# ---------- checks / export / preview ----------
def to_manifold(part, tol=0.02):
    """B-rep -> watertight Manifold (tessellation duplicates vertices along face seams: weld them)."""
    v, t = part.tessellate(tol, 0.2)
    v = np.array([[p.X, p.Y, p.Z] for p in v])
    uniq, inv = np.unique(np.round(v, 5), axis=0, return_inverse=True)
    tri = inv.reshape(-1)[np.asarray(t)]
    tri = tri[(tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])]
    man = Manifold(Mesh(vert_properties=uniq.astype(np.float32), tri_verts=tri.astype(np.uint32)))
    assert man.status().name == "NoError", man.status()
    return man


def valid(part):
    iv = part.is_valid
    return iv() if callable(iv) else iv


def report(name, part, man):
    bb = part.bounding_box()
    vol = part.volume / 1000
    print(f"{name}: valid={valid(part)} genus={man.genus()} vol={vol:.2f} cm3 "
          f"(~{vol * 1.24 * 0.8:.1f} g PLA) size {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f}")


def checks():
    pin_r = [hypot(*p) for p in pins]
    for x, y in pegs:
        r = hypot(x, y)
        assert min(r - SOCKET_D / 2 - BORE_R, BASE_R - r - SOCKET_D / 2) >= 0.9, "peg socket wall < 0.9"
    for x, y in screws:
        r = hypot(x, y)
        assert r + EAR_R - (r + CBORE_D / 2) >= 1.2, "ear wall around the counterbore < 1.2"
        assert r - SCREW_HEAD_D / 2 > PCB_R + 0.1 or SEAT_Z + SCREW_HEAD_H < PCB_BOT - 0.2, "screw head hits dial"
    tip = SEAT_Z - SCREW_LEN
    assert tip <= NUT_CEIL - NUT_H - 0.2, f"screw tip {tip} does not pass the nut"
    assert tip >= TRAY_Z0 + 0.2, f"screw tip {tip} sticks out of the tray bottom"
    assert NUT_CEIL - NUT_H - TRAY_Z0 >= 0.3 and -BOARD_T - NUT_CEIL >= 2.0, "nut pocket placement"
    assert max(pin_r) + GUIDE_D / 2 + 1.0 < BORE_R, "guide hole too close to the bore"
    # angular keep-outs for the lid tabs (r 22.2-25.1, z >= 7.5) and the base grooves (r 22.25-24.25)
    def ang_span(pts, r_lo):
        pts = pts[np.hypot(pts[:, 0], pts[:, 1]) >= r_lo]
        a = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])) % 360
        return a
    circ = lambda x, y, r: np.array([(x + rr * cos(t), y + rr * sin(t)) for rr in np.linspace(0, r, 8)
                                     for t in np.linspace(0, 2 * np.pi, 90)])
    ears = np.vstack([circ(x, y, EAR_R) for x, y in screws])
    for c in LUG_ANGLES:
        t_lo, t_hi = c - TAB_HW - 1.5, c + LOCK_ANGLE + TAB_HW + 1.5
        g_lo, g_hi = c - SLOT_HW - 1.0, c + LOCK_ANGLE + SLOT_HW + 1.0
        inside = lambda a, lo, hi: (((a - lo) % 360) <= hi - lo).any()
        assert not inside(ang_span(ears, LID_PLATE_R - 0.5), t_lo, t_hi), f"lug tab {c} sweeps over an ear"
        assert not inside(ang_span(ears, GROOVE_R), g_lo, g_hi), f"groove {c} cuts an ear"
    assert LUG_TOP_LOCK - LUG_H - DETENT > GROOVE_FLOOR, "lug hits groove floor at the detent"
    assert COMPRESS + DETENT < PIN_STROKE, "pins bottom out at the detent"
    assert LID_GAP - DETENT >= 0.2, "lid plate hits the base top at the detent"
    assert Z_TOP - (LUG_TOP_LOCK + LOCK_DROP) >= 0.6, "groove roof wall too thin at the slot"
    for name, k in KEEPOUTS.items():               # lug-tab twist sweeps vs the header plugs
        ang = np.degrees(np.arctan2(k[:, 1], k[:, 0])) % 360
        near = np.hypot(k[:, 0], k[:, 1]) < LID_R + 0.5
        for c in LUG_ANGLES:
            lo, hi = c - TAB_HW - 1.5, c + LOCK_ANGLE + TAB_HW + 1.5
            assert not (((ang[near] - lo) % 360) <= hi - lo).any(), f"lug tab {c} sweeps over {name}"
        assert np.hypot(k[:, 0], k[:, 1]).min() > LID_PLATE_R + 0.2, f"lid plate hits {name} plug"
    for a, d in FLATS.values():
        for t0 in SLOT_ANGLES:                      # grooves must not run into the flats
            half = degrees(np.arccos(min(1, d / BASE_R)))
            gap = abs((degrees(a) - (t0 - LOCK_ANGLE / 2) + 180) % 360 - 180)
            assert gap > half + LOCK_ANGLE / 2 + SLOT_HW + 2, f"groove at {t0} hits a flat"
    print("flats", {k: (round(degrees(a), 1), round(d, 2)) for k, (a, d) in FLATS.items()})
    print(f"stack: dial back {PCB_BOT:.2f}, top {PCB_TOP:.2f}; pins {COMPRESS:.2f} mm locked, "
          f"{COMPRESS + DETENT:.2f} at detent, {COMPRESS - LOCK_DROP:+.2f} at entry (stroke {PIN_STROKE})")
    print(f"lug {LUG_W:.1f} deg, twist {LOCK_ANGLE} deg; pin radii {min(pin_r):.2f}..{max(pin_r):.2f}; "
          f"M3x{SCREW_LEN:.0f} tip z {tip:.1f} (nut {NUT_CEIL - NUT_H:.1f}..{NUT_CEIL}, tray bottom {TRAY_Z0:.1f})")


def preview(base_m, lid_m, tray_m):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    def add_mesh(ax, man, color, dz=0.0):
        m = man.to_mesh()
        v = np.asarray(m.vert_properties)[:, :3] + [0, 0, dz]
        tri = v[np.asarray(m.tri_verts)]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.45 + 0.55 * np.clip(n @ np.array([0.3, -0.4, 0.85]), 0, 1)
        ax.add_collection3d(Poly3DCollection(tri, facecolors=np.outer(shade, color), edgecolors="none"))

    def polys(ax, cs, **kw):
        for p in cs.to_polygons():
            p = np.vstack([p, p[:1]])
            ax.plot(p[:, 0], p[:, 1], **kw)

    fig = plt.figure(figsize=(18, 11))
    views = [("base", base_m, 0, (35, -60)), ("lid (locked, lifted 12 mm)", lid_m, 12, (35, -60)),
             ("tray", tray_m, 0, (40, -60))]
    for i, (title, man, dz, (el, az)) in enumerate(views):
        ax = fig.add_subplot(2, 3, i + 1, projection="3d")
        add_mesh(ax, man, np.array([0.35, 0.55, 0.85]) if "lid" in title else np.array([0.8, 0.8, 0.82]), dz)
        if title == "base":
            add_mesh(ax, lid_m, np.array([0.35, 0.55, 0.85]), 14)
        lim = (-26, 38) if title == "tray" else (-26, 26)
        ax.set_xlim(*lim); ax.set_ylim(lim[0], lim[1] - (12 if title == "tray" else 0))
        ax.set_zlim(-10, 42 if title == "base" else 30)
        ax.set_box_aspect((1, 1, 0.9)); ax.view_init(el, az); ax.set_title(title); ax.set_axis_off()
    # top view: base section at z=3 and z=9.5 + pins + pilots + dial
    ax = fig.add_subplot(2, 3, 4)
    polys(ax, base_m.slice(3.0), color="k", lw=0.8)
    polys(ax, base_m.slice(9.6), color="tab:orange", lw=0.8)
    polys(ax, lid_m.slice(PCB_TOP + 0.05), color="tab:blue", lw=0.8)
    for x, y in pins:
        ax.add_patch(Circle((x, y), 1.0, fc="gold", ec="none", alpha=0.7))
    polys(ax, tray_m.slice(-2.0), color="tab:green", lw=0.6)
    for x, y in holes:
        ax.add_patch(Circle((x, y), 1.6, fill=False, ec="r"))
    ax.add_patch(Circle((0, 0), PCB_R, fill=False, ec="g", ls="--"))
    ax.add_patch(Circle((0, 0), 16.65, fill=False, ec="m", ls=":"))
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title("base z3 (k) z9.6 (orange), lid lip (blue), tray z-2 (green); pads gold, H1-H4 red")
    # vertical sections through the serial row (x = pin x) and through 0 deg (button)
    x_row = pins_serial[0][0]
    sections = [("section y=0 (row/H2 peg left, H4 peg right)", lambda m: m.rotate([-90, 0, 0]), False),
                (f"section x={x_row:.2f} through the serial row", lambda m: m.translate([-x_row, 0, 0]).rotate([0, 90, 0]), True)]
    for k, (title, xf, swap) in enumerate(sections):
        ax = fig.add_subplot(2, 3, 5 + k)
        for man, col in ((base_m, "k"), (lid_m, "tab:blue"), (tray_m, "tab:green")):
            for p in xf(man).slice(0.0).to_polygons():
                p = np.vstack([p, p[:1]])
                ax.plot(*((p[:, 1], p[:, 0]) if swap else (p[:, 0], p[:, 1])), color=col, lw=1)
        if swap:
            for _, y in pins_serial:
                ax.plot([y, y], [0, PCB_BOT], color="gold", lw=3)
        ax.axhspan(PCB_BOT, PCB_TOP, color="g", alpha=0.25)
        ax.axhline(PIN_HEIGHT, color="gold", ls="--", lw=0.8)
        ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_title(title + "  (dial green)")
    plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=80)


def write_plate(parts):
    """All print-oriented parts side by side on one plate -> Bambu Studio project 3MF (single filament)."""
    import sys
    sys.path[:0] = [str(HERE.parent / "_shared"),                 # 3Dprints master copy
                    str(HERE.parents[4] / "3Dprints" / "_shared")]  # copy in the watches repo
    import bambu3mf                                  # shared Bambu project writer (3Dprints/_shared)
    placed, x = [], 0.0
    for name, part in parts:
        m = to_manifold(part)
        lo, hi = np.array(m.bounding_box()[:3]), np.array(m.bounding_box()[3:])
        placed.append((name, m.translate([x - lo[0], -(lo[1] + hi[1]) / 2, -lo[2]]), 1))
        x += hi[0] - lo[0] + PLATE_GAP
    bambu3mf.write(OUT_PLATE, "watch_programmer_jig", placed, [PLATE_COLOUR])
    print(f"plate: {x - PLATE_GAP:.1f} mm wide, {len(parts)} parts -> {OUT_PLATE.name}")


if __name__ == "__main__":
    checks()
    base = base_part()
    lid = lid_part()
    tray = tray_part()
    base_m, lid_m, tray_m = to_manifold(base), to_manifold(lid), to_manifold(tray)
    report("tray", tray, tray_m)
    report("base", base, base_m)
    report("lid", lid, lid_m)
    # print orientation: tray floor down, base as modelled, lid flipped top face down
    lid_print = Pos(0, 0, PLATE_Z1) * (Rot(180, 0, 0) * lid)
    tray_print = Pos(0, 0, -TRAY_Z0) * tray
    for part, out in ((tray_print, OUT_TRAY), (base, OUT_BASE), (lid_print, OUT_LID)):
        export_stl(part, str(out.with_suffix(".stl")), tolerance=0.01, angular_tolerance=0.1)
        export_step(part, str(CAD / out.with_suffix(".step").name))
    bb = lid_print.bounding_box()
    print(f"lid print z {bb.min.Z:.3f}..{bb.max.Z:.3f}")
    write_plate([("jig_tray", tray_print), ("jig_base", base), ("jig_lid", lid_print)])
    preview(base_m, lid_m, tray_m)
    print("wrote", OUT_TRAY.name, OUT_BASE.name, OUT_LID.name, OUT_PNG.name)
