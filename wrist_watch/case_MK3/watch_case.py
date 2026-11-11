"""Binary wrist watch case MK III: a parametric redesign of Sverd Industries' MK II case.
It is screwless, the strap sits flush, it has a sprung crown, and it can take a metal case.
Two closure variants share every internal part:

  CASE A      One-piece case with a bezel lip. The crystal, dial ring, PCB and carrier all load from
              the BACK.
  CASEBACK A  Push-and-twist bayonet caseback. Its 3 lugs run in grooves cut into the case wall, below
              the inner floor, so the twist never passes the crown or switch. It has a coin slot.
              No screws, no glue: the caseback clamps the whole stack against the lip.
  CASE B      Closed-back body, like MK II.
  BEZEL B     Twist-lock bezel ring, using the programmer-jig lid pattern: a ramp, then a detent
              pocket. It carries the crystal, glued as in MK II. Its lugs run above the PCB, and the
              switch is below the PCB.
  CARRIER     Printed PCB carrier, which is also the insulating liner for a metal case. It has:
              - the MK II ledge footprint (proven component keep-out);
              - 4 inclined leaf springs that push the PCB up (take up stack tolerance and make the
                twist-lock click);
              - an anti-rotation key;
              - the crown return spring: a tangential flexure tongue whose pad presses the
                crown-stem head outward.
              3 copies, with tongue 0.5 / 0.6 / 0.7 mm.
  DIAL RING   Chapter ring between the PCB rim and the crystal in A. 12 ticks.
  CROWN       Fluted printed cap that self-threads onto the stem. The stem is an M2 flat-head
              (DIN 965) screw put in from inside the case; its head is the plunger and, seated in a
              countersink, the outward stop. Caps for M2x5 and M2x6.
  CRYSTAL     Dummy disc for fit tests. The real part is a 38 x 1.5 flat mineral crystal.

Sources
  MK II STLs   instructibles_prod/Watch_Body_-_MK_II.stl and Watch_Ring_-_MK_II.stl, measured 2026-11-11:
               - case D42.0, bore D38.6, lug gap 20.0, lug-to-lug 51.0;
               - spring-bar holes D1.0, drilled through, at y +-24.07, 2.86 above the lug bottom;
               - floor 1.2;
               - PCB ledge r 15.5-17.7, 4.8 above the floor. Its footprint is reused below.
               - gap from PCB top to crystal 2.5; crystal recess D38.1 x 1.5;
               - button window 7.9 x 3.9, centred 2.55 above the floor at 3 o'clock.
  Watch PCB    gerbers (instructibles_prod/FLVZX1QJUSNBBNV.zip):
               - D35.2 x 1.57;
               - side switch, THT pins at (14.3, +1.14 / -1.40), on the underside;
               - CR2032 holder underneath;
               - top-side pads reach r 16.65.
  Original     Hackaday.io 159919: D38 x 1.5 flat mineral crystal, CR2032, 20 mm spring bars.
  ESTIMATED    Switch body and actuator (SW_*): measure and regenerate.
               Strap end thickness 3.5-4 mm (user); one value is tried at each end.

Frame: origin on the case axis. +x = 3 o'clock (crown), +y = 12 o'clock, z up (crystal side).
z = 0 is the inner floor: the caseback inner face in A, the floor top in B. Modelled assembled and locked.

Print (exported files are already oriented, min Z = 0). 0.12 mm layers, 3-4 walls, NO supports.
  case_A        back face down. Lug bottoms are flat for FDM (LUG_DROP = 0).
  caseback_A    inner face down. The lugs are flush with it.
  case_B        back down.
  bezel_B       top face down.
  carrier_*     bottom down. The leaf springs are at 45 deg. PETG preferred (flexures).
  dial_ring     bottom down.
  crown_*       end face down.
  crystal_dummy flat.
Hardware: 1x M2x5 flat-head screw (DIN 965) as the crown stem; or file a freed M2x6 ring screw down
to 5 mm; or use the M2x6 with crown_M2x6. Also 2x 20 mm spring bars and a 38 x 1.5 crystal.

    python watch_case.py           ->  *_V1.stl/.step, render_A.png, render_B.png, render_parts.png and one
                                       Bambu plate per lab order (PLATES): watch_case_black_V1.3mf (PLA black),
                                       watch_case_silver_V1.3mf (PLA silver accents), watch_case_carriers_V1.3mf
                                       (PETG black)
    python watch_case.py --plates  ->  only re-plate from the exported STLs
"""
import json
import struct
import sys
import zipfile
from math import cos, degrees, radians, sin, sqrt
from pathlib import Path

import numpy as np
from build123d import (Axis, Compound, Plane, Polyline, Pos, Rot, export_step, export_stl, extrude,
                       make_face, mirror, revolve)
from manifold3d import CrossSection, FillRule, JoinType, Manifold, Mesh, OpType

HERE = Path(__file__).parent
SEG = 128
VER = "V1"

# --- process (sets fits and printability rules) ---
PROCESS = "fdm"                                    # "fdm" | "sla" | "cnc" | "slm"
PROFILES = {   # per-side clearances (mm); lug_drop = lug tips below the back (needs non-FDM)
    "fdm": dict(slide=0.2, crystal=0.15, lug_drop=0.0, min_wall=1.0),
    "sla": dict(slide=0.1, crystal=0.08, lug_drop=1.6, min_wall=0.8),
    "cnc": dict(slide=0.05, crystal=0.03, lug_drop=1.6, min_wall=0.8),
    "slm": dict(slide=0.3, crystal=0.15, lug_drop=1.6, min_wall=1.5)}
PRO = PROFILES[PROCESS]
SLIDE = PRO["slide"]

# --- watch PCB + side switch ---
PCB_R, PCB_T = 17.6, 1.57
PCB_BOT = 4.8                                      # MK II: ledge top is 4.8 above the floor
PCB_TOP = PCB_BOT + PCB_T                          # 6.37
TOP_PADS_R = 16.65                                 # top-side pads reach this radius (gerber)
Z_CROWN = 2.55                                     # crown / switch axis = MK II button-window centre
SW_W, SW_Z0 = 6.0, 1.3                             # ESTIMATED switch body width, bottom (hangs from PCB)
SW_X0 = 13.3                                       # body back face (THT pins at x 14.3)
SW_FRONT_R = 17.9                                  # ESTIMATED: body front ~flush with the PCB edge
SW_ACT_D = 1.6                                     # ESTIMATED actuator stub diameter
SW_PRETRAVEL = 0.2                                 # head face -> actuator tip at rest
SW_TRAVEL = 0.25                                   # typical tact switch travel

# --- crystal + stack above the PCB ---
CRYSTAL_D, CRYSTAL_T = 38.0, 1.5
GAP_ABOVE_PCB = 2.5                                # MK II (proven); shrink after measuring top parts
CRYSTAL_BOT = PCB_TOP + GAP_ABOVE_PCB              # 8.87 (A)
CRYSTAL_TOP = CRYSTAL_BOT + CRYSTAL_T              # 10.37 (A)

# --- case (both variants) ---
CASE_R = 21.0                                      # MK II
BORE_R = CRYSTAL_D / 2 + PRO["crystal"]            # 19.15: crystal + everything loads through it
GROOVE_R = BORE_R + 0.75                           # bayonet grooves / key slot root (wall left 1.1)
CH = 0.5                                           # outer bottom chamfer

# --- lugs + flush strap shoulder ---
LUG_IN_X = 10.0                                    # 20 mm strap
LUG_TIP_Y, LUG_TIP_X = 25.5, 13.4                  # lug-to-lug 51 (MK II)
LUG_ROOT = (15.6, 14.0)                            # lug outer face runs into the case here
LUG_ROOT_TOP = 7.0
BAR_Y, BAR_Z = 24.07, 0.0                          # MK II bar position (2.86 above its lug bottom)
BAR_HOLE_D = 1.1                                   # drill 1.0 after printing (through-drilled lugs)
LUG_TIP_TOP = BAR_Z + BAR_HOLE_D / 2 + 1.25        # 1.8
LUG_DROP = PRO["lug_drop"]
TIP_ROUND = 0.8
STRAP_T = {12: 4.0, 6: 3.5}                        # leather end thickness tried at each end (user: 3.5-4)
STRAP_GAP = 0.3
SHOULDER_Y = {k: round(BAR_Y - t / 2 - STRAP_GAP, 2) for k, t in STRAP_T.items()}   # 21.77 / 22.02

# --- crown (M2 flat-head screw stem, DIN 965) ---
SCREW_HEAD_D, SCREW_HEAD_K, SCREW_D = 3.8, 1.2, 2.0
CSK_D = 3.9                                        # countersink mouth at the bore
HEAD_TOP_R = BORE_R + (CSK_D - SCREW_HEAD_D) / 2   # 19.2: flat head face (points inward), seated
STEM_HOLE_D = 2.3
BOSS_D, BOSS_PROUD = 4.0, 0.4
BOSS_END_R = CASE_R + BOSS_PROUD                   # 21.4
TRAVEL = 0.5                                       # cap -> boss end at rest = hard stop (set by threading)
CAP_D, CAP_SKIRT_H, CAP_END_T = 5.4, 0.4, 0.7
CAP_SKIRT_ID = BOSS_D + 2 * SLIDE
CAP_R0 = BOSS_END_R + TRAVEL                       # cap inner face at rest (21.9)
PILOT_D = 1.6                                      # M2 self-taps into it
CAP_FLUTES, FLUTE_D = 18, 0.8
SCREW_LENGTHS = (5.0, 6.0)

# --- carrier (PCB holder / liner / springs) ---
SLEEVE_IN = PCB_R + 0.2                            # 17.8: centres the PCB
SLEEVE_OUT = BORE_R - SLIDE                        # 18.95
SLEEVE_TOP = 5.9                                   # 0.47 below the dial ring / bezel lip (push travel)
SEAT_Z = 4.45                                      # rigid seat; leaves hold the PCB 0.35 above it locked
LEAF_FREE = 0.7                                    # leaf tip above the seat, free
LEAF_TIP_Z = SEAT_Z + LEAF_FREE                    # 5.15
LEAF_T, LEAF_W, LEAF_R, LEAF_ROOT_Z = 0.6, 1.2, 16.55, 1.0
LEAF_ANGLES_TRY = (50.0, 135.0, 180.0, 225.0)   # lower right is full of parts (MK II ledge is a 0.6 rim)
LEDGE_R0 = 15.4
TONGUE_TS = (0.5, 0.6, 0.7)                        # crown return spring (tangential cantilever)
TONGUE_Z = (0.0, 4.5)                              # from the floor: prints as a wall (flexes sideways)
WINDOW_A = (-26.0, 9.0)                            # sleeve window (deg); tongue root at WINDOW_A[0]
TONGUE_FREE_A = 7.0
PAD_D, PAD_HOLE_D = 3.4, 2.4
PAD_FACE_R = HEAD_TOP_R + 0.05                     # 0.05 preload on the screw head
KEY_ANG, KEY_W = 180.0, 2.0                        # anti-rotation key at 9 o'clock
KEY_R_OUT = GROOVE_R - 0.25

# --- dial ring (A) ---
DIAL_IN, DIAL_OUT = 16.9, BORE_R - SLIDE           # rests on the PCB rim (clear of pads at r 16.65)
TICK_R, TICK_W, TICK_D = (17.05, 18.0), 0.6, 0.4

# --- variant A: back-loading case + push-and-twist caseback ---
A_BOT = -1.4
LIP_T = 0.7
A_TOP = CRYSTAL_TOP + LIP_T                        # 11.07
LIP_IN_R, LIP_SEAT_R = 18.2, 18.5                  # aperture D36.4; crystal sits on r 18.5-19.0
CB_R = BORE_R - SLIDE                              # caseback plate 18.95
A_LOCK = (55.0, 125.0, 235.0)                      # lug centres when locked (uneven: fits one way)
A_TWIST = 28.0                                     # clockwise seen from the back (= +theta here)
LUG_ARC = 4.5                                      # lug length along the circumference (mm)
A_LUG_Z = (-0.7, 0.0)                              # caseback lugs, flush with its inner face
A_POCKET = A_LUG_Z[0]                              # pocket floor (locked)
A_PLATEAU = A_POCKET + 0.2                         # path floor: push 0.2 to leave the pocket
A_PUSH = 0.3
A_CEIL = A_LUG_Z[1] + A_PUSH + 0.15
COIN_SLOT = (12.0, 1.4, 0.6)                       # length, width, depth

# --- variant B: closed body + twist-lock bezel ---
B_BOT = -1.2
B_TOP = 8.5
FLANGE_GAP = 0.35                                  # > detent, so the bezel can dip at the click
B_FU = B_TOP + FLANGE_GAP                          # bezel flange underside 8.85
B_SEAT = B_FU + 0.6                                # crystal seat 9.45
B_BEZ_TOP = B_SEAT + CRYSTAL_T                     # 10.95, crystal flush
B_REC_R = CRYSTAL_D / 2 + 0.1                      # glued crystal recess
BEZ_SKIRT_IN, BEZ_LIP_T = 18.2, 0.6
B_LOCK = (55.0, 125.0, 235.0)
B_TWIST = 28.0                                     # clockwise seen from the top (= -theta here)
B_LUG_Z = (6.6, 7.3)                               # skirt lugs, locked (top face bears on the roof)
B_DETENT, B_LOCK_DROP = 0.2, 0.45
B_FLOOR_G = B_LUG_Z[0] - B_DETENT - 0.15
BEZ_FLUTES, BEZ_FLUTE_D = 72, 0.6

OUT = {k: HERE / f"{k}_{VER}" for k in
       ("case_A", "caseback_A", "case_B", "bezel_B", "dial_ring", "crystal_dummy",
        "crown_M2x5", "crown_M2x6", "carrier_t05", "carrier_t06", "carrier_t07")}
PLATES = {   # one lab order each (one material + colour per order). "silver" = the visible accent parts,
             # which swap onto the black cases for the black + silver look.
    "black": ("#1A1A1C", "PLA, black", ["case_A", "caseback_A", "case_B", "bezel_B", "dial_ring",
                                        "crown_M2x5", "crown_M2x6"]),
    "silver": ("#B8B9BD", "PLA, silver", ["bezel_B", "dial_ring", "crown_M2x5", "crown_M2x6"]),
    "carriers": ("#1A1A1C", "PETG, black", ["carrier_t05", "carrier_t06", "carrier_t07"])}
LAYER_H = "0.12"
WALLS = "3"                                       # thin mechanical parts: 3 perimeters


# ---------- helpers ----------
def ang(mm, r):
    return degrees(mm / r)


def face_from_xy(pts):
    return make_face(Polyline(*[(x, y, 0) for x, y in pts], close=True))


def prism(pts, z0, z1):
    return Pos(0, 0, (z0 + z1) / 2) * extrude(face_from_xy(pts), (z1 - z0) / 2, both=True)


def prism_yz(pts, x0, x1):
    """(y, z) outline extruded along x from x0 to x1."""
    face = make_face(Polyline(*[(0, y, z) for y, z in pts], close=True))
    return Pos((x0 + x1) / 2, 0, 0) * extrude(face, (x1 - x0) / 2, both=True)


def sector(r0, r1, a0, a1, z0, z1):
    """Annular sector, angles in degrees (a0 < a1)."""
    n = max(3, int(abs(a1 - a0) / 1.5) + 2)
    a = np.radians(np.linspace(a0, a1, n))
    pts = [(r1 * cos(t), r1 * sin(t)) for t in a] + [(r0 * cos(t), r0 * sin(t)) for t in a[::-1]]
    return prism(pts, z0, z1)


def disc(x, y, d, z0, z1, n=48):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return prism([(x + d / 2 * cos(a), y + d / 2 * sin(a)) for a in t], z0, z1)


def revolve_rz(profile):
    return revolve(make_face(Polyline(*[(r, 0, z) for r, z in profile], close=True)), Axis.Z)


def revolve_x(profile, z=0.0):
    """Solid of revolution about the x axis from an (x, rho) outline, at height z."""
    return Pos(0, 0, z) * revolve(make_face(Polyline(*[(x, rho, 0) for x, rho in profile], close=True)),
                                  Axis.X)


def rod_x(d, x0, x1, y=0.0, z=0.0):
    return Pos(0, y, 0) * revolve_x([(x0, 0), (x1, 0), (x1, d / 2), (x0, d / 2)], z)


def cs_to_parts(cs, z0, z1):
    """manifold CrossSection -> list of build123d prisms (outer loop minus holes per component)."""
    out = []
    for comp in cs.decompose():
        polys = sorted(comp.to_polygons(), key=lambda p: -ncs([p]).area())
        solid = prism([tuple(p) for p in polys[0]], z0, z1)
        for hole in polys[1:]:
            solid = solid - prism([tuple(p) for p in hole], z0 - 1, z1 + 1)
        out.append(solid)
    return out


def ncs(polys):
    """CrossSection from loops of any winding."""
    return CrossSection([[tuple(p) for p in poly] for poly in polys], FillRule.NonZero)


def annulus_cs(r0, r1):
    return CrossSection.circle(r1, 360) - CrossSection.circle(r0, 360)


def valid(part):
    iv = part.is_valid
    return iv() if callable(iv) else iv


def to_manifold(part, tol=0.02):
    """B-rep -> watertight Manifold (tessellation duplicates seam vertices: weld them)."""
    v, t = part.tessellate(tol, 0.2)
    v = np.array([[p.X, p.Y, p.Z] for p in v])
    uniq, inv = np.unique(np.round(v, 5), axis=0, return_inverse=True)
    tri = inv.reshape(-1)[np.asarray(t)]
    tri = tri[(tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])]
    man = Manifold(Mesh(vert_properties=uniq.astype(np.float32), tri_verts=tri.astype(np.uint32)))
    assert man.status().name == "NoError", man.status()
    return man


def load_stl(path, offset):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    v = np.frombuffer(b[84:84 + n * 50], dtype=dt)["v"].reshape(-1, 3).astype(float) + offset
    uniq, inv = np.unique(np.round(v, 4), axis=0, return_inverse=True)
    tri = inv.reshape(-1, 3)
    tri = tri[(tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])]
    return Manifold(Mesh(vert_properties=uniq.astype(np.float32), tri_verts=tri.astype(np.uint32)))


def mk2_file(name):
    for c in (HERE.parent / "instructibles_prod" / name,
              HERE.parent.parent / "binary_watch" / "binary_watch" / "wrist_watch" / "instructibles_prod" / name):
        if c.exists():
            return c
    raise FileNotFoundError(name)


MK2_BODY = load_stl(mk2_file("Watch_Body_-_MK_II.stl"), np.array([-100.0, -125.5, -2.8]))
LUG_HW = ang(LUG_ARC, BORE_R) / 2
SLOT_HW = LUG_HW + ang(0.35, BORE_R)
POCKET_HW = LUG_HW + ang(0.2, BORE_R)


# ---------- shared geometry ----------
def mk2_ledge_cs():
    """MK II PCB-ledge footprint (material present at every height of the cavity), grown 0.25 so it
    fuses with the sleeve, clipped to the carrier band."""
    acc = None
    for z in (0.3, 1.0, 2.0, 3.0, 4.0, 4.6):
        s = MK2_BODY.slice(z) ^ annulus_cs(15.2, 17.85)
        acc = s if acc is None else acc ^ s
    acc = acc.offset(0.25, JoinType.Round, circular_segments=SEG).simplify(0.01)
    return acc ^ annulus_cs(LEDGE_R0, SLEEVE_IN + 0.15)


LEDGE_CS = mk2_ledge_cs()


def leaf_local(phi):
    """Leaf-spring footprints at angle phi: (pocket corner pts, leaf (s, z) outline, frame)."""
    u = np.array([cos(radians(phi)), sin(radians(phi))])
    t = np.array([-sin(radians(phi)), cos(radians(phi))])
    run = LEAF_TIP_Z - LEAF_ROOT_Z
    w = LEAF_T * sqrt(2)                                     # horizontal thickness of a 45 deg leaf
    s0 = -(run + w) / 2
    pocket_s = (s0 - 0.4, s0 + run + w + 0.7)                # room for the tip to move forward
    pocket_r = (LEDGE_R0 - 1.0, LEAF_R + LEAF_W / 2 + 0.25)   # open toward the centre (notch)
    pk = [tuple(rr * u + ss * t) for rr, ss in ((pocket_r[0], pocket_s[0]), (pocket_r[1], pocket_s[0]),
                                                (pocket_r[1], pocket_s[1]), (pocket_r[0], pocket_s[1]))]
    leaf = [(s0, LEAF_ROOT_Z - 0.05), (s0 + w, LEAF_ROOT_Z - 0.05), (s0 + w + run, LEAF_TIP_Z),
            (s0 + run, LEAF_TIP_Z)]
    return pk, leaf


def leaf_angles():
    """Pick 4 leaf positions whose pocket (+0.5 margin) lies inside the MK II ledge footprint."""
    out = []
    for phi0 in LEAF_ANGLES_TRY:
        for d in (0, 4, -4, 8, -8, 12, -12, 16, -16):
            phi = phi0 + d
            u = np.array([cos(radians(phi)), sin(radians(phi))])
            t = np.array([-sin(radians(phi)), cos(radians(phi))])
            pk, _ = leaf_local(phi)
            ss = [float(np.dot(np.array(q), t)) for q in pk]
            foot = [tuple(rr * u + sv * t) for rr, sv in ((LEDGE_R0 + 0.2, min(ss) - 0.6), (17.45, min(ss) - 0.6),
                                                          (17.45, max(ss) + 0.6), (LEDGE_R0 + 0.2, max(ss) + 0.6))]
            if (ncs([foot]) - LEDGE_CS).area() < 0.05:
                out.append(phi0 + d)
                break
        else:
            raise RuntimeError(f"no room for a leaf near {phi0} deg")
    return out


LEAF_ANGLES = leaf_angles()


def carrier_part(tongue_t):
    body = revolve_rz([(SLEEVE_IN + 0.3, 0), (SLEEVE_OUT - 0.3, 0), (SLEEVE_OUT, 0.3), (SLEEVE_OUT, SLEEVE_TOP),
                       (SLEEVE_IN + 0.3, SLEEVE_TOP), (SLEEVE_IN, SLEEVE_TOP - 0.3), (SLEEVE_IN, 0.3)])
    body = body + Compound(cs_to_parts(LEDGE_CS, 0, SEAT_Z))
    body = body + sector(SLEEVE_OUT - 0.3, KEY_R_OUT, KEY_ANG - ang(KEY_W / 2, SLEEVE_OUT),
                         KEY_ANG + ang(KEY_W / 2, SLEEVE_OUT), 0, SLEEVE_TOP)
    cuts = [sector(SLEEVE_IN - 0.6, SLEEVE_OUT + 0.6, WINDOW_A[0], WINDOW_A[1], -1, PCB_BOT),   # window
            sector(PCB_R - 0.05, SLEEVE_IN + 0.2, WINDOW_A[0] - 2, WINDOW_A[1] + 2, -1, PCB_BOT),  # ledge
            prism([(SW_X0 - 0.4, -SW_W / 2 - 0.4), (SW_FRONT_R + 0.25, -SW_W / 2 - 0.4),         # switch
                   (SW_FRONT_R + 0.25, SW_W / 2 + 0.4), (SW_X0 - 0.4, SW_W / 2 + 0.4)], -1, PCB_BOT)]
    for phi in LEAF_ANGLES:
        pk, _ = leaf_local(phi)
        cuts.append(prism(pk, LEAF_ROOT_Z, SEAT_Z + 1))
    body = body - Compound(cuts)
    # leaf springs (45 deg planks in the pockets)
    leaves = []
    for phi in LEAF_ANGLES:
        _, leaf = leaf_local(phi)
        leaves.append(Rot(0, 0, phi) * (Pos(LEAF_R, 0, 0) * prism_yz(leaf, -LEAF_W / 2, LEAF_W / 2)))
    body = body + Compound(leaves)
    # crown return tongue + pad (presses the screw head outward), hole for the actuator
    body = body + sector(SLEEVE_OUT - tongue_t, SLEEVE_OUT, WINDOW_A[0] - 2.0, TONGUE_FREE_A, *TONGUE_Z)
    body = body + rod_x(PAD_D, SLEEVE_OUT - tongue_t + 0.05, PAD_FACE_R, z=Z_CROWN)
    body = body - rod_x(PAD_HOLE_D, SLEEVE_IN - 1, PAD_FACE_R + 1, z=Z_CROWN)
    solids = sorted(body.solids(), key=lambda x: -x.volume)      # drop MK II ledge slivers cut loose
    if len(solids) > 1:                                           # by the switch notch / window
        print(f"  carrier t{tongue_t}: dropped {len(solids) - 1} loose sliver(s), "
              f"{sum(x.volume for x in solids[1:]):.2f} mm3")
    return solids[0]


def dial_ring_part():
    body = revolve_rz([(DIAL_IN, PCB_TOP), (DIAL_OUT, PCB_TOP), (DIAL_OUT, CRYSTAL_BOT),
                       (DIAL_IN + 0.4, CRYSTAL_BOT), (DIAL_IN, CRYSTAL_BOT - 0.4)])
    ticks = []
    for k in range(12):
        a = 90 - 30 * k
        hw = ang(TICK_W * (1.6 if k == 0 else 1.0) / 2, sum(TICK_R) / 2)
        ticks.append(sector(TICK_R[0], TICK_R[1], a - hw, a + hw, CRYSTAL_BOT - TICK_D, CRYSTAL_BOT + 1))
    return body - Compound(ticks)


def crown_part(screw_len):
    """Crown cap in its own frame: z = 0 at the inner face, +z outward."""
    engage = HEAD_TOP_R + screw_len - CAP_R0
    pilot = engage + 0.3
    length = pilot + CAP_END_T
    r, ri = CAP_D / 2, CAP_SKIRT_ID / 2
    body = revolve_rz([(ri, -CAP_SKIRT_H), (r - 0.2, -CAP_SKIRT_H), (r, -CAP_SKIRT_H + 0.2), (r, length - 0.5),
                       (r - 0.5, length), (0, length), (0, 0), (ri, 0)])
    fl = [disc((r + 0.1) * cos(2 * np.pi * k / CAP_FLUTES), (r + 0.1) * sin(2 * np.pi * k / CAP_FLUTES),
               FLUTE_D, -CAP_SKIRT_H - 1, length - 0.6, 24) for k in range(CAP_FLUTES)]
    return body - Compound(fl) - disc(0, 0, PILOT_D, -1, pilot, 32), length, engage


def crown_assembled(cap, travel=0.0):
    """Cap frame -> case frame (axis along +x at Z_CROWN), pushed in by `travel`."""
    return Pos(CAP_R0 - travel, 0, Z_CROWN) * (Rot(0, 90, 0) * cap)


def screw_part(length, travel=0.0):
    x0 = HEAD_TOP_R - travel
    cone_end = x0 + (SCREW_HEAD_D - SCREW_D) / 2
    return revolve_x([(x0, 0), (x0, SCREW_HEAD_D / 2), (cone_end, SCREW_D / 2), (x0 + length, SCREW_D / 2),
                      (x0 + length, 0)], Z_CROWN)


def end_block(sign, z_bot):
    """Lugs + flush strap shoulder at the 12 (sign +1) or 6 (sign -1) o'clock end."""
    sy = SHOULDER_Y[12 if sign > 0 else 6]
    lug = [(LUG_IN_X, 12.0), (LUG_IN_X, LUG_TIP_Y), (LUG_TIP_X, LUG_TIP_Y), LUG_ROOT, (LUG_ROOT[0], 12.0)]
    cs = (ncs([lug]) + ncs([[(-x, y) for x, y in lug]]) +
          ncs([[(-LUG_IN_X - 0.5, 12.0), (LUG_IN_X + 0.5, 12.0), (LUG_IN_X + 0.5, sy),
                         (-LUG_IN_X - 0.5, sy)]]))
    cs = cs.offset(-TIP_ROUND, JoinType.Round, circular_segments=48).offset(TIP_ROUND, JoinType.Round,
                                                                            circular_segments=48)
    plan = Compound(cs_to_parts(cs, z_bot - LUG_DROP - 2, 20))
    # side profile (y, z)
    pts = [(12.0, z_bot)]
    if LUG_DROP > 0:
        pts += [(y, z_bot - LUG_DROP * ((y - 19.5) / (LUG_TIP_Y - 19.5)) ** 2)
                for y in np.linspace(19.5, LUG_TIP_Y, 12)]
    else:
        pts.append((LUG_TIP_Y, z_bot))
    pts.append((LUG_TIP_Y, LUG_TIP_TOP - TIP_ROUND))
    for a in np.linspace(0, 90, 7)[1:]:
        pts.append((LUG_TIP_Y - TIP_ROUND + TIP_ROUND * cos(radians(a)),
                    LUG_TIP_TOP - TIP_ROUND + TIP_ROUND * sin(radians(a))))
    y0, y1 = 17.0, LUG_TIP_Y - TIP_ROUND
    for s in np.linspace(1, 0, 16)[1:]:
        pts.append((y0 + s * (y1 - y0), LUG_ROOT_TOP - (LUG_ROOT_TOP - LUG_TIP_TOP) * s ** 1.4))
    pts.append((12.0, LUG_ROOT_TOP))
    side = prism_yz(pts, -LUG_ROOT[0] - 2, LUG_ROOT[0] + 2)
    block = plan & side
    holes = Compound([rod_x(BAR_HOLE_D, x0, x1, BAR_Y, BAR_Z) for x0, x1 in ((LUG_IN_X - 0.5, LUG_ROOT[0] + 1),
                                                                            (-LUG_ROOT[0] - 1, -LUG_IN_X + 0.5))])
    block = block - holes
    return block if sign > 0 else mirror(block, Plane.XZ)


def crown_boss():
    return rod_x(BOSS_D, CASE_R - 1.2, BOSS_END_R, z=Z_CROWN)   # only 0.4 proud: no teardrop needed


def crown_cuts():
    csk = revolve_x([(BORE_R - 1.0, 0), (BORE_R - 1.0, CSK_D / 2 + 1.0), (BORE_R + CSK_D / 2, 0)], Z_CROWN)
    return [csk, rod_x(STEM_HOLE_D, BORE_R - 1, BOSS_END_R + 1, z=Z_CROWN)]


def key_slot(z0, z1):
    hw = ang(KEY_W / 2 + SLIDE, BORE_R)
    return sector(BORE_R - 0.2, GROOVE_R, KEY_ANG - hw, KEY_ANG + hw, z0, z1)


# ---------- variant A ----------
def case_a_part():
    body = revolve_rz([(BORE_R + 0.3, A_BOT), (CASE_R - CH, A_BOT), (CASE_R, A_BOT + CH), (CASE_R, A_TOP - 0.6),
                       (CASE_R - 0.6, A_TOP), (LIP_IN_R + 0.2, A_TOP), (LIP_IN_R, A_TOP - 0.2),
                       (LIP_IN_R, CRYSTAL_TOP + 0.3), (LIP_SEAT_R, CRYSTAL_TOP), (BORE_R, CRYSTAL_TOP),
                       (BORE_R, A_BOT + 0.3)])
    body = body + end_block(1, A_BOT) + end_block(-1, A_BOT) + crown_boss()
    cuts = [revolve_rz([(0, A_BOT - 1), (BORE_R, A_BOT - 1), (BORE_R, CRYSTAL_TOP - 0.01), (0, CRYSTAL_TOP - 0.01)]),
            key_slot(A_BOT - 1, SLEEVE_TOP + 0.3)] + crown_cuts()
    for lock in A_LOCK:
        e = lock - A_TWIST
        cuts += [sector(BORE_R - 0.2, GROOVE_R, e - SLOT_HW, e + SLOT_HW, A_BOT - 1, A_CEIL),       # entry slot
                 sector(BORE_R - 0.2, GROOVE_R, e - SLOT_HW, lock + POCKET_HW, A_PLATEAU, A_CEIL),  # path
                 sector(BORE_R - 0.2, GROOVE_R, lock - POCKET_HW, lock + POCKET_HW, A_POCKET, A_CEIL),
                 sector(CASE_R - 1.0, CASE_R - 0.25, e - ang(0.3, 20.4), e + ang(0.3, 20.4),       # tick
                        A_BOT - 1, A_BOT + 0.3)]
    return body - Compound(cuts)


def caseback_a_part():
    body = revolve_rz([(0, A_BOT), (CB_R - 0.4, A_BOT), (CB_R, A_BOT + 0.4), (CB_R, 0), (0, 0)])
    body = body + Compound([sector(CB_R - 0.3, GROOVE_R - 0.25, a - LUG_HW, a + LUG_HW, *A_LUG_Z) for a in A_LOCK])
    lx, lw, ld = COIN_SLOT
    slot = [(lx / 2 - lw / 2 + lw / 2 * cos(t), lw / 2 * sin(t)) for t in np.linspace(-np.pi / 2, np.pi / 2, 12)]
    slot += [(-lx / 2 + lw / 2 + lw / 2 * cos(t), lw / 2 * sin(t)) for t in np.linspace(np.pi / 2, 3 * np.pi / 2, 12)]
    cuts = [prism(slot, A_BOT - 1, A_BOT + ld)]
    cuts += [sector(CB_R - 2.0, CB_R - 0.8, a - ang(0.3, 17.5), a + ang(0.3, 17.5), A_BOT - 1, A_BOT + 0.3)
             for a in A_LOCK]                             # ticks at the lugs (align with the case ticks)
    return body - Compound(cuts)


# ---------- variant B ----------
def b_roof(theta, lock):
    """Groove roof z at angle theta for the lug locking at `lock` (lug moves from lock+TWIST down to lock)."""
    pocket_hi = lock + POCKET_HW
    ramp_hi = lock + B_TWIST - SLOT_HW
    if theta <= pocket_hi:
        return B_LUG_Z[1]
    f = min(1.0, (theta - pocket_hi) / (ramp_hi - pocket_hi))
    return B_LUG_Z[1] - B_DETENT + f * (B_LOCK_DROP + B_DETENT)


def case_b_part():
    body = revolve_rz([(0, B_BOT), (CASE_R - CH, B_BOT), (CASE_R, B_BOT + CH), (CASE_R, B_TOP - 0.4),
                       (CASE_R - 0.4, B_TOP), (BORE_R + 0.3, B_TOP), (BORE_R, B_TOP - 0.3), (BORE_R, 0), (0, 0)])
    body = body + end_block(1, B_BOT) + end_block(-1, B_BOT) + crown_boss()
    cuts = [revolve_rz([(0, 0), (BORE_R, 0), (BORE_R, B_TOP + 1), (0, B_TOP + 1)]),
            key_slot(0, B_TOP + 1)] + crown_cuts()
    for lock in B_LOCK:
        e = lock + B_TWIST
        cuts.append(sector(BORE_R - 0.2, GROOVE_R, e - SLOT_HW, e + SLOT_HW, B_FLOOR_G, B_TOP + 1))   # entry
        edges = np.arange(lock - POCKET_HW, e - SLOT_HW, 1.0).tolist() + [e - SLOT_HW + 0.01]
        for a0, a1 in zip(edges[:-1], edges[1:]):
            cuts.append(sector(BORE_R - 0.2, GROOVE_R, a0, a1 + 0.01, B_FLOOR_G, b_roof(a0, lock)))
        cuts.append(sector(CASE_R - 0.3, CASE_R + 1, e - ang(0.3, CASE_R), e + ang(0.3, CASE_R),     # tick
                           B_TOP - 1.2, B_TOP + 1))
    return body - Compound(cuts)


def bezel_b_part():
    body = revolve_rz([(DIAL_IN, PCB_TOP), (BORE_R - SLIDE, PCB_TOP), (BORE_R - SLIDE, B_FU), (CASE_R - 0.4, B_FU),
                       (CASE_R, B_FU + 0.4), (CASE_R, B_BEZ_TOP - 0.5), (CASE_R - 0.5, B_BEZ_TOP),
                       (B_REC_R + 0.25, B_BEZ_TOP), (B_REC_R, B_BEZ_TOP - 0.25), (B_REC_R, B_SEAT),
                       (BEZ_SKIRT_IN, B_SEAT), (BEZ_SKIRT_IN, PCB_TOP + BEZ_LIP_T),
                       (DIAL_IN + 0.4, PCB_TOP + BEZ_LIP_T), (DIAL_IN, PCB_TOP + BEZ_LIP_T - 0.4)])
    body = body + Compound([sector(BORE_R - SLIDE - 0.2, GROOVE_R - 0.25, a - LUG_HW, a + LUG_HW, *B_LUG_Z)
                            for a in B_LOCK])
    fl = [disc((CASE_R + 0.1) * cos(2 * np.pi * k / BEZ_FLUTES), (CASE_R + 0.1) * sin(2 * np.pi * k / BEZ_FLUTES),
               BEZ_FLUTE_D, B_FU + 0.5, B_BEZ_TOP - 0.6, 16) for k in range(BEZ_FLUTES)]
    dots = [disc(20.0 * cos(radians(a)), 20.0 * sin(radians(a)), 0.8, B_BEZ_TOP - 0.3, B_BEZ_TOP + 1, 24)
            for a in B_LOCK]
    for k in range(12):                                    # chapter ticks on the inner lip
        a = 90 - 30 * k
        hw = ang(TICK_W * (1.6 if k == 0 else 1.0) / 2, 17.5)
        dots.append(sector(DIAL_IN + 0.3, BEZ_SKIRT_IN - 0.1, a - hw, a + hw, PCB_TOP + BEZ_LIP_T - TICK_D,
                           PCB_TOP + BEZ_LIP_T + 1))
    return body - Compound(fl + dots)


def crystal_part():
    return revolve_rz([(0, 0), (CRYSTAL_D / 2 - SLIDE / 2, 0), (CRYSTAL_D / 2 - SLIDE / 2, CRYSTAL_T), (0, CRYSTAL_T)])


# ---------- context (renders / checks only) ----------
def pcb_context():
    pcb = revolve_rz([(0, PCB_BOT), (PCB_R, PCB_BOT), (PCB_R, PCB_TOP), (0, PCB_TOP)])
    sw = prism([(SW_X0, -SW_W / 2), (SW_FRONT_R, -SW_W / 2), (SW_FRONT_R, SW_W / 2), (SW_X0, SW_W / 2)],
               SW_Z0, PCB_BOT)
    act = rod_x(SW_ACT_D, SW_FRONT_R - 0.1, HEAD_TOP_R - SW_PRETRAVEL, z=Z_CROWN)
    return pcb, sw + act


def strap_context(z_center=BAR_Z):
    out = []
    for sign, key in ((1, 12), (-1, 6)):
        t = STRAP_T[key]
        pts = [(BAR_Y + t / 2 * cos(a), z_center + t / 2 * sin(a)) for a in np.linspace(np.pi / 2, 3 * np.pi / 2, 16)]
        pts += [(BAR_Y + 18, z_center - t / 2), (BAR_Y + 18, z_center + t / 2)]
        s = prism_yz(pts, -LUG_IN_X + 0.15, LUG_IN_X - 0.15)
        out.append(s if sign > 0 else mirror(s, Plane.XZ))
    return out


# ---------- checks ----------
def report(name, part, man):
    bb = part.bounding_box()
    vol = part.volume / 1000
    print(f"  {name:14s} valid={valid(part)} genus={man.genus():3d} vol={vol:5.2f} cm3 (~{vol * 1.24 * 0.85:4.1f} g PLA)"
          f"  size {bb.size.X:5.2f} x {bb.size.Y:5.2f} x {bb.size.Z:5.2f}")


def overlap(a, b):
    return (a ^ b).volume()


def static_checks():
    assert CASE_R - GROOVE_R >= PRO["min_wall"], f"wall behind the grooves {CASE_R - GROOVE_R:.2f} < min"
    assert DIAL_IN > TOP_PADS_R + 0.2, "dial ring / bezel lip lands on top-side pads"
    assert SLEEVE_TOP + A_PUSH + 0.1 < PCB_TOP, "sleeve hits the dial ring when the caseback is pushed"
    assert LEAF_FREE - (PCB_BOT - SEAT_Z) >= 0.3, "leaf preload too small"
    assert A_PLATEAU - A_POCKET >= 0.15 and A_LUG_Z[0] + (PCB_BOT - SEAT_Z) > A_PLATEAU, \
        "caseback cannot be pushed high enough to leave the pocket"
    assert PCB_BOT - SEAT_Z >= B_DETENT + 0.1, "PCB bottoms on the seat at the B detent"
    assert LEAF_TIP_Z - PCB_BOT <= B_LOCK_DROP - 0.05, "B entry: lugs hit the roof before the slot"
    assert B_TOP - (B_LUG_Z[1] + B_LOCK_DROP) >= 0.7, "wall above the B groove too thin"
    assert B_FLOOR_G > SLEEVE_TOP, "B groove floor below the sleeve top"
    assert Z_CROWN - SCREW_HEAD_D / 2 > A_CEIL + 0.1, "screw head reaches the A bayonet zone"
    assert Z_CROWN + SCREW_HEAD_D / 2 < PCB_BOT - 0.2, "screw head hits the PCB"
    head_in = HEAD_TOP_R - TRAVEL
    act_tip = HEAD_TOP_R - SW_PRETRAVEL
    assert act_tip - head_in >= SW_TRAVEL, "crown travel does not actuate the switch"
    for k, y in SHOULDER_Y.items():
        print(f"  strap {k:2d} o'clock: shoulder y {y:.2f}, gap to a {STRAP_T[k]} mm strap end {BAR_Y - STRAP_T[k] / 2 - y:.2f}")
    print(f"  crown: travel {TRAVEL}, switch pressed {act_tip - head_in:.2f} (travel {SW_TRAVEL}); "
          f"leaf preload {LEAF_FREE - (PCB_BOT - SEAT_Z):.2f}, PCB above seat {PCB_BOT - SEAT_Z:.2f}")
    print(f"  leaf angles {LEAF_ANGLES}; lug {2 * LUG_HW:.1f} deg, slot {2 * SLOT_HW:.1f}, twist {A_TWIST}")
    print(f"  heights: A {A_TOP - A_BOT:.2f} (z {A_BOT}..{A_TOP:.2f}), B {B_BEZ_TOP - B_BOT:.2f} "
          f"(z {B_BOT}..{B_BEZ_TOP:.2f}); MK II body 11.6")


def assembly_checks(M):
    """M: dict of manifolds in assembled pose. Prints overlaps; asserts the ones that must be zero."""
    def check(name, parts, expected):
        names = list(parts)
        print(f"  {name}:")
        bad = []
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                v = overlap(parts[a], parts[b])
                if v > 0.02:
                    tag = "expected" if {a, b} in expected else "CLASH"
                    print(f"    {a:9s} x {b:9s} {v:7.3f} mm3  {tag}")
                    if tag == "CLASH":
                        bad.append((a, b))
        assert not bad, f"clashes in {name}: {bad}"

    exp = [{"carrier", "pcb"}, {"carrier", "screw"}, {"crown", "screw"}]
    check("A assembled", {k: M[k] for k in ("case_A", "caseback", "carrier", "dial", "crystal", "pcb", "switch",
                                            "screw", "crown")}, exp)
    check("B assembled", {k: M[k] for k in ("case_B", "bezel", "carrier", "crystal_B", "pcb", "switch", "screw",
                                            "crown")}, exp)
    # A twist: lugs lifted onto the plateau, swept entry -> lock; and the slot from below
    lug = M["cb_lugs"]
    sweep = []
    for lock_off in np.arange(-A_TWIST, 0.01, 2.0):
        sweep.append(lug.rotate([0, 0, lock_off]).translate([0, 0, A_PLATEAU - A_POCKET + 0.05]))
    for dz in np.arange(-3.0, 0.3, 0.5):
        sweep.append(lug.rotate([0, 0, -A_TWIST]).translate([0, 0, dz]))
    sw_a = Manifold.batch_boolean(sweep, OpType.Add)
    v_case = overlap(sw_a, M["case_A"])
    full = Manifold.batch_boolean([M["caseback"].rotate([0, 0, a]).translate([0, 0, A_PUSH])
                                   for a in np.arange(-A_TWIST, 0.01, 4.0)], OpType.Add)
    v_crown = overlap(full, M["screw"] + M["switch"] + M["crown"])
    print(f"  A twist: lug path vs case {v_case:.3f} mm3, whole caseback sweep vs crown/screw/switch {v_crown:.3f}")
    assert v_case < 0.02 and v_crown < 0.02
    # B twist: lug follows the roof (top just under it) from the slot to the pocket
    sweep = []
    for k, lock in enumerate(B_LOCK):
        lug_b = M["bz_lugs"][k]
        for th in np.arange(lock, lock + B_TWIST + 0.01, 1.0):
            span = np.linspace(th - LUG_HW, th + LUG_HW, 9)
            dz = min(b_roof(a, lock) for a in span) - B_LUG_Z[1] - 0.08
            sweep.append(lug_b.rotate([0, 0, th - lock]).translate([0, 0, dz]))
        for dz in np.arange(0.0, 3.0, 0.5):
            sweep.append(lug_b.rotate([0, 0, B_TWIST]).translate([0, 0, B_LOCK_DROP + dz]))
    sw_b = Manifold.batch_boolean(sweep, OpType.Add)
    full_b = Manifold.batch_boolean([M["bezel"].rotate([0, 0, a]).translate([0, 0, -B_DETENT])
                                     for a in np.arange(0, B_TWIST + 0.01, 4.0)], OpType.Add)
    v_case, v_crown = overlap(sw_b, M["case_B"]), overlap(full_b, M["screw"] + M["switch"] + M["carrier"])
    print(f"  B twist: lug path vs body {v_case:.3f} mm3, whole bezel sweep vs crown/switch/carrier {v_crown:.3f}")
    assert v_case < 0.02 and v_crown < 0.02
    # crown pressed by TRAVEL: cap must not hit the case flank (boss end contact only)
    v = overlap(M["crown_in"], M["case_A"]), overlap(M["crown_in"], M["case_B"])
    print(f"  crown pressed {TRAVEL}: cap vs case A {v[0]:.3f}, vs body B {v[1]:.3f} mm3")
    assert max(v) < 0.05


# ---------- export / plates ----------
def export(part, out):
    export_stl(part, str(out.with_suffix(".stl")), tolerance=0.01, angular_tolerance=0.1)
    export_step(part, str(out.with_suffix(".step")))
    bb = part.bounding_box()
    assert abs(bb.min.Z) < 1e-3, f"{out.name} min Z {bb.min.Z}"


def write_plate(path, name, parts, colour, width=190.0, gap=6.0):
    """Shelf-pack print-oriented manifolds [(name, Manifold)] into rows <= width, write a Bambu project 3MF
    (one filament, template is PLA: the technician applies the ordered material's profile), then set the
    layer height and wall count."""
    sys.path.insert(0, str(HERE.parent / "tambola"))
    try:
        import functools
        import bambu3mf                                  # shared Bambu project writer (3Dprints/tambola)
        if not hasattr(bambu3mf._filament_keys, "cache_info"):    # it reads ~1900 profile JSONs per call
            bambu3mf._filament_keys = functools.lru_cache(None)(bambu3mf._filament_keys)
    except ImportError:
        print("  bambu3mf not found: plate skipped")
        return
    ms = list(parts)
    ms.sort(key=lambda t: -(t[1].bounding_box()[4] - t[1].bounding_box()[1]))
    placed, x, y, row_h = [], 0.0, 0.0, 0.0
    for n, m in ms:
        lo, hi = np.array(m.bounding_box()[:3]), np.array(m.bounding_box()[3:])
        w, h = hi[0] - lo[0], hi[1] - lo[1]
        if x > 0 and x + w > width:
            x, y, row_h = 0.0, y + row_h + gap, 0.0
        placed.append((n, m.translate([x - lo[0], y - lo[1], -lo[2]]), 1))
        x += w + gap
        row_h = max(row_h, h)
    bambu3mf.write(path, name, placed, [colour])
    z = zipfile.ZipFile(path)
    items = {i.filename: z.read(i.filename) for i in z.infolist()}
    z.close()
    ps = json.loads(items["Metadata/project_settings.config"])
    ps["layer_height"] = LAYER_H
    ps["wall_loops"] = WALLS
    items["Metadata/project_settings.config"] = json.dumps(ps, indent=4).encode()
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zo:
        for k, v in items.items():
            zo.writestr(k, v)
    print(f"  plate {path.name}: {len(parts)} parts, {x:.0f} x {y + row_h:.0f} mm, layer {LAYER_H}, {WALLS} walls")


def make_plates():
    """One Bambu project per lab order, built from the exported (print-oriented) STLs."""
    for key, (colour, label, names) in PLATES.items():
        parts = [(n, load_stl(OUT[n].with_suffix(".stl"), np.zeros(3))) for n in names]
        write_plate(HERE / f"watch_case_{key}_{VER}.3mf", f"watch_case_{key}", parts, colour)
        print(f"    {key}: {label}: {', '.join(names)}")


# ---------- renders ----------
def renders(M, prints):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    col = {"case_A": (0.20, 0.20, 0.23), "case_B": (0.20, 0.20, 0.23), "caseback": (0.45, 0.45, 0.50),
           "bezel": (0.45, 0.45, 0.50), "carrier": (0.95, 0.55, 0.15), "dial": (0.85, 0.75, 0.45),
           "crystal": (0.55, 0.75, 0.95), "crystal_B": (0.55, 0.75, 0.95), "pcb": (0.10, 0.42, 0.22),
           "switch": (0.85, 0.15, 0.15), "screw": (0.70, 0.70, 0.75), "crown": (0.45, 0.45, 0.50),
           "strap": (0.45, 0.28, 0.15), "mk2": (0.6, 0.1, 0.6)}

    def add_mesh(ax, man, c, dz=0.0, alpha=1.0):
        m = man.to_mesh()
        v = np.asarray(m.vert_properties)[:, :3] + [0, 0, dz]
        tri = v[np.asarray(m.tri_verts)]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.35 + 0.65 * np.clip(n @ np.array([0.3, -0.5, 0.8]), 0, 1)
        fc = np.clip(np.outer(shade, c), 0, 1)
        ax.add_collection3d(Poly3DCollection(tri, facecolors=np.c_[fc, np.full(len(fc), alpha)], edgecolors="none"))

    def section(ax, items, xf, swap=False, lw=1.0):
        for man, c in items:
            for p in xf(man).slice(0.0).to_polygons():
                p = np.vstack([p, p[:1]])
                ax.fill(*((p[:, 1], p[:, 0]) if swap else (p[:, 0], p[:, 1])), color=c, alpha=0.35, lw=0)
                ax.plot(*((p[:, 1], p[:, 0]) if swap else (p[:, 0], p[:, 1])), color=c, lw=lw)

    y0 = lambda m: m.rotate([-90, 0, 0])                                # y = 0 plane -> (x, z)
    at = lambda a: (lambda m: m.rotate([0, 0, -a]).rotate([-90, 0, 0]))  # radial plane at angle a -> (r, z)
    x0 = lambda m: m.rotate([0, 90, 0])                                 # x = 0 plane -> (z, y), swap

    for var in ("A", "B"):
        case = M["case_" + var]
        lid = M["caseback"] if var == "A" else M["bezel"]
        cry = M["crystal"] if var == "A" else M["crystal_B"]
        stack = [(case, "case_" + var), (lid, "caseback" if var == "A" else "bezel"), (M["carrier"], "carrier"),
                 (M["pcb"], "pcb"), (M["switch"], "switch"), (cry, "crystal"), (M["screw"], "screw"),
                 (M["crown"], "crown")] + ([(M["dial"], "dial")] if var == "A" else [])
        fig = plt.figure(figsize=(21, 13))
        # exploded
        ax = fig.add_subplot(2, 3, 1, projection="3d")
        off = {"A": {"case_A": 0, "caseback": -14, "carrier": -8, "pcb": -2, "switch": -2, "dial": 10,
                     "crystal": 16, "screw": 0, "crown": 0},
               "B": {"case_B": 0, "bezel": 16, "carrier": 0, "pcb": 4, "switch": 4, "crystal": 24, "screw": 0,
                     "crown": 0}}[var]
        for man, k in stack:
            add_mesh(ax, man, col[k], off.get(k, 0))
        ax.set_xlim(-26, 26); ax.set_ylim(-26, 26); ax.set_zlim(-22, 30); ax.set_box_aspect((1, 1, 1))
        ax.view_init(22, -55); ax.set_axis_off(); ax.set_title(f"variant {var}: exploded")
        # assembled, from below / above with straps
        ax = fig.add_subplot(2, 3, 2, projection="3d")
        for man, k in stack[:2] + [(M["crown"], "crown")]:
            add_mesh(ax, man, col[k])
        for s in M["straps"]:
            add_mesh(ax, s, col["strap"], alpha=0.9)
        ax.set_xlim(-28, 28); ax.set_ylim(-36, 36); ax.set_zlim(-20, 20); ax.set_box_aspect((1, 1.3, 0.7))
        ax.view_init(28, -35); ax.set_axis_off(); ax.set_title(f"variant {var}: assembled, 20 mm leather straps")
        # crown section
        ax = fig.add_subplot(2, 3, 3)
        section(ax, [(m, col[k]) for m, k in stack], y0)
        ax.set_xlim(13, 27); ax.set_ylim(-2, 12 if var == "A" else 12); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title("section y=0: crown, screw stem, flexure tongue + pad, switch (red, ESTIMATED)")
        # bayonet section at the first lock angle
        ax = fig.add_subplot(2, 3, 4)
        a = (A_LOCK if var == "A" else B_LOCK)[0]
        section(ax, [(m, col[k]) for m, k in stack], at(a))
        ax.set_xlim(13, 23); ax.set_ylim(-2, 12); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title(f"section at {a:.0f} deg (bayonet lug, locked)")
        # side section x=0 with MK II overlay
        ax = fig.add_subplot(2, 3, 5)
        section(ax, [(m, col[k]) for m, k in stack] + [(s, col["strap"]) for s in M["straps"]], x0, swap=True)
        for p in x0(M["mk2"]).slice(0.0).to_polygons():
            p = np.vstack([p, p[:1]])
            ax.plot(p[:, 1], p[:, 0], color=col["mk2"], lw=0.8, ls="--")
        ax.set_xlim(-30, 30); ax.set_ylim(-5, 13); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title("section x=0 (12 o'clock right), MK II body dashed; straps flush with the shoulder")
        # top view
        ax = fig.add_subplot(2, 3, 6)
        for man, c, z in ((case, "k", 1.0), (M["mk2"], col["mk2"], 3.0)):
            for p in man.slice(z).to_polygons():
                p = np.vstack([p, p[:1]])
                ax.plot(p[:, 0], p[:, 1], color=c, lw=0.8, ls="--" if c != "k" else "-")
        for p in M["carrier"].slice(3.0).to_polygons():
            p = np.vstack([p, p[:1]])
            ax.fill(p[:, 0], p[:, 1], color=col["carrier"], alpha=0.5)
        for s in M["straps"]:
            for p in s.slice(BAR_Z).to_polygons():
                p = np.vstack([p, p[:1]])
                ax.fill(p[:, 0], p[:, 1], color=col["strap"], alpha=0.6)
        ax.set_xlim(-28, 30); ax.set_ylim(-40, 40); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title("top: case z=1 (black), MK II z=3 (dashed), carrier z=3 (orange), straps")
        plt.tight_layout()
        plt.savefig(HERE / f"render_{var}.png", dpi=75)
        plt.close(fig)
    # print-oriented parts
    fig = plt.figure(figsize=(20, 11))
    for i, (n, part) in enumerate(prints):
        ax = fig.add_subplot(3, 5, i + 1, projection="3d")
        m = to_manifold(part)
        add_mesh(ax, m, (0.75, 0.75, 0.78))
        lo, hi = np.array(m.bounding_box()[:3]), np.array(m.bounding_box()[3:])
        c, r = (lo + hi) / 2, max(hi - lo) / 2 + 1
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(0, 2 * r)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(30, -60); ax.set_axis_off(); ax.set_title(n + " (as printed)")
    plt.tight_layout()
    plt.savefig(HERE / "render_parts.png", dpi=70)
    plt.close(fig)


if __name__ == "__main__":
    if "--plates" in sys.argv:                            # re-plate from the exported STLs only
        make_plates()
        sys.exit()
    static_checks()
    print("building parts ...")
    caseA, cbA = case_a_part(), caseback_a_part()
    caseB, bezB = case_b_part(), bezel_b_part()
    carriers = {t: carrier_part(t) for t in TONGUE_TS}
    dial, cry = dial_ring_part(), crystal_part()
    crowns = {L: crown_part(L) for L in SCREW_LENGTHS}
    print("checks:")
    M = {"case_A": to_manifold(caseA), "caseback": to_manifold(cbA), "case_B": to_manifold(caseB),
         "bezel": to_manifold(bezB), "carrier": to_manifold(carriers[0.6]), "dial": to_manifold(dial),
         "crystal": to_manifold(Pos(0, 0, CRYSTAL_BOT) * cry), "crystal_B": to_manifold(Pos(0, 0, B_SEAT) * cry)}
    pcb, sw = pcb_context()
    M["pcb"], M["switch"] = to_manifold(pcb), to_manifold(sw)
    M["screw"] = to_manifold(screw_part(SCREW_LENGTHS[0]))
    M["crown"] = to_manifold(crown_assembled(crowns[SCREW_LENGTHS[0]][0]))
    M["crown_in"] = to_manifold(crown_assembled(crowns[SCREW_LENGTHS[0]][0], TRAVEL))
    M["cb_lugs"] = to_manifold(Compound([sector(CB_R - 0.3, GROOVE_R - 0.25, a - LUG_HW, a + LUG_HW, *A_LUG_Z)
                                         for a in A_LOCK]))
    M["bz_lugs"] = [to_manifold(sector(BORE_R - SLIDE - 0.2, GROOVE_R - 0.25, a - LUG_HW, a + LUG_HW, *B_LUG_Z))
                    for a in B_LOCK]
    M["straps"] = [to_manifold(s) for s in strap_context()]
    M["mk2"] = MK2_BODY
    for n, p, m in (("case_A", caseA, M["case_A"]), ("caseback_A", cbA, M["caseback"]), ("case_B", caseB, M["case_B"]),
                    ("bezel_B", bezB, M["bezel"]), ("dial_ring", dial, M["dial"]), ("crystal_dummy", cry, M["crystal"])):
        report(n, p, m)
    for t, p in carriers.items():
        report(f"carrier_t{t}", p, to_manifold(p))
    for L, (p, length, eng) in crowns.items():
        report(f"crown_M2x{L:.0f}", p, to_manifold(p))
        print(f"    cap length {length:.2f}, thread engagement {eng:.2f}, protrudes "
              f"{CAP_R0 + length - CASE_R:.2f} from the flank")
    assembly_checks(M)
    # print orientation
    prints = [("case_A", Pos(0, 0, -A_BOT) * caseA),
              ("caseback_A", Rot(180, 0, 0) * cbA),
              ("case_B", Pos(0, 0, -B_BOT) * caseB),
              ("bezel_B", Pos(0, 0, B_BEZ_TOP) * (Rot(180, 0, 0) * bezB)),
              ("dial_ring", Pos(0, 0, -PCB_TOP) * dial),
              ("crystal_dummy", cry)]
    for L, (p, length, _) in crowns.items():
        prints.append((f"crown_M2x{L:.0f}", Pos(0, 0, length) * (Rot(180, 0, 0) * p)))
    carrier_prints = [(f"carrier_t{int(round(t * 10)):02d}", p) for t, p in carriers.items()]
    for n, p in prints + carrier_prints:
        export(p, OUT[n])
    print("exported", ", ".join(n for n, _ in prints + carrier_prints))
    make_plates()
    renders(M, prints + carrier_prints)
    print("wrote render_A.png, render_B.png, render_parts.png")
