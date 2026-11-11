# Programming jig (3D printed, 3 pieces)

These parts hold the watch PCB ("dial") on the programmer's pogo pins.

Folders: `print/` holds the print-ready STLs and the one-plate 3MF `jig_plate_V2.3mf` (already oriented), `cad/` the STEP files, `renders/` the PNGs.

| Part | File | What it does |
|---|---|---|
| Tray | `jig_tray_V1.stl` / `.step` | The programmer board sits in it. Two pegs come up through H2/H4 (9 and 3 o'clock). There are hex-nut pockets under H1/H3, a USB-C cut-out, and clearance under the board for through-hole pins. |
| Base | `jig_base_V2.stl` / `.step` | Sits on the pegs and is held by 2 screws from the top through H1/H3, which clamp base, board and tray together. Holds the 12 pogo pins upright, centres the Ø35.2 dial, and keys its rotation with a notch for the 3 o'clock side button. |
| Lid | `jig_lid_V2.stl` / `.step` | Twist-lock clamp ring. Turn it 28° clockwise and its lip presses the dial's rim down, compressing the pins 0.89 mm (64 % of stroke). The window leaves the LEDs visible. The lugs are unevenly spaced, so it only fits one way. |

The base has flats facing the J13 and J15 headers, and the lid's lug tabs never pass over them, so a
2x3 IDC cable plug on either header clears the jig.

Renders: `render_unlocked.png`, `render_locked.png`, `render_exploded.png` and `render_cutaway.png`
(a cut through both screws). `preview.png` shows the parts, a top-view overlay and sections.
`watch_programmer_jig.py` is the parametric source (build123d) and `render.py` makes the renders.
The master copy, with the 2-piece V1 in `archive/V1/`, lives in `Agents/3Dprints/watch_programmer_jig/` (private repo `yathAg/3Dprints`); the `watches` repo has a copy in `wrist_watch/programming_jig/HW/3Dprint/`, next to the programmer PCB in `../PCB/`.

## Hardware
- 2× M3 × 10 socket cap screws (DIN 912).
- 2× M3 hex nuts. The pockets are a snug 5.75 mm across flats, so the nuts press in from below and stay put.
- 6 or 12 Mill-Max 0907-2-15-20-75-14-11-0 pins on J1–J6 (serial: RTS TXD RXD 3V3 CTS GND) and/or
  J7–J12 (ISP, J13). Solder them to the pads, or just drop them into the base: the flange is trapped in a
  pocket under the base and the spring keeps it pressed on the pad.

## Assembly
1. Press the 2 nuts into the pockets on the underside of the tray.
2. Drop the programmer board into the tray, over the pegs.
3. Put the base on top of the pegs.
4. Fit the 2 M3 × 10 screws from the top, through the base ears into the nuts.
   The screw heads sit in counterbores below the dial.

## Printing
Use PLA, 0.2 mm layers and 3–4 walls, with **no supports**. The files are already oriented:
- **Tray:** floor down. The nut pockets open on the bed face with a 6 mm bridge over each.
- **Base:** programmer face down.
- **Lid:** top face down.

Expected weight is about 10.5 g for the tray, 7.4 g for the base and 1.7 g for the lid.

## Use
1. Drop the dial in LED side up, with the side button in the base notch. It only fits one way.
2. Line up the ticks on the lid tabs with the ticks under the base slots, then lower the lid.
3. Turn the lid **clockwise** 28° (the engraved arrow) until it clicks into the end pocket.
4. Upload over USB-C (CP2102N), or use ISP through J13. Turn it back anticlockwise to release.

## Dimensions worth checking
- The side button size is **estimated**: 6.0 mm wide, 1.5 mm proud of the edge, 3.5 mm tall
  (`BTN_*` in the script). Measure it and regenerate.
- Pin guide holes are Ø1.9 for the Ø1.5 barrel. If the pins bind, open them up with a 2.0 mm drill.
- The pegs are Ø3.0 for the 3.2 mm board holes. If they print fat, sand them lightly.
- The tray leaves 3.6 mm under the board for through-hole pins, which are assumed to stick out about
  3 mm. Trim longer header pins.
