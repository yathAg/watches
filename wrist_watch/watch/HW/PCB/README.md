# Watch PCB

`gerbers/` holds the watch PCB fabrication files (EAGLE CAM output: copper, mask, paste and silkscreen
layers, board profile, drills). They are extracted unchanged from
`../../../instructibles_archive/FLVZX1QJUSNBBNV.zip` (folder `gerber3/CAMOutputs/`), which stays as the
original download.

There is no editable board design (schematic or layout) in this repo, only the gerbers.

Board facts used by the case and the jig (measured from these gerbers):
- Round, Ø35.2 mm, 1.57 mm thick.
- Side switch at 3 o'clock, with THT pins on the underside. CR2032 holder underneath.
- Top-side pads reach r 16.65 mm.
- The back-side pads line up with both pogo-pin groups on the programmer (`../../../programming_jig/HW/PCB/`).
