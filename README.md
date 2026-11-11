# watches

Binary watch projects: a wrist watch (ATmega328 + DS3231 RTC, 4×4 LED binary display), its programming
jig, and a table clock.

```
wrist_watch/
  watch/
    HW/PCB/          watch PCB gerbers (from the original Instructables files)
    HW/3Dprint/      case MK III: watch_case.py, print/ (STL + Bambu plates), cad/ (STEP), renders/
    SW/              firmware: simpleTime/, timeUpdate/, libraries/ (RTClib, LowPower, Adafruit BusIO)
  programming_jig/
    HW/PCB/          programmer board (KiCad: CP2102N USB-serial + ISP, pogo pins), production/ for fab
    HW/3Dprint/      3-piece jig that holds the watch PCB on the pogo pins
    SW/              notes: no firmware runs on the programmer
  instructibles_archive/   original Instructables download: MK II case STLs + watch PCB gerbers (zip)
table_clock/         table_clock.ino
```

The `HW/3Dprint/` folders are copies. The masters, with older versions in `archive/`, live in the
3Dprints workspace (`Agents/3Dprints/watch_case/` and `Agents/3Dprints/watch_programmer_jig/`).
