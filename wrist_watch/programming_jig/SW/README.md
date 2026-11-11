# Programming jig software

No firmware runs on the programmer board. It is a USB-C serial bridge (CP2102N) plus an ISP header (J13),
wired to pogo pins that touch pads on the back of the watch PCB:
- J1–J6: serial (RTS, TXD, RXD, 3V3, CTS, GND).
- J7–J12: ISP, through J13.

The firmware it uploads is in `../../watch/SW/`:
- `simpleTime/`: the watch firmware.
- `timeUpdate/`: a newer version that adds setting the hour and minute with the side button (long press).
  It was committed as "not tested".
- `libraries/`: RTClib, LowPower (LowPowerLab) and Adafruit BusIO. Copy them into the Arduino libraries folder.

The board settings are in the comment block at the top of `simpleTime.ino`:
- variant 328/328PA
- internal 1 MHz clock
- bootloader on UART0
- BOD 1.8 V
- EEPROM retained
- LTO disabled

## Upload
1. Put the watch PCB in the jig and lock the lid (`../HW/3Dprint/README.md`, "Use").
2. Upload over USB-C (serial bootloader), or use an ISP programmer on J13.
