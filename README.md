# TeensyLogicAnalyzer
Simple logic analyzer for 8-bit CPUs based on Jeff Tranter's 8-bit CPU logic analyzer

This is a work-in-progress.  You can find Jeff's work [here](https://github.com/jefftranter/6502).

This version aims to support a wide variety of 8-bit CPUs (so long as they fit in a 40-pin DIP)
and be easily expandable to new CPU types (e.g. 8080 and 8088) through the use of off-board adapter
modules, as well as to support sampling the full complement of control signals on the supported CPUs.

The main board is very similar to Jeff's original: it contains the Teensy 4.1, input buffers to
level-shift the 5V TTL signals from the target CPU down to the 3.3V levels supported by the Teensy,
a manual trigger button, and a ground reference pin (as a convenience).

The main difference is the use of a single keyed 40-pin IDC header.  This header's pinout does not
correspond directly to any single CPU type, but merely provides the input to the analyzer.
Individual adapter modules mate the analyzer to a specific CPU.  These adapter modules also contain
a 40-pin keyed IDC header, as well as 2 (optionally 3) rows of 1x20 pin headers for connecting a 40-pin
DIP clip to the adapter.  The adapter maps the target CPU's pin configuration to the analyzer's input
pins.  In addition to providing future expandability, it also keeps the manufacturing cost of the boards
low (some PCB fabricators charge as little as $5 + shipping for a run of 10 if the board is smaller
than 100mm x 100mm).  The analyzer board is approximately 91mm x 91mm (roughly 3.5 inches square), and
uses all through-hole components to make it easy to build.

In addition, I plan to make some specific enhancements to the 6809 support, including:
- Support additional signals on the 6809 (e.g. BS, BA, /FIRQ, /HALT).
- Properly notice instruction fetch on 6809E (by watching the LIC signal).
- Improve 6809 dissasembly (decode all of the addressing modes, as well as the 2-byte opcodes).

Beause the 6809E enhancements I want to make require additional signals to be routed, I needed
to make a new board anyway, which is why this project exists.

More to come.  Stay tuned!
