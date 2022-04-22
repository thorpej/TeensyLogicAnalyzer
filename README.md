# TeensyLogicAnalyzer
Simple logic analyzer for 8-bit CPUs based on Jeff Tranter's 8-bit CPU logic analyzer

This is a work-in-progress.  You can find Jeff's work [here](https://github.com/jefftranter/6502).
Once this version is more complete, I will document more about it here.

I plan to make the following changes:
- Support additional signals on the 6809 (e.g. BS, BA, /FIRQ, /HALT).
- Properly notice instruction fetch on 6809E (by watching the LIC signal).
- Improve 6809 dissasembly (decode all of the addressing modes, as well as the 2-byte opcodes).
- Use 40-pin keyed IDC connectors that map 1-1 to the 40-pin 8-bit CPUs the analyzer supports.  A daughter board with pin headers and an IDC connector will interface the DIP clip to the analyzer.

Beause the 6809E enhancements I want to make require additional signals to be routed, I needed
to make a new board anyway, which is why this project exists.

More to come.  Stay tuned!
