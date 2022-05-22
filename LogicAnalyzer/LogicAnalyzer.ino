/*

  Teensy Logic Analyzer
  Logic Analyzer for 6502, 6800, 6809, or Z80 microprocessors based on a
  Teensy 4.1 microcontroller.

  See https://github.com/thorpej/TeensyLogicAnalyzer

  Based on https://github.com/jefftranter/6502/tree/master/LogicAnalyzer

  Copyright (c) 2022 by Jason R. Thorpe <thorpej@me.com>
  Copyright (c) 2021-2022 by Jeff Tranter <tranter@pobox.com>

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

#include <SD.h>

#include "tla.h"
#include "insn_decode.h"

// Maximum buffer size (in samples). Increase if needed; should be
// able to go up to at least 30,000 before running out of memory.
#define BUFFSIZE 5000

const char *versionString = "Teensy Logic Analyzer version 0.4";
const char *verboseVersionStringAdditions = " by Jason R. Thorpe <thorpej@me.com>";
const char *origVersionString = "Based on Logic Analyzer version 0.30 by Jeff Tranter <tranter@pobox.com>";

#define cpu_has_iospace(c)  ((c) == cpu_z80)

// Global variables
uint32_t control[BUFFSIZE];           // Recorded control line data
uint32_t address[BUFFSIZE];           // Recorded address data
uint32_t data[BUFFSIZE];              // Recorded data lines
uint32_t triggerAddress = 0;          // Address to trigger on
uint32_t triggerData = 0;             // Data to trigger on
uint32_t aTriggerBits;                // GPIO bit pattern to trigger address on
uint32_t aTriggerMask;                // bitmask of GPIO address bits
uint32_t cTriggerBits;                // GPIO bit pattern to trigger control on
uint32_t cTriggerMask;                // bitmask of GPIO control bits
uint32_t dTriggerBits;                // GPIO bit pattern to trigger data on
uint32_t dTriggerMask;                // bitmask of GPIO data bits
int samples = 20;                     // Total number of samples to record (up to BUFFSIZE)
int pretrigger = 0;                   // Number of samples to record before trigger (up to samples)
int triggerPoint = 0;                 // Sample in buffer corresponding to trigger point
int samplesTaken = 0;                 // Number of samples taken
trigger_t triggerMode = tr_none;      // Type of trigger
cycle_t triggerCycle = tr_either;     // Trigger on read, write, or either
space_t triggerSpace = tr_mem;        // default to memory space
bool triggerLevel = false;            // Trigger level (false=low, true=high);
volatile bool triggerPressed = false; // Set by hardware trigger button

extern "C" {
  cpu_t cpu = cpu_none;                 // Current CPU type
}

extern "C" {
  __attribute__((__format__(__printf__, 1, 2)))
  int
  tla_printf(const char *fmt, ...)
  {
    static char tla_printf_buf[256];
    va_list ap;
    int rv;

    va_start(ap, fmt);
    rv = vsnprintf(tla_printf_buf, sizeof(tla_printf_buf), fmt, ap);
    va_end(ap);

    Serial.print(tla_printf_buf);

    return rv;
  }
}

void
show_version(bool verbose)
{
  if (verbose) {
    tla_printf("%s%s\n", versionString, verboseVersionStringAdditions);
    tla_printf("%s\n", origVersionString);
  } else {
    tla_printf("%s\n", versionString);
  }
}

//
// MASTER TABLE OF Teensy 4.1 DIGITAL I/O PINS
// In general, these inputs are grouped so e.g. CPU data and address lines can all be read
// from a single 32-bit GPIO port register.
//
//  0 (1.3)  - CA0
//  1 (1.2)  - CA1
//  2 (4.4)  - CC0
//  3 (4.5)  - CC1
//  4 (4.6)  - CC2
//  5 (4.8)  - CC3
//  6 (2.10) - CD0
//  7 (2.17) - CD1
//  8 (2.16) - CD2
//  9 (2.11) - CD3
// 10 (2.0)  - CD4
// 11 (2.2)  - CD5
// 12 (2.1)  - CD6
// 13 -- on-board LED
// 14 (1.18) - CA2
// 15 (1.19) - CA3
// 16 (1.23) - CA4
// 17 (1.22) - CA5
// 18 (1.17) - CA6
// 19 (1.16) - CA7
// 20 (1.26) - CA8
// 21 (1.27) - CA9
// 22 (1.24) - CA10
// 23 (1.25) - CA11
// 24 (1.12) - CA12
// 25 (1.13) - CA13
// 26 (1.30) - CA14
// 27 (1.31) - CA15
// 28 (3.18) -- data bus enable
// 29 (4.31) - CC4
// 30 (3.23) - n/c
// 31 (3.22) -- trigger button
// 32 (2.12) - CD7
// 33 (4.7)  - CC5
// 34 (2.29) - CC7
// 35 (2.28) - CC8
// 36 (2.18) - CC9
// 37 (2.19) - CC10
// 38 (1.28) - CC6
// 39 (1.29) - CC11
// 40 (1.20) - CC12
// 41 (1.21) - CC13
//
// Note that we snoop a total of 38 signals:
// - 16 address signals
// - 8 data signals
// - 14 control signals
//
// This is exactly enough in order to fully probe any 40-pin 8-bit CPU
// (2 pins will always be taken up by Vcc and GND).

#define ENABLE_PIN        28
#define BUTTON_PIN        31

#define CAxx_PSR          CORE_PIN0_PINREG   // All CAxx lines are in the same GPIO port
#define CA0_PIN_BITMASK   CORE_PIN0_BITMASK
#define CA1_PIN_BITMASK   CORE_PIN1_BITMASK
#define CA2_PIN_BITMASK   CORE_PIN14_BITMASK
#define CA3_PIN_BITMASK   CORE_PIN15_BITMASK
#define CA4_PIN_BITMASK   CORE_PIN16_BITMASK
#define CA5_PIN_BITMASK   CORE_PIN17_BITMASK
#define CA6_PIN_BITMASK   CORE_PIN18_BITMASK
#define CA7_PIN_BITMASK   CORE_PIN19_BITMASK
#define CA8_PIN_BITMASK   CORE_PIN20_BITMASK
#define CA9_PIN_BITMASK   CORE_PIN21_BITMASK
#define CA10_PIN_BITMASK  CORE_PIN22_BITMASK
#define CA11_PIN_BITMASK  CORE_PIN23_BITMASK
#define CA12_PIN_BITMASK  CORE_PIN24_BITMASK
#define CA13_PIN_BITMASK  CORE_PIN25_BITMASK
#define CA14_PIN_BITMASK  CORE_PIN26_BITMASK
#define CA15_PIN_BITMASK  CORE_PIN27_BITMASK

#define CDxx_PSR          CORE_PIN6_PINREG    // All CDxx lines are in the same GPIO port  
#define CD0_PIN_BITMASK   CORE_PIN6_BITMASK
#define CD1_PIN_BITMASK   CORE_PIN7_BITMASK
#define CD2_PIN_BITMASK   CORE_PIN8_BITMASK
#define CD3_PIN_BITMASK   CORE_PIN9_BITMASK
#define CD4_PIN_BITMASK   CORE_PIN10_BITMASK
#define CD5_PIN_BITMASK   CORE_PIN11_BITMASK
#define CD6_PIN_BITMASK   CORE_PIN12_BITMASK
#define CD7_PIN_BITMASK   CORE_PIN32_BITMASK

// Control signals are grouped into physical inputs (CCxx_PIN_BITMASK),
// and their logical equivalents (CCxx_BITMASK).  The logical equivalants,
// once normalized re then grouped into their per-CPU meanings (CC_xxx_yyy).

#define CCxx_PSR          CORE_PIN2_PINREG
#define CC0_PIN_BITMASK   CORE_PIN2_BITMASK
#define CC1_PIN_BITMASK   CORE_PIN3_BITMASK
#define CC2_PIN_BITMASK   CORE_PIN4_BITMASK
#define CC3_PIN_BITMASK   CORE_PIN5_BITMASK
#define CC4_PIN_BITMASK   CORE_PIN29_BITMASK
#define CC5_PIN_BITMASK   CORE_PIN33_BITMASK
#define CC6_PIN_BITMASK   CORE_PIN38_BITMASK  // in CAxx_PSR
#define CC7_PIN_BITMASK   CORE_PIN34_BITMASK  // in CDxx_PSR
#define CC8_PIN_BITMASK   CORE_PIN35_BITMASK  // in CDxx_PSR
#define CC9_PIN_BITMASK   CORE_PIN36_BITMASK  // in CDxx_PSR
#define CC10_PIN_BITMASK  CORE_PIN37_BITMASK  // in CDxx_PSR
#define CC11_PIN_BITMASK  CORE_PIN39_BITMASK  // in CAxx_PSR
#define CC12_PIN_BITMASK  CORE_PIN40_BITMASK  // in CAxx_PSR
#define CC13_PIN_BITMASK  CORE_PIN41_BITMASK  // in CAxx_PSR

// The control signals in CDxx_PSR need to be read at the same time as the
// other control signals, which means we need to read the CDxx_PSR twice
// and properly mix the bits from the each read into the final result.
#define CDxx_PSR_CC_MASK  (CC7_PIN_BITMASK | CC8_PIN_BITMASK | CC9_PIN_BITMASK | CC10_PIN_BITMASK)
#define CDxx_PSR_CD_MASK  (~CDxx_PSR_CC_MASK)

// We need to poll certain CC pins, so define constants for
// them here.
#define CC0_PIN           2
#define CC1_PIN           3
#define CC2_PIN           4
#define CC3_PIN           5
#define CC4_PIN           29
#define CC5_PIN           33
#define CC6_PIN           38
#define CC7_PIN           34
#define CC8_PIN           35
#define CC9_PIN           36
#define CC10_PIN          37
#define CC11_PIN          39
#define CC12_PIN          40
#define CC13_PIN          41

#define CC0_BITMASK       (1U <<  0)
#define CC1_BITMASK       (1U <<  1)
#define CC2_BITMASK       (1U <<  2)
#define CC3_BITMASK       (1U <<  3)
#define CC4_BITMASK       (1U <<  4)
#define CC5_BITMASK       (1U <<  5)
#define CC6_BITMASK       (1U <<  6)
#define CC7_BITMASK       (1U <<  7)
#define CC8_BITMASK       (1U <<  8)
#define CC9_BITMASK       (1U <<  9)
#define CC10_BITMASK      (1U << 10)
#define CC11_BITMASK      (1U << 11)
#define CC12_BITMASK      (1U << 12)
#define CC13_BITMASK      (1U << 13)

#define CC_6502_PHI2      CC0_BITMASK
#define CC_6502_SYNC      CC1_BITMASK
#define CC_6502_RW        CC2_BITMASK
#define CC_6502_RESET     CC3_BITMASK
#define CC_6502_IRQ       CC4_BITMASK
#define CC_6502_NMI       CC5_BITMASK
#define CC_6502_RDY       CC6_BITMASK
#define CC_6502_SO        CC7_BITMASK
#define CC_6502_PHI1      CC11_BITMASK

#define CC_6502_PHI1_PIN  CC11_PIN
#define CC_6502_PHI2_PIN  CC0_PIN

#define CC_6800_PHI2      CC0_BITMASK         // same as 6502
#define CC_6800_VMA       CC1_BITMASK
#define CC_6800_RW        CC2_BITMASK         // same as 6502
#define CC_6800_RESET     CC3_BITMASK         // same as 6502
#define CC_6800_IRQ       CC4_BITMASK         // same as 6502
#define CC_6800_NMI       CC5_BITMASK         // same as 6502
#define CC_6800_HALT      CC6_BITMASK
#define CC_6800_DBE       CC7_BITMASK
#define CC_6800_BA        CC8_BITMASK
#define CC_6800_TSC       CC10_BITMASK
#define CC_6800_PHI1      CC11_BITMASK

#define CC_6800_PHI1_PIN  CC11_PIN
#define CC_6800_PHI2_PIN  CC0_PIN

#define CC_6809_E         CC0_BITMASK
#define CC_6809_Q         CC1_BITMASK
#define CC_6809_RW        CC2_BITMASK         // same as 6502
#define CC_6809_RESET     CC3_BITMASK         // same as 6502
#define CC_6809_IRQ       CC4_BITMASK         // same as 6502
#define CC_6809_NMI       CC5_BITMASK         // same as 6502
#define CC_6809_FIRQ      CC6_BITMASK
#define CC_6809E_LIC      CC7_BITMASK
#define CC_6809_BA        CC8_BITMASK
#define CC_6809_BS        CC9_BITMASK
#define CC_6809E_TSC      CC10_BITMASK
#define CC_6809_MRDY      CC11_BITMASK        // NOT 6809E
#define CC_6809E_AVMA     CC11_BITMASK
#define CC_6809_DMA_BREQ  CC12_BITMASK        // NOT 6809E
#define CC_6809E_BUSY     CC12_BITMASK
#define CC_6809_HALT      CC13_BITMASK

#define CC_6809_E_PIN     CC0_PIN
#define CC_6809_Q_PIN     CC1_PIN

#define CC_Z80_CLK        CC0_BITMASK
#define CC_Z80_M1         CC1_BITMASK
#define CC_Z80_MREQ       CC2_BITMASK
#define CC_Z80_IORQ       CC3_BITMASK
#define CC_Z80_RD         CC4_BITMASK
#define CC_Z80_WR         CC5_BITMASK
#define CC_Z80_RESET      CC6_BITMASK
#define CC_Z80_INT        CC7_BITMASK
#define CC_Z80_NMI        CC8_BITMASK
#define CC_Z80_BUSACK     CC9_BITMASK
#define CC_Z80_BUSRQ      CC10_BITMASK
#define CC_Z80_WAIT       CC11_BITMASK
#define CC_Z80_HALT       CC12_BITMASK
#define CC_Z80_RFSH       CC13_BITMASK

#define CC_Z80_CLK_PIN    CC0_PIN

//
// FAKE SAMPLE DATA FOR TEST AND DEBUGGING PURPOSES.  You can only enable
// one of these at a time.
//
// #define DEBUG_6502
// #define DEBUG_6809E
// #define DEBUG_Z80
//
#include "test_samples.h"

// Macros to await signal transitions.
#define WAIT_PHI2_LOW while (digitalReadFast(CC_6502_PHI2_PIN) == HIGH) ;
#define WAIT_PHI2_HIGH while (digitalReadFast(CC_6502_PHI2_PIN) == LOW) ;
#define WAIT_Q_LOW while (digitalReadFast(CC_6809_Q) == HIGH) ;
#define WAIT_Q_HIGH while (digitalReadFast(CC_6809_Q) == LOW) ;
#define WAIT_E_LOW while (digitalReadFast(CC_6809_E) == HIGH) ;
#define WAIT_E_HIGH while (digitalReadFast(CC_6809_E) == LOW) ;
#define WAIT_CLK_LOW while (digitalReadFast(CC_Z80_CLK_PIN) == HIGH) ;
#define WAIT_CLK_HIGH while (digitalReadFast(CC_Z80_CLK_PIN) == LOW) ;

uint32_t
scramble_CAxx(uint32_t ca)
{
  return ((ca & (1U <<  0)) ?  CA0_PIN_BITMASK : 0) |
         ((ca & (1U <<  1)) ?  CA1_PIN_BITMASK : 0) |
         ((ca & (1U <<  2)) ?  CA2_PIN_BITMASK : 0) |
         ((ca & (1U <<  3)) ?  CA3_PIN_BITMASK : 0) |
         ((ca & (1U <<  4)) ?  CA4_PIN_BITMASK : 0) |
         ((ca & (1U <<  5)) ?  CA5_PIN_BITMASK : 0) |
         ((ca & (1U <<  6)) ?  CA6_PIN_BITMASK : 0) |
         ((ca & (1U <<  7)) ?  CA7_PIN_BITMASK : 0) |
         ((ca & (1U <<  8)) ?  CA8_PIN_BITMASK : 0) |
         ((ca & (1U <<  9)) ?  CA9_PIN_BITMASK : 0) |
         ((ca & (1U << 10)) ? CA10_PIN_BITMASK : 0) |
         ((ca & (1U << 11)) ? CA11_PIN_BITMASK : 0) |
         ((ca & (1U << 12)) ? CA12_PIN_BITMASK : 0) |
         ((ca & (1U << 13)) ? CA13_PIN_BITMASK : 0) |
         ((ca & (1U << 14)) ? CA14_PIN_BITMASK : 0) |
         ((ca & (1U << 15)) ? CA15_PIN_BITMASK : 0);
}

uint32_t
unscramble_CAxx(uint32_t reg)
{
  return ((reg &  CA0_PIN_BITMASK) ? (1U <<  0) : 0) |
         ((reg &  CA1_PIN_BITMASK) ? (1U <<  1) : 0) |
         ((reg &  CA2_PIN_BITMASK) ? (1U <<  2) : 0) |
         ((reg &  CA3_PIN_BITMASK) ? (1U <<  3) : 0) |
         ((reg &  CA4_PIN_BITMASK) ? (1U <<  4) : 0) |
         ((reg &  CA5_PIN_BITMASK) ? (1U <<  5) : 0) |
         ((reg &  CA6_PIN_BITMASK) ? (1U <<  6) : 0) |
         ((reg &  CA7_PIN_BITMASK) ? (1U <<  7) : 0) |
         ((reg &  CA8_PIN_BITMASK) ? (1U <<  8) : 0) |
         ((reg &  CA9_PIN_BITMASK) ? (1U <<  9) : 0) |
         ((reg & CA10_PIN_BITMASK) ? (1U << 10) : 0) |
         ((reg & CA11_PIN_BITMASK) ? (1U << 11) : 0) |
         ((reg & CA12_PIN_BITMASK) ? (1U << 12) : 0) |
         ((reg & CA13_PIN_BITMASK) ? (1U << 13) : 0) |
         ((reg & CA14_PIN_BITMASK) ? (1U << 14) : 0) |
         ((reg & CA15_PIN_BITMASK) ? (1U << 15) : 0);
}

uint32_t
scramble_CDxx(uint32_t cd)
{
    return ((cd & (1U << 0)) ? CD0_PIN_BITMASK : 0) |
           ((cd & (1U << 1)) ? CD1_PIN_BITMASK : 0) |
           ((cd & (1U << 2)) ? CD2_PIN_BITMASK : 0) |
           ((cd & (1U << 3)) ? CD3_PIN_BITMASK : 0) |
           ((cd & (1U << 4)) ? CD4_PIN_BITMASK : 0) |
           ((cd & (1U << 5)) ? CD5_PIN_BITMASK : 0) |
           ((cd & (1U << 6)) ? CD6_PIN_BITMASK : 0) |
           ((cd & (1U << 7)) ? CD7_PIN_BITMASK : 0);
}

uint32_t
unscramble_CDxx(uint32_t reg)
{
  return ((reg & CD0_PIN_BITMASK) ? (1U << 0) : 0) |
         ((reg & CD1_PIN_BITMASK) ? (1U << 1) : 0) |
         ((reg & CD2_PIN_BITMASK) ? (1U << 2) : 0) |
         ((reg & CD3_PIN_BITMASK) ? (1U << 3) : 0) |
         ((reg & CD4_PIN_BITMASK) ? (1U << 4) : 0) |
         ((reg & CD5_PIN_BITMASK) ? (1U << 5) : 0) |
         ((reg & CD6_PIN_BITMASK) ? (1U << 6) : 0) |
         ((reg & CD7_PIN_BITMASK) ? (1U << 7) : 0);
}

uint32_t
scramble_CCxx(uint32_t cc, uint32_t *aregp, uint32_t *dregp)
{
  if (cc & CC6_BITMASK) {
    *aregp |= CC6_PIN_BITMASK;
  }
  if (cc & CC11_BITMASK) {
    *aregp |= CC11_PIN_BITMASK;
  }
  if (cc & CC12_BITMASK) {
    *aregp |= CC12_PIN_BITMASK;
  }
  if (cc & CC13_BITMASK) {
    *aregp |= CC13_PIN_BITMASK;
  }

  if (cc & CC7_BITMASK) {
    *dregp |= CC7_PIN_BITMASK;
  }
  if (cc & CC8_BITMASK) {
    *dregp |= CC8_PIN_BITMASK;
  }
  if (cc & CC9_BITMASK) {
    *dregp |= CC9_PIN_BITMASK;
  }
  if (cc & CC10_BITMASK) {
    *dregp |= CC10_PIN_BITMASK;
  }

  return ((cc & CC0_BITMASK) ? CC0_PIN_BITMASK : 0) |
         ((cc & CC1_BITMASK) ? CC1_PIN_BITMASK : 0) |
         ((cc & CC2_BITMASK) ? CC2_PIN_BITMASK : 0) |
         ((cc & CC3_BITMASK) ? CC3_PIN_BITMASK : 0) |
         ((cc & CC4_BITMASK) ? CC4_PIN_BITMASK : 0) |
         ((cc & CC5_BITMASK) ? CC5_PIN_BITMASK : 0);
}

uint32_t
unscramble_CCxx(uint32_t creg, uint32_t areg, uint32_t dreg)
{
  return ((creg &  CC0_PIN_BITMASK) ?  CC0_BITMASK : 0) |
         ((creg &  CC1_PIN_BITMASK) ?  CC1_BITMASK : 0) |
         ((creg &  CC2_PIN_BITMASK) ?  CC2_BITMASK : 0) |
         ((creg &  CC3_PIN_BITMASK) ?  CC3_BITMASK : 0) |
         ((creg &  CC4_PIN_BITMASK) ?  CC4_BITMASK : 0) |
         ((creg &  CC5_PIN_BITMASK) ?  CC5_BITMASK : 0) |
         // CC6 is in the areg
         ((areg &  CC6_PIN_BITMASK) ?  CC6_BITMASK : 0) |
         // CC7-10 are in the dreg
         ((dreg &  CC7_PIN_BITMASK) ?  CC7_BITMASK : 0) |
         ((dreg &  CC8_PIN_BITMASK) ?  CC8_BITMASK : 0) |
         ((dreg &  CC9_PIN_BITMASK) ?  CC9_BITMASK : 0) |
         ((dreg & CC10_PIN_BITMASK) ? CC10_BITMASK : 0) |
         // CC11-13 are in the areg
         ((areg & CC11_PIN_BITMASK) ? CC11_BITMASK : 0) |
         ((areg & CC12_PIN_BITMASK) ? CC12_BITMASK : 0) |
         ((areg & CC13_PIN_BITMASK) ? CC13_BITMASK : 0);
}

// Rearrange sampled bits of data in buffer back into address, data,
// and control lines.
void
unscramble(void)
{
  for (int i = 0; i < samples; i++) {
    // Unscramble control signals first; some of them have bits in
    // the data and address line GPIO ports.
    control[i] = unscramble_CCxx(control[i], address[i], data[i]);
    address[i] = unscramble_CAxx(address[i]);
    data[i]    = unscramble_CDxx(data[i]);
   }
}

void
setBusEnabled(bool e)
{
  digitalWriteFast(ENABLE_PIN, e ? LOW : HIGH);
}

// Startup function
void
setup(void)
{
  // Configure all of the pins.  We use all of the digital I/O
  // pins vailable on the Teensy 4.1 board.
  for (int i = 0; i <= 41; i++) {
    switch (i) {
      case CORE_LED0_PIN:
        // LED pin is push-pull ouput.  This is used to indicate
        // that we're waiting to trigger.
        pinMode(CORE_LED0_PIN, OUTPUT);
        break;

      case ENABLE_PIN:
        // ENABLE pin is an open-drain output (there is a pull-up
        // resistor connected to the /CE inputs of the 74LVC245s).
        pinMode(ENABLE_PIN, OUTPUT_OPENDRAIN);
        setBusEnabled(false);
        break;

      case BUTTON_PIN:
        // The button goes to ground; we need to figure as an
        // input-pullup.  Interrupt is triggered on the falling
        // edge.
        pinMode(BUTTON_PIN, INPUT_PULLUP);
        break;

      default:
        // Every other pin is configured as an input.  *No* pull-up;
        // the 74LVC245 has push-pull outputs.
        pinMode(i, INPUT);
        break;
    }
  }

  // Manual trigger button - low on this pin forces a trigger.
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), triggerButton, FALLING);

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB.
  }

  Serial.setTimeout(60000);
  show_version(false);
  tla_printf("Type \"h\" or \"?\" for help.\n");
}

// Interrupt handler for trigger button.
void
triggerButton(void)
{
  triggerPressed = true;
}

const char *
cpu_name(void)
{
  switch (cpu) {
    case cpu_6502:    return "6502";
    case cpu_65c02:   return "65C02";
    case cpu_6800:    return "6800";
    case cpu_6809:    return "6809";
    case cpu_6809e:   return "6809E";
    case cpu_z80:     return "Z80";
    default:          return "not set";
  }
}

void
show_cpu(void)
{
  tla_printf("CPU: %s\n", cpu_name());
}

void
set_cpu(cpu_t ncpu)
{
  if (triggerSpace == tr_io && !cpu_has_iospace(ncpu)) {
    triggerSpace = tr_mem;
    triggerMode = tr_none;
  }
  cpu = ncpu;
}

const char *
trigger_signal_name(trigger_t t)
{
  switch (t) {
    case tr_reset:    return "/RESET";
    case tr_irq:      return cpu == cpu_z80 ? "/INT" : "/IRQ";
    case tr_firq:     return "/FIRQ";
    case tr_nmi:      return "/NMI";
    default:          return "<unknown>";
  }
}

const char *
trigger_cycle_name(cycle_t c)
{
  switch (c) {
    case tr_read:       return "read";
    case tr_write:      return "write";
    case tr_either:     return "read or write";
    default:            return "<unknown>";
  }
}

void
show_trigger(void)
{
  char msg[80], *cp = msg;

  cp += sprintf(cp, "Trigger: ");
  switch (triggerMode) {
    case tr_address:
    case tr_data:
    case tr_addr_data:
      cp = msg;
      cp += sprintf(cp, "on%s %s ",
          triggerSpace == tr_io ? " io" : "",
          triggerMode == tr_data ? "data" : "address");
      if (triggerMode != tr_data) {
        if (triggerSpace == tr_io) {
          cp += sprintf(cp, "%02lX ", triggerAddress);
        } else {
          cp += sprintf(cp, "%04lX ", triggerAddress);
        }
      } else {
        cp += sprintf(cp, "%02lX ", triggerData);
      }
      if (triggerMode == tr_addr_data) {
        cp += sprintf(cp, "and data %02lX ", triggerData);
      }
      cp += sprintf(cp, "%s", trigger_cycle_name(triggerCycle));
      break;

    case tr_reset:
    case tr_irq:
    case tr_firq:
    case tr_nmi:
      cp += sprintf(cp, "on %s %s", trigger_signal_name(triggerMode),
          triggerLevel ? "high" : "low");
      break;

    case tr_manual:
      cp += sprintf(cp, "manual (button)");
      break;

    case tr_none:
      cp += sprintf(cp, "none (immediate)");
      break;
  }
  tla_printf("%s\n", msg);
}

void
show_samples(void)
{
  tla_printf("Sample buffer size: %d\n", samples);
}

void
show_pretrigger(void)
{
  tla_printf("Pretrigger samples: %d\n", pretrigger);
}

void
disassemble_one(uint32_t where)
{
  struct insn_decode id;
  insn_decode_init(&id);

  int first = (triggerPoint - pretrigger + samples) % samples;
  int last = (triggerPoint - pretrigger + samples - 1) % samples;
  int i;

  // First, search through the sample data looking for the address.
  for (i = first; i != last; i = (i + 1) % samples) {
    if (address[i] == where) {
      break;
    }
  }
  if (address[i] != where) {
    tla_printf("Address not found in sample data.\n");
    return;
  }

  for (;; i = (i + 1) % samples) {
    switch (id.state) {
      case ds_idle:
        insn_decode_begin(&id, address[i], data[i]);
        goto printit;

      case ds_fetching:
        if (address[i] != where + id.bytes_fetched) {
          tla_printf("!!!! Non-contiguous instruction fetch?\n");
        }
        // FALLTHROUGH

      default:
        insn_decode_continue(&id, data[i]);
      printit:
        tla_printf("%04lX  %02lX  %s\n",
            address[i], data[i], insn_decode_complete(&id));
        break;
    }
    if (id.state == ds_complete || i == last) {
      return;
    }
  }
}

// List recorded data from start to end.
void
list(Stream &stream, int start, int end, int validSamples)
{
  char output[80];
  char comment[30], *cp;

  if (cpu == cpu_none || validSamples == 0) {
    return;
  }

  int first = (triggerPoint - pretrigger + samples) % samples;
  int last = (triggerPoint - pretrigger + samples - 1) % samples;

  bool have_lic, seen_lic = false;

  const char *cycle, *trig;
  const char *comma;

  struct insn_decode id;
  insn_decode_init(&id);

  // Display data
  int i = first;
  int j = 0;
  while (true) {
    cycle = "";
    trig = "";
    comma = "";
    comment[0] = '\0';
    cp = comment;

    if ((j >= start) && (j <= end)) {

      // 6502 SYNC high indicates opcode/instruction fetch, otherwise
      // show as read or write.
      if ((cpu == cpu_65c02) || (cpu == cpu_6502)) {
        if (control[i] & CC_6502_SYNC) {
          insn_decode_begin(&id, address[i], data[i]);
          cycle = "F";
        } else if (control[i] & CC_6502_RW) {
          cycle = insn_decode_continue(&id, data[i]) ? "*" : "R";
        } else {
          cycle = "W";
        }
      }

      if (cpu == cpu_6809 || cpu == cpu_6809e) {
        // Get the current status of LIC.  Note that LIC will also
        // be high while the processor is in SYNC state or while
        // stacking registers during an interrupt.
        have_lic = (cpu == cpu_6809e && (control[i] & CC_6809E_LIC));

        // 6809 doens't have a VMA signal like the 6800, but the
        // data sheet describes how to detect a so-called "dummy
        // cycle" (which is also calls "/VMA").
        if (address[i] == 0xffff &&
            (control[i] & (CC_6809_RW | CC_6809_BS)) == CC_6809_RW) {
          cycle = "-";
        } else if (control[i] & CC_6809_RW) {
          // On 6809E, if we saw LIC on the previous cycle, then
          // this is an insn fetch.  Don't try to decode an instruction
          // if it looks like we're doing a vector fetch, though.
          cycle = "R";
          if (cpu == cpu_6809e && address[i] < 0xfff0) {
            // Even if we have seen LIC go by, it's not an
            // instruction fetch until LIC goes low.
            if (seen_lic && !have_lic) {
              cycle = "F";
              insn_decode_begin(&id, address[i], data[i]);
              seen_lic = false;
            } else {
              if (insn_decode_continue(&id, data[i])) {
                cycle = "*";
              }
            }
          }
        } else {
          cycle = "W";
        }
        if (have_lic) {
          seen_lic = true;
        }
      }

      if (cpu == cpu_z80) {
        // /M1 /MREQ  /IORQ /RD /WR
        //  1    0      1    0   1   Memory read
        //  1    0      1    1   0   Memory write
        //  0    0      1    0   1   Instruction fetch
        //  1    1      0    0   1   I/O read
        //  1    1      0    1   0   I/O write

        if (!(control[i] & CC_Z80_M1)) {
          cycle = "F";
          insn_decode_begin(&id, address[i], data[i]);
        } else if (!(control[i] & CC_Z80_MREQ) && !(control[i] & CC_Z80_RD)) {
          cycle = insn_decode_continue(&id, data[i]) ? "*" : "R";
        } else if (!(control[i] & CC_Z80_MREQ) && !(control[i] & CC_Z80_WR)) {
          cycle = "W";
        } else if (!(control[i] & CC_Z80_IORQ) && !(control[i] & CC_Z80_RD)) {
          cycle = "IR";
        } else if (!(control[i] & CC_Z80_IORQ) && !(control[i] & CC_Z80_WR)) {
          cycle = "IW";
        }
      }

      if (cpu == cpu_6800) {
        // VMA R/W
        //  0   X  Internal cycle
        //  1   0  Memory read
        //  1   1  Memory write
        if (!(control[i] & CC_6800_VMA)) {
          cycle = "-";
        } else {
          if (control[i] & CC_6800_RW) {
            cycle = "R";
          } else {
            cycle = "W";
          }
        }
      }

#define COMMENT(str) do { cp += sprintf(cp, "%s%s", comma, str); comma = ","; } while (0)

      // Check for 6502 /RESET, /IRQ, or /NMI active, vector address, or
      // stack access
      if ((cpu == cpu_65c02) || (cpu == cpu_6502)) {
        if (!(control[i] & CC_6502_RESET)) {
          COMMENT("RESET");
        }
        if (!(control[i] & CC_6502_IRQ)) {
          COMMENT("IRQ");
        }
        if (!(control[i] & CC_6502_NMI)) {
          COMMENT("NMI");
        }
        if ((address[i] == 0xfffa) || (address[i] == 0xfffb)) {
          COMMENT("NMI VECTOR");
        } else if ((address[i] == 0xfffc) || (address[i] == 0xfffd)) {
          COMMENT("RESET VECTOR");
        } else if ((address[i] == 0xfffe) || (address[i] == 0xffff)) {
          COMMENT("IRQ/BRK VECTOR");
        } else if ((address[i] >= 0x0100) && (address[i] <= 0x01ff)) {
          COMMENT("STACK ACCESS");
        }
      }

      // Check for 6800 /RESET, /IRQ, or /NMI active, vector address.
      if (cpu == cpu_6800) {
        if (!(control[i] & CC_6800_RESET)) {
          COMMENT("RESET");
        }
        if (!(control[i] & CC_6800_IRQ)) {
          COMMENT("IRQ");
        }
        if (!(control[i] & CC_6800_NMI)) {
          COMMENT("NMI");
        }
        if ((address[i] == 0xfff8) || (address[i] == 0xfff8)) {
          COMMENT("IRQ VECTOR");
        } else if ((address[i] == 0xfffa) || (address[i] == 0xfffb)) {
          COMMENT("SWI VECTOR");
        } else if ((address[i] == 0xfffc) || (address[i] == 0xfffd)) {
          COMMENT("NMI VECTOR");
        } else if (address[i] == 0xfffe) { // Not 0xffff since it commonly occurs when bus is tri-state
          COMMENT("RESET VECTOR");
        }
      }

      // Check for 6809 /RESET, /IRQ, or /NMI active, vector address.
      if (cpu == cpu_6809 || cpu == cpu_6809e) {
        if (!(control[i] & CC_6809_RESET)) {
          COMMENT("RESET");
        }
        if (!(control[i] & CC_6809_IRQ)) {
          COMMENT("IRQ");
        }
        if (!(control[i] & CC_6809_FIRQ)) {
          COMMENT("FIRQ");
        }
        if (!(control[i] & CC_6809_NMI)) {
          COMMENT("NMI");
        }
        if ((address[i] == 0xfff2) || (address[i] == 0xfff3)) {
          COMMENT("SWI3 VECTOR");
        } else if ((address[i] == 0xfff4) || (address[i] == 0xfff5)) {
          COMMENT("SWI2 VECTOR");
        } else if ((address[i] == 0xfff6) || (address[i] == 0xfff7)) {
          COMMENT("FIRQ VECTOR");
        } else if ((address[i] == 0xfff8) || (address[i] == 0xfff9)) {
          COMMENT("IRQ VECTOR");
        } else if ((address[i] == 0xfffa) || (address[i] == 0xfffb)) {
          COMMENT("SWI VECTOR");
        } else if ((address[i] == 0xfffc) || (address[i] == 0xfffd)) {
          COMMENT("NMI VECTOR");
        } else if (address[i] == 0xfffe) { // Not 0xffff since it commonly occurs when bus is tri-state
          COMMENT("RESET VECTOR");
        }
      }

      // Check for Z80 /RESET or /INT active
      if (cpu == cpu_z80) {
        if (!(control[i] & CC_Z80_RESET)) {
          COMMENT("RESET");
        }
        if (!(control[i] & CC_Z80_INT)) {
          COMMENT("INT");
        }
      }

#undef COMMENT

      // Indicate when trigger happened
      if (i == triggerPoint) {
        trig = "<--";
      }

      // This printf format needs to be kept in sync with INSN_DECODE_MAXSTRING.
      sprintf(output,
          "%04lX  %-2s  %02lX  %-28s  %-3s  %s",
          address[i], cycle, data[i], insn_decode_complete(&id),
          trig, comment);

      stream.println(output);
    }

    if (i == last) {
      break;
    }

    i = (i + 1) % samples;
    j++;
  }
}

#define EXPORT_CC(s)  (control[i] & (s)) ? '1' : '0'

const char *
exportCSV_header_6502(void)
{
  return "Index,Trigger,SYNC,R/W,/RESET,/IRQ,/NMI,Address,Data";
}

void
exportCSV_entry_6502(int i, int j, char *output)
{
  sprintf(output, "%d,%d,%c,%c,%c,%c,%c,%04lX,%02lX", j, i == triggerPoint,
      EXPORT_CC(CC_6502_SYNC),
      EXPORT_CC(CC_6502_RW),
      EXPORT_CC(CC_6502_RESET),
      EXPORT_CC(CC_6502_IRQ),
      EXPORT_CC(CC_6502_NMI),
      address[i], data[i]);
}

const char *
exportCSV_header_6800(void)
{
  return "Index,Trigger,VMA,R/W,/RESET,/IRQ,/NMI,Address,Data";
}

void
exportCSV_entry_6800(int i, int j, char *output)
{
  sprintf(output, "%d,%d,%c,%c,%c,%c,%c,%04lX,%02lX", j, i == triggerPoint,
      EXPORT_CC(CC_6800_VMA),
      EXPORT_CC(CC_6800_RW),
      EXPORT_CC(CC_6800_RESET),
      EXPORT_CC(CC_6800_IRQ),
      EXPORT_CC(CC_6800_NMI),
      address[i], data[i]);
}

const char *
exportCSV_header_6809(void)
{
  return "Index,Trigger,BA,BS,R/W,/RESET,/IRQ,/FIRQ,/NMI,Address,Data";
}

void
exportCSV_entry_6809(int i, int j, char *output)
{
  sprintf(output, "%d,%d,%c,%c,%c,%c,%c,%c,%c,%04lX,%02lX",j, i == triggerPoint,
      EXPORT_CC(CC_6809_BA),
      EXPORT_CC(CC_6809_BS),
      EXPORT_CC(CC_6809_RW),
      EXPORT_CC(CC_6809_RESET),
      EXPORT_CC(CC_6809_IRQ),
      EXPORT_CC(CC_6809_FIRQ),
      EXPORT_CC(CC_6809_NMI),
      address[i], data[i]);
}

const char *
exportCSV_header_6809e(void)
{
  return "Index,Trigger,BA,BS,LIC,R/W,/RESET,/IRQ,/FIRQ,/NMI,Address,Data";
}

void
exportCSV_entry_6809e(int i, int j, char *output)
{
  sprintf(output, "%d,%d,%c,%c,%c,%c,%c,%c,%c,%c,%04lX,%02lX", j, i == triggerPoint,
      EXPORT_CC(CC_6809_BA),
      EXPORT_CC(CC_6809_BS),
      EXPORT_CC(CC_6809E_LIC),
      EXPORT_CC(CC_6809_RW),
      EXPORT_CC(CC_6809_RESET),
      EXPORT_CC(CC_6809_IRQ),
      EXPORT_CC(CC_6809_FIRQ),
      EXPORT_CC(CC_6809_NMI),
      address[i], data[i]);
}

const char *
exportCSV_header_z80(void)
{
  return "Index,Trigger,/M1,/RD,/WR,/MREQ,/IORQ,/RESET,/INT,Address,Data";
}

void
exportCSV_entry_z80(int i, int j, char *output)
{
  sprintf(output, "%d,%d,%c,%c,%c,%c,%c,%c,%c,%04lX,%02lX", j, i == triggerPoint,
      EXPORT_CC(CC_Z80_M1),
      EXPORT_CC(CC_Z80_RD),
      EXPORT_CC(CC_Z80_WR),
      EXPORT_CC(CC_Z80_MREQ),
      EXPORT_CC(CC_Z80_IORQ),
      EXPORT_CC(CC_Z80_RESET),
      EXPORT_CC(CC_Z80_INT),
      address[i], data[i]);
}

#undef EXPORT_CCC

// Show the recorded data in CSV format (e.g. to export to spreadsheet or other program).
void
exportCSV(Stream &stream, int validSamples)
{
  void (*export_entry)(int, int, char *);
  const char *header;
  char output[50];

  if (validSamples == 0) {
      return;
  }

  // Output header
  switch (cpu) {
    case cpu_6502:
    case cpu_65c02:
      header = exportCSV_header_6502();
      export_entry = exportCSV_entry_6502;
      break;

    case cpu_6809:
      header = exportCSV_header_6809();
      export_entry = exportCSV_entry_6809;
      break;

    case cpu_6809e:
      header = exportCSV_header_6809e();
      export_entry = exportCSV_entry_6809e;
      break;

    case cpu_6800:
      header = exportCSV_header_6800();
      export_entry = exportCSV_entry_6800;
      break;

    case cpu_z80:
      header = exportCSV_header_z80();
      export_entry = exportCSV_entry_z80;
      break;

    default:
      return;
  }
  stream.println(header);

  int first = (triggerPoint - pretrigger + samples) % samples;
  int last = (triggerPoint - pretrigger + samples - 1) % samples;

  // Display data
  int i = first;
  int j = 0;
  while (true) {
    (*export_entry)(i, j, output);
    stream.println(output);

    if (i == last) {
      break;
    }

    i = (i + 1) % samples;
    j++;
  }
}


// Write the recorded data to files on the internal SD card slot.
void
writeSD(void)
{
  const char *CSV_FILE = "analyzer.csv";
  const char *TXT_FILE = "analyzer.txt";

  if (cpu == cpu_none || samplesTaken == 0) {
    tla_printf("No samples to save.\n");
    return;
  }

  if (!SD.begin(BUILTIN_SDCARD)) {
    tla_printf("Unable to initialize internal SD card.\n");
    return;
  }

  // Remove any existing file
  if (SD.exists(CSV_FILE)) {
    SD.remove(CSV_FILE);
  }

  File file = SD.open(CSV_FILE, FILE_WRITE);
  if (file) {
    tla_printf("Writing %s\n", CSV_FILE);
    exportCSV(file, samplesTaken);
    file.close();
  } else {
    tla_printf("Unable to write %s\n", CSV_FILE);
  }

  // Remove any existing file
  if (SD.exists(TXT_FILE)) {
    SD.remove(TXT_FILE);
  }

  file = SD.open(TXT_FILE, FILE_WRITE);
  if (file) {
    tla_printf("Writing %s\n", TXT_FILE);
    list(file, 0, samples - 1, samplesTaken);
    file.close();
  } else {
    tla_printf("Unable to write %s\n", TXT_FILE);
  }
}


// Start recording.
void
go(void)
{
  aTriggerBits = 0;
  aTriggerMask = 0;
  dTriggerBits = 0;
  dTriggerMask = 0;
  cTriggerBits = 0;
  cTriggerMask = 0;

  uint32_t which_c_trigger = 0;
  uint32_t cd_psr_cc_bits;

  if (cpu == cpu_none) {
    tla_printf("No CPU type selected!\n");
  }

  // Scramble the trigger address, control, and data lines to match what we will read on the ports.

  if (triggerMode == tr_address || triggerMode == tr_data || triggerMode == tr_addr_data) {

    if (triggerMode == tr_address || triggerMode == tr_addr_data) {
      aTriggerBits = scramble_CAxx(triggerAddress);
      if (triggerSpace == tr_io) {
        aTriggerMask = scramble_CAxx(0xff);
      } else {
        aTriggerMask = scramble_CAxx(0xffff);
      }
    }
    if (triggerMode == tr_data || triggerMode == tr_addr_data) {
      dTriggerBits = scramble_CDxx(triggerData);
      dTriggerMask = scramble_CDxx(0xff);
    }

    // Check for r/w qualifer
    if (cpu != cpu_z80) {
      // 6502, 6800, 6809 -- all 6800-like
      cTriggerMask = scramble_CCxx(CC_6800_RW, &aTriggerMask, &dTriggerMask);
      if (triggerCycle == tr_read) {
        cTriggerBits = scramble_CCxx(CC_6800_RW, &aTriggerBits, &dTriggerBits);
      }
    } else {
      uint32_t tmask, tbits;

      tmask = CC_Z80_MREQ | CC_Z80_IORQ;
      if (triggerSpace == tr_io) {
        tbits = CC_Z80_MREQ;              // I/O cycle
      } else {
        tbits = CC_Z80_IORQ;              // Memory cycle
      }
      if (triggerCycle == tr_read) {
        tmask |= CC_Z80_RD | CC_Z80_WR;
        tbits |= CC_Z80_WR;               // Read cycle
      } else if (triggerCycle == tr_write) {
        tmask |= CC_Z80_RD | CC_Z80_WR;
        tbits |= CC_Z80_RD;               // Write cycle
      }
      cTriggerMask = scramble_CCxx(tmask, &aTriggerMask, &dTriggerMask);
      cTriggerBits = scramble_CCxx(tbits, &aTriggerBits, &dTriggerBits);
    }

  } else if (triggerMode == tr_reset) {
    if (cpu != cpu_z80) {
      // 6502, 6800, 6809 -- all 6800-like
      which_c_trigger = CC_6800_RESET;
    } else {
      which_c_trigger = CC_Z80_RESET;
    }
  } else if (triggerMode == tr_irq) {
    if (cpu != cpu_z80) {
      // 6502, 6800, 6809 -- all 6800-like
      which_c_trigger = CC_6800_IRQ;
    } else {
      which_c_trigger = CC_Z80_INT;
    }
  } else if (triggerMode == tr_firq) {
    if (cpu == cpu_6809 || cpu == cpu_6809e) {
      which_c_trigger = CC_6809_FIRQ;
    }
  } else if (triggerMode == tr_nmi) {
    if (cpu != cpu_z80) {
      // 6502, 6800, 6809 -- all 6800-like
      which_c_trigger = CC_6800_NMI;
    } else {
      which_c_trigger = CC_Z80_NMI;
    }
  }
  // TODO: Add support other Z80 control line triggers.

  // If a control signal trigger was specified, encode it.
  if (which_c_trigger) {
    cTriggerMask = scramble_CCxx(which_c_trigger, &aTriggerMask, &dTriggerMask);
    if (triggerLevel) {
      cTriggerBits = scramble_CCxx(which_c_trigger, &aTriggerBits, &dTriggerBits);
    }
  }

  tla_printf("Waiting for trigger...\n");

  // tr_none is like the button is pressed instantly.
  triggerPressed = triggerMode == tr_none;

  setBusEnabled(true);
  digitalWriteFast(CORE_LED0_PIN, HIGH); // Indicates waiting for trigger

  int i = 0; // Index into data buffers
  bool triggered = false; // Set when triggered

  samplesTaken = 0;

  while (true) {

    if ((cpu == cpu_65c02) || (cpu == cpu_6502) || (cpu == cpu_6800)) {
      // Wait for PHI2 to go from low to high
      WAIT_PHI2_LOW;
      WAIT_PHI2_HIGH;
    }
    if (cpu == cpu_6809 || cpu == cpu_6809e) {
      // Wait for Q to go from low to high
      WAIT_Q_LOW;
      WAIT_Q_HIGH;
    }
    if (cpu == cpu_z80) {
      // Wait CLK to go from high to low
      WAIT_CLK_HIGH;
      WAIT_CLK_LOW;
    }

    // Read address and control lines.  Note that some of the control
    // lines are in the CDxx_PSR, and we need to extract those as well.
    control[i] = CCxx_PSR;
    address[i] = CAxx_PSR;
    cd_psr_cc_bits = CDxx_PSR & CDxx_PSR_CC_MASK;

    if ((cpu == cpu_65c02) || (cpu == cpu_6502) || (cpu == cpu_6800)) {
      // Wait for PHI2 to go from high to low
      WAIT_PHI2_HIGH;
      WAIT_PHI2_LOW;
    }
    if (cpu == cpu_6809 || cpu == cpu_6809e) {
      // Wait for E to go from high to low
      WAIT_E_HIGH;
      WAIT_E_LOW;
    }
    if (cpu == cpu_z80) {
      // Wait CLK to go from low to high
      WAIT_CLK_LOW;
      WAIT_CLK_HIGH;
    }

    // Read data lines.  Mask out the control bits on this
    // read and mix in the control bits read above.
    data[i] = (CDxx_PSR & CDxx_PSR_CD_MASK) | cd_psr_cc_bits;

    // Set triggered flag if trigger button pressed or trigger seen
    // If triggered, increment buffer index
    if (!triggered) {
      if (triggerPressed ||
          (((address[i] & aTriggerMask) == (aTriggerBits & aTriggerMask)) &&
           ((data[i] & dTriggerMask) == (dTriggerBits & dTriggerMask)) &&
           ((control[i] & cTriggerMask) == (cTriggerBits & cTriggerMask)))) {
        triggered = true;
        triggerPoint = i;
        digitalWriteFast(CORE_LED0_PIN, LOW); // Indicates received trigger
      }
    }

    // Count number of samples taken after trigger
    if (triggered) {
      samplesTaken++;
    }

    // Exit when buffer is full of samples
    if (samplesTaken >= (samples - pretrigger)) {
      break;
    }

    i = (i + 1) % samples; // Increment index, wrapping around at end for circular buffer
  }

  setBusEnabled(false);

  tla_printf("Data recorded (%d samples).\n", samples);
  unscramble();
}

bool
parseHexNumber(char *cp, uint32_t *valp)
{
  char *endptr = NULL;

  if (cp[0] == '0' && (cp[1] == 'x' || cp[1] == 'X')) {
    cp += 2;
  } else if (cp[0] == '$') {
    cp++;
  } else {
    size_t len = strlen(cp);
    if (len != 0 && (cp[len - 1] == 'h' || cp[len - 1] == 'H')) {
      cp[len - 1] = '\0';
    }
  }

  int rv = strtol(cp, &endptr, 16);
  if (endptr[0] != '\0') {
    return false;
  }
  *valp = rv;
  return true;
}

bool
parseDecimalNumber(char *cp, int *valp)
{
  char *endptr = NULL;

  int rv = strtol(cp, &endptr, 10);
  if (endptr[0] != '\0') {
    return false;
  }
  *valp = rv;
  return true;
}

bool
parseAddress(char *cp, space_t space, uint32_t *addrp)
{
  uint32_t maxaddr = space == tr_io ? 0xff : 0xffff;
  uint32_t val;

  if (! parseHexNumber(cp, &val) || val > maxaddr) {
    tla_printf("Invalid address.\n");
    return false;
  }
  *addrp = val;
  return true;
}

int
stringMatch(const char *match, const char *user)
{
  int rv = 0;
  size_t matchlen = strlen(match);
  size_t userlen = strlen(user);

  if (matchlen == userlen) {
    if (strcmp(match, user) == 0) {
      return (int)matchlen;
    } else {
      return 0;
    }
  } else if (matchlen < userlen) {
    return 0;
  }

  // We now know that the user string is shorter than
  // the match string.

  for (rv = 0; *user != '\0'; rv++) {
    if (*user++ != *match++) {
      return 0;
    }
  }
  return rv;
}

//
// Command parsing and execution
//
#define MAX_ARGS  8
int argc;
char *argv[MAX_ARGS];


#define CMDBUF_LEN  64
char cmdbuf[CMDBUF_LEN];
char saved_cmdbuf[CMDBUF_LEN];

const struct {
  const char *cpustr;
  cpu_t cputype;
} cputab[] = {
  "6502",     cpu_6502,
  "65C02",    cpu_65c02,
  "6800",     cpu_6800,
  "6809",     cpu_6809,
  "6809E",    cpu_6809e,
  "Z80",      cpu_z80,
  NULL,       cpu_none,
};

void
help_cpu(void)
{
  int i;

  tla_printf("usage: cpu        - show current CPU type\n");
  tla_printf("       cpu <type> - set CPU type\n");
  for (i = 0; cputab[i].cpustr != NULL; i++) {
    tla_printf("%s%s%s\n",
        i == 0 ? "\n" : "",
        i == 0 ? "<type> must be: "
               : "                ",
        cputab[i].cpustr);
  }
}

void
command_cpu(void)
{
  if (argc == 1) {
    show_cpu();
    return;
  } else if (argc != 2) {
    help_cpu();
    return;
  }

  int i;
  for (i = 0; cputab[i].cpustr != NULL; i++) {
    if (strcasecmp(cputab[i].cpustr, argv[1]) == 0) {
      set_cpu(cputab[i].cputype);
      if (triggerMode != tr_none && triggerMode != tr_manual) {
        triggerMode = tr_none;
        tla_printf("WARNING: trigger mode reset\n");
      }
      return;
    }
  }
  tla_printf("Invalid CPU type: %s\n", argv[1]);
}

void
help_samples(void)
{
  tla_printf("usage: samples         - show current sample count\n");
  tla_printf("       samples <count> - set sample count\n");
  tla_printf("\n<count> must be between 1 and %d.\n", BUFFSIZE);
}

void
command_samples(void)
{
  if (argc == 1) {
    show_samples();
    return;
  } else if (argc != 2) {
    help_samples();
    return;
  }

  int c;
  c = (int) strtol(argv[1], NULL, 10);
  if (c < 1 || c > BUFFSIZE) {
    tla_printf("Invalid sample count.\n");
    help_samples();
    return;
  }

  samples = c;
  if (pretrigger > c) {
    pretrigger = 0;
    tla_printf("WARNING: pretrigger sample count reset to 0.\n");
  }
  memset(control, 0, sizeof(control)); // Clear existing data
  memset(address, 0, sizeof(address));
  memset(data, 0, sizeof(data));
}

void
help_pretrigger(void)
{
  tla_printf("usage: pretrigger         - show current pretrigger sample count\n");
  tla_printf("       pretrigger <count> - set pretrigger sample count\n");
  tla_printf("\n<count> must be between 0 and the numnber of samples (currently %d).\n",
      samples);
  tla_printf("\nType \"help samples\" for more information.\n");
}

void
command_pretrigger(void)
{
  if (argc == 1) {
    show_pretrigger();
    return;
  } else if (argc != 2) {
    help_pretrigger();
    return;
  }

  int c;
  c = (int) strtol(argv[1], NULL, 10);
  if (c < 0 || c > samples) {
    tla_printf("Invalid pretrigger samples count.\n");
    help_pretrigger();
    return;
  }

  if (triggerMode == tr_none) {
    tla_printf("Cannot have pretrigger samples with trigger mode 'none'.\n");
    pretrigger = 0;
    return;
  }

  pretrigger = c;
}

const struct {
  const char *typestr;
  trigger_t   type;
  uint32_t    forcpus;
  uint32_t    notcpus;
} triggertab[] = {
  { "address",  tr_address },
  { "data",     tr_data },
  { "reset",    tr_reset },
  { "irq",      tr_irq,
                0,
                (1U << cpu_z80) },
  { "int",      tr_irq,
                (1U << cpu_z80),
                0 },
  { "firq",     tr_firq,
                (1U << cpu_6809) | (1U << cpu_6809e),
                0 },
  { "nmi",      tr_nmi },
  { "manual",   tr_manual },
  { "none",     tr_none },
  { NULL },
};

void
help_trigger(void)
{
  tla_printf("usage: trigger %s                                  - show current trigger\n",
      cpu_has_iospace(cpu) ? "     " : "");
  tla_printf("       trigger %saddress <addr> [r|w]              - trigger on address\n",
      cpu_has_iospace(cpu) ? "[io] " : "");
  tla_printf("       trigger %sdata <value> [r|w]                - trigger on data\n",
      cpu_has_iospace(cpu) ? "[io] " : "");
  tla_printf("       trigger %saddress <addr> data <value> [r|w] - trigger on address and data\n",
      cpu_has_iospace(cpu) ? "[io] " : "");
  tla_printf("\n       trigger reset 0|1%s                         - trigger on /RESET level\n",
      cpu_has_iospace(cpu) ? "     " : "");
  if (cpu == cpu_z80) {
    tla_printf("       trigger int 0|1%s                           - trigger on /INT level\n",
        cpu_has_iospace(cpu) ? "     " : "");
  } else if (cpu != cpu_none) {
    tla_printf("       trigger irq 0|1%s                           - trigger on /IRQ level\n",
        cpu_has_iospace(cpu) ? "     " : "");
  }
  if (cpu == cpu_6809 || cpu == cpu_6809e) {
    tla_printf("       trigger firq 0|1%s                          - trigger on /FIRQ level\n",
        cpu_has_iospace(cpu) ? "     " : "");
  }
  tla_printf("       trigger nmi 0|1%s                           - trigger on /NMI level\n",
      cpu_has_iospace(cpu) ? "     " : "");

  if (cpu_has_iospace(cpu)) {
    tla_printf("\n<addr> must be between 0 and FF for I/O space and 0 and FFFF for memory space.\n");
  } else {
    tla_printf("\n<addr> must be between 0 and FFFF.\n");
  }
  tla_printf("<data> must be between 0 and FF.\n");
}

void
command_trigger(void)
{
  if (argc == 1) {
    show_trigger();
    return;
  }

  int i, argidx = 1, modeidx;
  uint32_t cpumask;
  bool iomodifier = false;

  cpumask = (cpu == cpu_none) ? 0 : (1U << cpu);

  // First, the trigger type.
 mode_again:
  for (modeidx = -1, i = 0; triggertab[i].typestr != NULL; i++) {
    // Special case for CPUs with I/O space -- check for "io" modifier.
    if (cpu_has_iospace(cpu) && strcmp(argv[argidx], "io") == 0) {
      if (iomodifier) {
        help_trigger();
        return;
      }
      iomodifier = true;
      argidx++;
      goto mode_again;
    }
    if (triggertab[i].forcpus != 0 || triggertab[i].notcpus != 0) {
      // If there's a CPU filter, we need to have the CPU type set.
      if (cpumask == 0) {
        continue;
      }
      if (triggertab[i].notcpus != 0 &&
          (triggertab[i].notcpus & cpumask) != 0) {
        continue;
      }
      if (triggertab[i].forcpus != 0 &&
          (triggertab[i].forcpus & cpumask) == 0) {
        continue;
      }
    }
    if (stringMatch(triggertab[i].typestr, argv[argidx]) > 0) {
      if (modeidx != -1) {
        tla_printf("Ambiguous trigger mode.");
        help_trigger();
        return;
      }
      modeidx = i;
    }
  }

  trigger_t new_triggerMode;
  if (modeidx == -1) {
    // If we got the iomodifier, allow it as an alias for "address".
    if (iomodifier) {
      new_triggerMode = tr_address;
      // Walk back the argument cursor to point at "io".
      argidx--;
    } else {
      tla_printf("Invalid trigger mode.\n");
      help_trigger();
      return;
    }
  } else {
    new_triggerMode = triggertab[modeidx].type;
  }

  cycle_t new_triggerCycle = triggerCycle;
  space_t new_triggerSpace = triggerSpace;
  bool new_triggerLevel = triggerLevel;
  uint32_t new_triggerAddress = triggerAddress;
  uint32_t new_triggerData = triggerData;

  argidx++;

  if (iomodifier && (new_triggerMode != tr_address &&
                     new_triggerMode != tr_data)) {
    tla_printf("Invalid trigger mode for \"io\" modifier.\n");
    help_trigger();
    return;
  }

  switch (new_triggerMode) {
    case tr_none:
      if (pretrigger != 0) {
        tla_printf("Warning: pretrigger reset to 0.\n");
        pretrigger = 0;
      }
      // FALLTHROUGH
    case tr_manual:
      if (argidx != argc) {
        help_trigger();
        return;
      }
      break;

    case tr_address:
    case tr_data: {
      //
      // We accept lines like this:
      //
      //  t io addr 0x42 data 0xff
      //  t data $a5 address $cafe w
      //  t a FFFFh r
      //
      // We arrive with our argument index pointing:
      //
      //  t data $a5 address $cafe w
      //         ^^^
      //         here.
      //
      // Because we have to handle both keywords, we're
      // going to move our argidx back one and just parse
      // the whole directive again in a loop.  We redundantly
      // compare the first keyword, but oh well.  We do
      // already have the new trigger mode stashed away, and
      // we detect if we got both qualifiers and set the
      // mode accordingly.
      //
      bool got_address = false;
      bool got_data = false;
      bool got_cycle = false;

      new_triggerSpace = iomodifier ? tr_io : tr_mem;
      new_triggerCycle = tr_either;

      // Must at least have first numeric argument.
      if (argidx == argc) {
        help_trigger();
        return;
      }
      argidx--;

      while (argidx != argc) {
        if (!got_address && (stringMatch("address", argv[argidx]) > 0 ||
            (iomodifier && strcmp("io", argv[argidx]) == 0))) {
          got_address = true;
          argidx++;
          if (argidx == argc) {
            help_trigger();
            return;
          }
          if (! parseAddress(argv[argidx++], new_triggerSpace, &new_triggerAddress)) {
            help_trigger();
            return;
          }
          continue;
        }
        if (!got_data && stringMatch("data", argv[argidx]) > 0) {
          got_data = true;
          argidx++;
          if (argidx == argc) {
            help_trigger();
            return;
          }
          if (! parseHexNumber(argv[argidx++], &new_triggerData)) {
            help_trigger();
            return;
          }
          if (new_triggerData > 0xff) {
            tla_printf("Invalid data value.\n");
            help_trigger();
            return;
          }
          continue;
        }
        if (!got_cycle) {
          if (stringMatch("read", argv[argidx]) > 0) {
            got_cycle = true;
            new_triggerCycle = tr_read;
            argidx++;
            continue;
          } else if (stringMatch("write", argv[argidx]) > 0) {
            got_cycle = true;
            new_triggerCycle = tr_write;
            argidx++;
            continue;
          }
        }
        help_trigger();
        return;
      }
      if (got_data && got_address) {
        new_triggerMode = tr_addr_data;
      }
      break;
    }

    case tr_reset:
    case tr_irq:
    case tr_firq:
    case tr_nmi:
      // All the rest need a level indicator, and only a level indicator.
      if (argidx + 1 != argc) {
        tla_printf("Missing level indicator.\n");
        help_trigger();
        return;
      }
      if (strcmp(argv[argidx], "1") == 0 ||
          stringMatch("high", argv[argidx]) > 0) {
        new_triggerLevel = true;
      } else if (strcmp(argv[argidx], "0") == 0 ||
                 stringMatch("low", argv[argidx]) > 0) {
        new_triggerLevel = false;
      } else {
        tla_printf("Invalid level indicator.\n");
        help_trigger();
        return;
      }
      break;

    case tr_addr_data:
    default:
      tla_printf("*** INTERNAL ERROR: unxpected trigger mode %d ***\n", (int)new_triggerMode);
      return;
  }

  // Everthing thing is good -- commit the changes.
  triggerMode = new_triggerMode;
  triggerCycle = new_triggerCycle;
  triggerSpace = new_triggerSpace;
  triggerLevel = new_triggerLevel;
  triggerAddress = new_triggerAddress;
  triggerData = new_triggerData;
}

void
help_go(void)
{
  tla_printf("usage: go - start the analyzer\n");
}

void
command_go(void)
{
  if (argc != 1) {
    help_go();
    return;
  }
  go();
}

void
help_list(void)
{
  tla_printf("usage: list [<start> [<end>]] - list samples\n");
  tla_printf("\n<start> must be between 0 and the number of samples - 1 (curretly %d).\n",
      samples - 1);
  tla_printf("<end> must be between <start> and the number of samples - 1.\n");
  tla_printf("\nType \"help samples\" for more information.\n");
}

void
command_list(void)
{
  int start = 0;
  int end = samples - 1;
  int n;

  if (argc > 1) {
    if (!parseDecimalNumber(argv[1], &n)) {
      tla_printf("Invalid <start>.\n");
      help_list();
      return;
    }
    start = n;
  }
  if (argc > 2) {
    if (!parseDecimalNumber(argv[2], &n)) {
      tla_printf("Invalid <end>.\n");
      help_list();
      return;
    }
    end = n;
  }
  if (argc > 3) {
    help_list();
    return;
  }
  if (start < 0 || start >= samples || end < start || end >= samples) {
    tla_printf("Invalid samples range: must be between 0 and %d.\n", samples - 1);
    return;
  }
  list(Serial, start, end, samplesTaken);
}

void
help_export(void)
{
  tla_printf("usage: export - export samples in CSV format\n");
}

void
command_export(void)
{
  if (argc != 1) {
    help_export();
    return;
  }
  exportCSV(Serial, samplesTaken);
}

void
help_write(void)
{
  tla_printf("usage: write - write data to SD card\n");
  tla_printf("\nThe file \"analyzer.csv\" will contain the sample data in CSV format.\n");
  tla_printf("The file \"analyzer.txt\" will contain the sample data in \"list\" format.\n");
}

void
command_write(void)
{
  if (argc != 1) {
    help_write();
    return;
  }
  writeSD();
}

void
help_decode(void)
{
  tla_printf("usage: decode <addr> - decode a single instruction at <addr>\n");
  tla_printf("\n<addr> must be between 0 and FFFF and must be present in the sample data.\n");
}

void
command_decode(void)
{
  if (argc != 2) {
    help_decode();
    return;
  }
  uint32_t pc;
  if (parseAddress(argv[1], tr_mem, &pc)) {
    disassemble_one(pc);
  } else {
    help_decode();
  }
}

#ifdef DEBUG_SAMPLES
void
command_loadtest(void)
{
  samples = samplesTaken = sizeof(debug_data) / sizeof(debug_data[0]);
  cpu = DEBUG_CPU;
  memcpy(data, debug_data, sizeof(debug_data));
  memcpy(address, debug_address, sizeof(debug_address));
  memcpy(control, debug_control, sizeof(debug_control));
#ifdef DEBUG_TRIGGER_POINT
  triggerPoint = DEBUG_TRIGGER_POINT;
  pretrigger = DEBUG_TRIGGER_POINT;
#endif  // DEBUG_TRIGGER_POINT
}
#endif // DEBUG_SAMPLES

void  command_help(void);           // forward decl for table

const struct tla_command {
  const char *cmdstr;
  void (*cmdfunc)(void);
  void (*helpfunc)(void);
  const char *summary;
} cmdtab[] = {
  { "cpu",        command_cpu,        help_cpu,         "Set CPU type" },
  { "samples",    command_samples,    help_samples,     "Set number of samples" },
  { "pretrigger", command_pretrigger, help_pretrigger,  "Set pre-trigger samples" },
  { "trigger",    command_trigger,    help_trigger,     "Set trigger mode" },
  { "go",         command_go,         help_go,          "Go - start analyzer" },
  { "list",       command_list,       help_list,        "List samples" },
  { "export",     command_export,     help_export,      "Export samples as CSV" },
  { "write",      command_write,      help_write,       "Write data to SD card" },
  { "decode",     command_decode,     help_decode,      "Decode instruction" },
#ifdef DEBUG_SAMPLES
  { "loadtest",   command_loadtest,   NULL,             "Load test samples" },
#endif
  { "help",       command_help,       NULL,             "Show help" },
  { "?",          command_help,       NULL },

  { NULL },
};

const struct tla_command *
lookupCommand(const char *cp, const struct tla_command *from)
{
  if (from == NULL) {
    from = cmdtab;
  }

  for (; from->cmdstr != NULL; from++) {
    if (stringMatch(from->cmdstr, cp) > 0) {
      return from;
    }
  }
  return NULL;
}

void
command_help(void)
{
  int i;

  // We're pretty forviging with the "help" command.
  // If we get garbage here, we just go to the generic
  // help message.

  if (argc > 1) {
    const struct tla_command * const cmd = lookupCommand(argv[1], NULL);
    if (cmd != NULL && cmd->helpfunc != NULL) {
      (*cmd->helpfunc)();
      return;
    }
  }

  show_version(true);
  show_cpu();
  show_trigger();
  show_samples();
  show_pretrigger();

  tla_printf("Commands:\n");
  for (i = 0; cmdtab[i].cmdstr != NULL; i++) {
    if (cmdtab[i].summary == NULL) {
      continue;
    }
    tla_printf("%-16s - %s\n", cmdtab[i].cmdstr, cmdtab[i].summary);
  }
  tla_printf("\nType \"help <command>\" for additional information.\n");
}

bool
tokenizeCommand(void)
{
  char *cp;
  int i;

  argc = 0;

  for (cp = cmdbuf, i = 0; *cp != '\0';) {
    // consume leading whitespace.
    while (isspace(*cp)) {
      cp++;
    }
    if (*cp == '\0') {
      goto done;
    }

    // Stash this token, advance to the next whitespace, and
    // terminate it.
    if (i == MAX_ARGS) {
      return false;
    }
    argv[i++] = cp;
    while (!isspace(*cp)) {
      if (*cp == '\0') {
        goto done;
      }
      cp++;
    }
    *cp++ = '\0';
  }

 done:
  argc = i;
  return true;
}

void
invalidCommand(void)
{
  tla_printf("Invalid command: '%s'\n", saved_cmdbuf);
}

void
loop(void)
{
  const struct tla_command *cmd, *foundcmd;
  unsigned int ci;

  while (true) {
    Serial.print("% "); // Command prompt
    Serial.flush();

    memset(cmdbuf, 0, sizeof(cmdbuf));
    ci = 0;

    while (true) {
      int c = Serial.read();
      if ((c == '\r') || (c == '\n')) {
        // End of command line.
        cmdbuf[ci] = '\0';
        break;
      }

      if ((c == '\b') || (c == 0x7f)) { // Handle backspace or delete
        if (ci > 0) {
          ci--;                  // Remove last character
          Serial.print("\b \b"); // Backspace over last character entered.
          continue;
        }
      }
      if (c != -1 && ci <= CMDBUF_LEN - 1) {
        Serial.write((char)c);  // Echo character
        cmdbuf[ci++] = (char)c; // Append to command string
      }
    }

    Serial.println("");
    memcpy(saved_cmdbuf, cmdbuf, sizeof(saved_cmdbuf));

    if (!tokenizeCommand()) {
      invalidCommand();
      continue;
    }

    if (argc == 0) {
      continue;
    }

    foundcmd = NULL;
    if ((cmd = lookupCommand(argv[0], NULL)) != NULL) {
      if (foundcmd == NULL) {
        foundcmd = cmd;
        cmd = lookupCommand(argv[0], foundcmd + 1);
        if (cmd != NULL) {
          tla_printf("Ambiguous command: '%s'\n", saved_cmdbuf);
          continue;
        }
      }
    }
    if (foundcmd == NULL) {
      invalidCommand();
    }
    foundcmd->cmdfunc();
  }
}
