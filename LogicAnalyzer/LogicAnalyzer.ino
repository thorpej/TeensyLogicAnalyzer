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

#include "insn_decode.h"

// Maximum buffer size (in samples). Increase if needed; should be
// able to go up to at least 30,000 before running out of memory.
#define BUFFSIZE 5000

const char *versionString = "Teensy Logic Analyzer version 0.2";
const char *verboseVersionStringAdditions = " by Jason R. Thorpe <thorpej@me.com>";
const char *origVersionString = "Based on Logic Analyzer version 0.30 by Jeff Tranter <tranter@pobox.com>";

// Trigger and CPU type definitions
typedef enum { tr_address, tr_data, tr_addr_data, tr_reset, tr_irq, tr_firq, tr_nmi, tr_none } trigger_t;
typedef enum { tr_mem, tr_io } space_t;
typedef enum { tr_read, tr_write, tr_either } cycle_t;
typedef enum { cpu_none, cpu_6502, cpu_65c02, cpu_6800, cpu_6809, cpu_6809e, cpu_z80 } cpu_t;

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
cpu_t cpu = cpu_none;                 // Current CPU type
trigger_t triggerMode = tr_none;      // Type of trigger
cycle_t triggerCycle = tr_either;     // Trigger on read, write, or either
space_t triggerSpace = tr_mem;        // default to memory space
bool triggerLevel = false;            // Trigger level (false=low, true=high);
volatile bool triggerPressed = false; // Set by hardware trigger button

void
show_version(bool verbose)
{
  if (verbose) {
    Serial.print(versionString);
    Serial.println(verboseVersionStringAdditions);
    Serial.println(origVersionString);
  } else {
    Serial.println(versionString);
  }
}

//
// 6502 instruction decoding
//
// The 6502 / 65C02 don't have nearly the same addressing mode complexity as
// the 6809, so we take a different tack here; the number of bytes following
// the opcode is kept alongside the opcode, along with the rest of the formatting
// to display it:
//
// nn     read an additional byte after the opcode, replace with hex
//        representation
//
// nnnn   read an additional 2 bytes after the opcode, replace with hex
//        representation
//
// rrrr   read an additional byte after the opcode, replace with signed
//        decimal representation.  This will also cause us to calculate
//        the resolved address.
//
const char *opcodes_65c02[256] = {
  "BRK",       "ORA ($nn,X)", "?",         "?",   "TSB $nn",     "ORA $nn",     "ASL $nn",     "RMB0 $nn",
  "PHP",       "ORA #$nn",    "ASLA",      "?",   "TSB $nnnn",   "ORA $nnnn",   "ASL $nnnn",   "BBR0 $nn",
  "BPL rrrr",  "ORA ($nn),Y", "ORA ($nn)", "?",   "TRB $nn",     "ORA $nn,X",   "ASL $nn,X",   "RMB1 $nn",
  "CLC",       "ORA $nnnn,Y", "INCA",      "?",   "TRB $nnnn",   "ORA $nnnn,X", "ASL $nnnn,X", "BBR1 $nn",
  "JSR $nnnn", "AND ($nn,X)", "?",         "?",   "BIT $nn",     "AND $nn",     "ROL $nn",     "RMB2 $nn",
  "PLP",       "AND #$nn",    "ROLA",      "?",   "BIT $nnnn",   "AND $nnnn",   "ROL $nnnn",   "BBR2 $nn",
  "BMI rrrr",  "AND ($nn),Y", "AND ($nn)", "?",   "BIT $nn,X",   "AND $nn,X",   "ROL $nn,X",   "RMB3 $nn",
  "SEC",       "AND $nnnn,Y", "DECA",      "?",   "BIT $nn,X",   "AND $nnnn,X", "ROL $nnnn,X", "BBR3 $nn",
  "RTI",       "EOR ($nn,X)", "?",         "?",   "?",           "EOR $nn",     "LSR $nn",     "RMB4 $nn",
  "PHA",       "EOR #$nn",    "LSRA",      "?",   "JMP $nnnn",   "EOR $nnnn",   "LSR $nnnn",   "BBR4 $nn",
  "BVC rrrr",  "EOR ($nn),Y", "EOR ($nn)", "?",   "?",           "EOR $nn,X",   "LSR $nn,X",   "RMB5 $nn",
  "CLI",       "EOR $nnnn,Y", "PHY",       "?",   "?",           "EOR $nnnn,X", "LSR $nnnn,X", "BBR5 $nn",
  "RTS",       "ADC ($nn,X)", "?",         "?",   "STZ $nn",     "ADC $nn",     "ROR $nn",     "RMB6 $nn",
  "PLA",       "ADC #$nn",    "RORA",      "?",   "JMP ($nnnn)", "ADC $nnnn",   "ROR $nnnn",   "BBR6 $nn",
  "BVS rrrr",  "ADC ($nn),Y", "ADC ($nn)", "?",   "STZ $nn,X",   "ADC $nn,X",   "ROR $nn,X",   "RMB7 $nn",
  "SEI",       "ADC $nnnn,Y", "PLY",       "?",   "JMP ($nn,X)", "ADC $nnnn,X", "ROR $nnnn,X", "BBR7 $nn",
  "BRA rrrr",  "STA ($nn,X)", "?",         "?",   "STY $nn",     "STA $nn",     "STX $nn",     "SMB0 $nn",
  "DEY",       "BIT #$nn",    "TXA",       "?",   "STY $nnnn",   "STA $nnnn",   "STX $nnnn",   "BBS0 $nn",
  "BCC rrrr",  "STA ($nn),Y", "STA ($nn)", "?",   "STY $nn,X",   "STA $nn,X",   "STX ($nn),Y", "SMB1 $nn",
  "TYA",       "STA $nnnn,Y", "TXS",       "?",   "STZ $nn",     "STA $nnnn,X", "STZ $nn,X",   "BBS1 $nn",
  "LDY #$nn",  "LDA ($nn,X)", "LDX #$nn",  "?",   "LDY $nn",     "LDA $nnnn",   "LDX $nn",     "SMB2 $nn",
  "TAY",       "LDA #$nn",    "TAX",       "?",   "LDY $nnnn",   "LDA $nnnn",   "LDX $nnnn",   "BBS2 $nn",
  "BCS rrrr",  "LDA ($nn),Y", "LDA ($nn)", "?",   "LDY $nn,X",   "LDA $nn,X",   "LDX ($nn),Y", "SMB3 $nn",
  "CLV",       "LDA $nnnn,Y", "TSX",       "?",   "LDY $nnnn,X", "LDA $nnnn,X", "LDX $nnnn,Y", "BBS3 $nn",
  "CPY #$nn",  "CMP ($nn,X)", "?",         "?",   "CPY $nnnn",   "CMP $nnnn",   "DEC $nnnn",   "SMB4 $nn",
  "INY",       "CMP #$nn",    "DEX",       "WAI", "CPY $nn",     "CMP $nn",     "DEC $nn",     "BBS4 $nn",
  "BNE rrrr",  "CMP ($nn),Y", "CMP ($nn)", "?",   "?",           "CMP $nn,X",   "DEC $nn,X",   "SMB5 $nn",
  "CLD",       "CMP $nnnn,Y", "PHX",       "STP", "?",           "CMP $nnnn,X", "DEC $nnnn,X", "BBS5 $nn",
  "CPX #$nn",  "SBC ($nn,X)", "?",         "?",   "CPX $nn",     "SBC $nn",     "INC $nn",     "SMB6 $nn",
  "INX",       "SBC #$nn",    "NOP",       "?",   "CPX $nnnn",   "SBC $nnnn",   "INC $nnnn",   "BBS6 $nn",
  "BEQ rrrr",  "SBC ($nn),Y", "SBC ($nn)", "?",   "?",           "SBC $nn,X",   "INC $nn,X",   "SMB7 $nn",
  "SED",       "SBC $nnnn,Y", "PLX",       "?",   "?",           "SBC $nnnn,X", "INC $nnnn,X", "BBS7 $nn"
};

const char *opcodes_6502[256] = {
  "BRK",       "ORA ($nn,X)", "?",        "?", "?",           "ORA $nn",     "ASL $nn",     "?",
  "PHP",       "ORA #$nn",    "ASLA",     "?", "?",           "ORA $nnnn",   "ASL $nnnn",   "?",
  "BPL rrrr",  "ORA ($nn),Y", "?",        "?", "?",           "ORA $nn,X",   "ASL $nn,X",   "?",
  "CLC",       "ORA $nnnn,Y", "?",        "?", "?",           "ORA $nnnn,X", "ASL $nnnn,X", "?",
  "JSR $nnnn", "AND ($nn,X)", "?",        "?", "BIT $nn",     "AND $nn",     "ROL $nn",     "?",
  "PLP",       "AND #$nn",    "ROLA",     "?", "BIT $nnnn",   "AND $nnnn",   "ROL $nnnn",   "?",
  "BMI rrrr",  "AND ($nn),Y", "?",        "?", "?",           "AND $nn,X",   "ROL $nn,X",   "?",
  "SEC",       "AND $nnnn,Y", "?",        "?", "?",           "AND $nnnn,X", "ROL $nnnn,X", "?",
  "RTI",       "EOR ($nn,X)", "?",        "?", "?",           "EOR nn",      "LSR $nn",     "?",
  "PHA",       "EOR #$nn",    "LSRA",     "?", "JMP $nnnn",   "EOR $nnnn",   "LSR $nnnn",   "?",
  "BVC rrrr",  "EOR ($nn),Y", "?",        "?", "?",           "EOR $nn,X",   "LSR $nn,X",   "?",
  "CLI",       "EOR $nnnn,Y", "?",        "?", "?",           "EOR $nnnn,X", "LSR $nnnn,X", "?",
  "RTS",       "ADC ($nn,X)", "?",        "?", "?",           "ADC $nn",     "ROR $nn",     "?",
  "PLA",       "ADC #$nn",    "RORA",     "?", "JMP ($nnnn)", "ADC $nnnn",   "ROR $nnnn",   "?",
  "BVS rrrr",  "ADC ($nn),Y", "?",        "?", "?",           "ADC $nn,X",   "ROR $nn,X",   "?",
  "SEI",       "ADC $nnnn,Y", "?",        "?", "?",           "ADC $nnnn,X", "ROR $nnnn,X", "?",
  "?",         "STA ($nn,X)", "?",        "?", "STY $nn",     "STA $nn",     "STX $nn",     "?",
  "DEY",       "?",           "TXA",      "?", "STY $nnnn",   "STA $nnnn",   "STX $nnnn",   "?",
  "BCC rrrr",  "STA ($nn),Y", "?",        "?", "STY $nn,X",   "STA $nn,X",   "STX $nn,Y",   "?",
  "TYA",       "STA $nnnn,Y", "TXS",      "?", "?",           "STA $nnnn,X", "?",           "?",
  "LDY #$nn",  "LDA ($nn,X)", "LDX #$nn", "?", "LDY $nn",     "LDA $nn",     "LDX $nn",     "?",
  "TAY",       "LDA #$nn",    "TAX",      "?", "LDY $nnnn",   "LDA $nnnn",   "LDX $nnnn",   "?",
  "BCS rrrr",  "LDA ($nn),Y", "?",        "?", "LDY $nn,X",   "LDA $nn,X",   "LDX $nn,Y",   "?",
  "CLV",       "LDA $nnnn,Y", "TSX",      "?", "LDY $nnnn,X", "LDA $nnnn,X", "LDX $nnnn,Y", "?",
  "CPY #$nn",  "CMP ($nn,X)", "?",        "?", "CPY $nn",     "CMP $nn",     "DEC $nn",     "?",
  "INY",       "CMP #$nn",    "DEX",      "?", "CPY $nnnn",   "CMP $nnnn",   "DEC $nnnn",   "?",
  "BNE rrrr",  "CMP ($nn),Y", "?",        "?", "?",           "CMP $nn,X",   "DEC $nn,X",   "?",
  "CLD",       "CMP $nnnn,Y", "?",        "?", "?",           "CMP $nnnn,X", "DEC $nnnn,X", "?",
  "CPX #$nn",  "SBC ($nn,X)", "?",        "?", "CPX $nn",     "SBC $nn",     "INC $nn",     "?",
  "INX",       "SBC #$nn",    "NOP",      "?", "CPX $nnnn",   "SBC $nnnn",   "INC $nnnn",   "?",
  "BEQ rrrr",  "SBC ($nn),Y", "?",        "?", "?",           "SBC $nn,X",   "INC $nn,X",   "?",
  "SED",       "SBC $nnnn,Y", "?",        "?", "?",           "SBC $nnnn,X", "INC $nnnn,X", "?"
};

void
insn_decode_format_6502(struct insn_decode *id)
{
  const char **opcodes = (cpu == cpu_65c02) ? opcodes_65c02 : opcodes_6502;
  char *cp, op[5];
  int16_t val;

  strcpy(id->insn_string, opcodes[id->bytes[0]]);

  switch (id->addrmode) {
    case am6502_u8:
      val = id->bytes[1];
      if ((cp = strstr(id->insn_string, "nn")) != NULL) {
        sprintf(op, "%02X", val);
        memcpy(cp, op, 2);
      }
      break;

    case am6502_u16:
      val = read_u16le(id->bytes, 1);
      if ((cp = strstr(id->insn_string, "nnnn")) != NULL) {
        sprintf(op, "%04X", val);
        memcpy(cp, op, 4);
      }
      break;

    case am6502_rel8:
      val = (int8_t)id->bytes[1];   // sign-extend
      if ((cp = strstr(id->insn_string, "rrrr")) != NULL) {
        sprintf(op, "%-4d", val);
        memcpy(cp, op, 4);
        id->resolved_address = id->insn_address + id->bytes_required + val;
        id->resolved_address_valid = true;
      }

    default:
      break;
  }
}

void
insn_decode_next_state_6502(struct insn_decode *id)
{
  
  if (id->state != ds_fetching || id->bytes_fetched == 0) {
    return;
  }

  if (id->bytes_required == 0) {
    const char **opcodes = (cpu == cpu_65c02) ? opcodes_65c02 : opcodes_6502;
    const char *cp1 = opcodes[id->bytes[0]];

    if (strstr(cp1, "nnnn") != NULL) {
      id->addrmode = am6502_u16;
      id->bytes_required = 3;
    } else if (strstr(cp1, "nn") != NULL) {
      id->addrmode = am6502_u8;
      id->bytes_required = 2;
    } else if (strstr(cp1, "rrrr") != NULL) {
      id->addrmode = am6502_rel8;
      id->bytes_required = 2;
    } else {
      id->addrmode = am6502_implied;
      id->bytes_required = 1;
    }
  }

  // If we've now fetched the number of required bytes, we can
  // fully decode and format the instruction.
  if (id->bytes_fetched == id->bytes_required) {
    insn_decode_format_6502(id);
    id->state = ds_complete;
  }
}

//
// 6800 instruction decoding
//
const char *opcodes_6800[256] = {
  "?",    "NOP",  "?",    "?",    "?",    "?",    "TAP",  "TPA",
  "INX",  "DEX",  "CLV",  "SEV",  "CLC",  "SEC",  "CLI",  "SEI",
  "SBA",  "CBA",  "?",    "?",    "?",    "?",    "TAB",  "TBA",
  "?",    "DAA",  "?",    "ABA",  "?",    "?",    "?",    "?",
  "BRA",  "?",    "BHI",  "BLS",  "BCC",  "BCS",  "BNE",  "BEQ",
  "BVC",  "BVS",  "BPL",  "BMI",  "BGE",  "BLT",  "BGT",  "BLE",
  "TSX",  "INS",  "PULA", "PULB", "DES",  "TXS",  "PSHA", "PSHB",
  "?",    "RTS",  "?",    "RTI",  "?",    "?",    "WAI",  "SWI",
  "NEGA", "?",    "?",    "COMA", "LSRA", "?",    "RORA", "ASRA",
  "ASLA", "ROLA", "DECA", "?",    "INCA", "TSTA", "?",    "CLRA",
  "NEGB", "?",    "?",    "COMB", "LSRB", "?",    "RORB", "ASRB",
  "ASLB", "ROLB", "DECB", "?",    "INCB", "TSTB", "?",    "CLRB",
  "NEG",  "?",    "?",    "COM",  "LSR",  "?",    "ROR",  "ASR",
  "ASL",  "ROL",  "DEC",  "?",    "INC",  "TST",  "JMP",  "CLR",
  "NEG",  "?",    "?",    "COM",  "LSR",  "?",    "ROR",  "ASR",
  "ASL",  "ROL",  "DEC",  "?",    "INC",  "TST",  "JMP",  "CLR",
  "SUBA", "CMPA", "SBCA", "?",    "ANDA", "BITA", "LDAA", "?",
  "EORA", "ADCA", "ORAA", "ADDA", "CPX",  "BSR",  "LDS",  "?",
  "SUBA", "CMPA", "SBCA", "?",    "ANDA", "BITA", "LDAA", "STAA",
  "EORA", "ADCA", "ORAA", "ADDA", "CPX",  "?",    "LDS",  "STS",
  "SUBA", "CMPA", "SBCA", "?",    "ANDA", "BITA", "LDAA", "STAA",
  "EORA", "ADCA", "ORAA", "ADDA", "CPX",  "JSR",  "LDS",  "STS",
  "SUBA", "CMPA", "SBCA", "?",    "ANDA", "BITA", "LDAA", "STAA",
  "EORA", "ADCA", "ORAA", "ADDA", "CPX",  "JSR",  "LDS",  "STS",
  "SUBB", "CMPB", "SBCB", "?",    "ANDB", "BITB", "LDAB", "?",
  "EORB", "ADCB", "ORAB", "ADDB", "?",    "?",    "LDX",  "?",
  "SUBB", "CMPB", "SBCB", "?",    "ANDB", "BITB", "LDAB", "STAB",
  "EORB", "ADCB", "ORAB", "ADDB", "?",    "?",    "LDX",  "STX",
  "SUBB", "CMPB", "SBCB", "?",    "ANDB", "BITB", "LDAB", "STAB",
  "EORB", "ADCB", "ORAB", "ADDB", "?",    "?",    "LDX",  "STX",
  "SUBB", "CMPB", "SBCB", "?",    "ANDB", "BITB", "LDAB", "STAB",
  "EORB", "ADCB", "ORAB", "ADDB", "?",    "?",    "LDX",  "STX"
};

addrmode_t
insn_decode_addrmode_6800(struct insn_decode *id)
{
  // Refer to "TABLE 1 - HEXADECIMAL VALUES OF MACHINE CODES" in the 6800 data sheet.
  // We do incomplete decoding here such that we may return a valid addressing mode
  // for an invalid opcode.  The hardware also does incomplete decoding, although not
  // necessarily the same incomplete decoding we do here.

  if (id->bytes_fetched == 0) {
      return am_invalid;
  }

  uint32_t opc = id->bytes[0];

  if ((opc >= 0 && opc <= 0x1f) ||
      (opc >= 0x30 && opc <= 0x5f)) {
    return am6800_inherent;
  }

  if (opc >= 0x20 && opc <= 0x2f) {
    return am6800_rel;
  }

  if ((opc >= 0x60 && opc <= 0x6f) ||
      (opc >= 0xa0 && opc <= 0xaf) ||
      (opc >= 0xe0 && opc <= 0xef)) {
    return am6800_indexed;
  }

  if ((opc >= 0x70 && opc <= 0x7f) ||
      (opc >= 0xb0 && opc <= 0xbf) ||
      (opc >= 0xf0 && opc <= 0xff)) {
    return am6800_extended;
  }

  if ((opc >= 0x80 && opc <= 0x8f) ||
      (opc >= 0xc0 && opc <= 0xcf)) {
    // Special case for BSR.
    if (opc == 0x8d) {
      return am6800_rel;
    }
    // Special case for LDS, LDX.
    if (opc == 0x8e || opc == 0xce) {
      return am6800_imm16;
    }
    return am6800_imm8;
  }

  if ((opc >= 0x90 && opc <= 0x9f) ||
      (opc >= 0xd0 && opc <= 0xdf)) {
    return am6800_direct;
  }

  return am_invalid;
}

void
insn_decode_format_6800(struct insn_decode *id)
{
  const char *opc = opcodes_6800[id->bytes[0]];

  switch (id->addrmode) {
    case am6800_inherent:
      sprintf(id->insn_string, "%s", opc);
      break;

    case am6800_rel:
      sprintf(id->insn_string, "%s %d", opc, (int8_t)id->bytes[1]); // sign-extend
      id->resolved_address = id->insn_address + 2 + (int8_t)id->bytes[1];
      id->resolved_address_valid = true;
      break;

    case am6800_indexed:
      sprintf(id->insn_string, "%s %u,X", opc, id->bytes[1]);
      break;

    case am6800_extended:
      sprintf(id->insn_string, "%s $%04X", opc, read_u16be(id->bytes, 1));
      break;

    case am6800_direct:
      sprintf(id->insn_string, "%s $%02X", opc, id->bytes[1]);
      break;

    case am6800_imm8:
      sprintf(id->insn_string, "%s #$%02X", opc, id->bytes[1]);
      break;

    case am6800_imm16:
      sprintf(id->insn_string, "%s #$%04X", opc, read_u16be(id->bytes, 1));
      break;

    default:
      sprintf(id->insn_string, "<?ADDRMODE?>");
      break;
  }
}

void
insn_decode_next_state_6800(struct insn_decode *id)
{

  if (id->state != ds_fetching || id->bytes_fetched == 0) {
    return;
  }

  if (id->bytes_required == 0) {
    id->addrmode = insn_decode_addrmode_6800(id);

    // We know the full size after the opcode fetch.
    switch (id->addrmode) {
      case am6800_inherent:
        id->bytes_required = 1;
        break;

      case am6800_rel:
      case am6800_indexed:
      case am6800_imm8:
      case am6800_direct:
        id->bytes_required = 2;
        break;

      case am6800_extended:
      case am6800_imm16:
        id->bytes_required = 3;
        break;

      default:
        // Should never happen.
        break;
    }
  }

  // If we've now fetched the number of required bytes, we can
  // fully decode and format the instruction.
  if (id->bytes_fetched == id->bytes_required) {
    insn_decode_format_6800(id);
    id->state = ds_complete;
  }
}

//
// Z80 instruction decoding
//
const char *opcodes_z80[256] = {
  "NOP",          "LD BC,XXXXh",  "LD (BC),A",    "INC BC",       "INC B",        "DEC B",        "LD B,XXh",     "RLCA",
  "EX AF,AF'",    "ADD HL,BC",    "LD A,(BC)",    "DEC BC",       "INC C",        "DEC C",        "LD C,XXh",     "RRCA",
  "DJNZ rrrr",    "LD DE,XXXXh",  "LD (DE),A",    "INC DE",       "INC D",        "DEC D",        "LD D,XXh",     "RLA",
  "JR rrrr",      "ADD HL,DE",    "LD A,(DE)",    "DEC DE",       "INC E",        "DEC E",        "LD E,XXh",     "RRA",
  "JR NZ,rrrr",   "LD HL,XXXXh",  "LD (XXXXh),HL","INC HL",       "INC H",        "DEC H",        "LD H,XXh",     "DAA",
  "JR Z,rrrr",    "ADD HL,HL",    "LD HL,(XXXXh)","DEC HL",       "INC L",        "DEC L",        "LD L,XXh",     "CPL",
  "JR NC,rrrr",   "LD SP,XXXXh",  "LD (XXXXh),A", "INC SP",       "INC (HL)",     "DEC (HL)",     "LD (HL),XXh",  "SCF",
  "JR C,rrrr",    "ADD HL,SP",    "LD A,(XXXXh)", "DEC SP",       "INC A",        "DEC A",        "LD A,XXh",     "CCF",
  "LD B,B",       "LD B,C",       "LD B,D",       "LD B,E",       "LD B,H",       "LD B,L",       "LD B,(HL)",    "LD B,A",
  "LD C,B",       "LD C,C",       "LD C,D",       "LD C,E",       "LD C,H",       "LD C,L",       "LD C,(HL)",    "LD C,A",
  "LD D,B",       "LD D,C",       "LD D,D",       "LD D,E",       "LD D,H",       "LD D,L",       "LD D,(HL)",    "LD D,A",
  "LD E,B",       "LD E,C",       "LD E,D",       "LD E,E",       "LD E,H",       "LD E,L",       "LD E,(HL)",    "LD E,A",
  "LD H,B",       "LD H,C",       "LD H,D",       "LD H,E",       "LD H,H",       "LD H,L",       "LD H,(HL)",    "LD H,A",
  "LD L,B",       "LD L,C",       "LD L,D",       "LD L,E",       "LD L,H",       "LD L,L",       "LD L,(HL)",    "LD L,A",
  "LD (HL),B",    "LD (HL),C",    "LD (HL),D",    "LD (HL),E",    "LD (HL),H",    "LD (HL),L",    "HALT",         "LD (HL),A",
  "LD A,B",       "LD A,C",       "LD A,D",       "LD A,E",       "LD A,H",       "LD A,L",       "LD A,(HL)",    "LD A,A",
  "ADD B",        "ADD C",        "ADD D",        "ADD E",        "ADD H",        "ADD L",        "ADD (HL)",     "ADD A",
  "ADC B",        "ADC C",        "ADC D",        "ADC E",        "ADC H",        "ADC L",        "ADC (HL)",     "ADC A",
  "SUB B",        "SUB C",        "SUB D",        "SUB E",        "SUB H",        "SUB L",        "SUB (HL)",     "SUB A",
  "SBC B",        "SBC C",        "SBC D",        "SBC E",        "SBC H",        "SBC L",        "SBC (HL)",     "SBC A",
  "AND B",        "AND C",        "AND D",        "AND E",        "AND H",        "AND L",        "AND (HL)",     "AND A",
  "XOR B",        "XOR C",        "XOR D",        "XOR E",        "XOR H",        "XOR L",        "XOR (HL)",     "XOR A",
  "OR B",         "OR C",         "OR D",         "OR E",         "OR H",         "OR L",         "OR (HL)",      "OR A",
  "CP B",         "CP C",         "CP D",         "CP E",         "CP H",         "CP L",         "CP (HL)",      "CP A",
  "RET NZ",       "POP BC",       "JP NZ,XXXXh",  "JP XXXXh",     "CALL NZ,XXXXh","PUSH BC",      "ADD XXh",      "RST 00h",
  "RET Z",        "RET",          "JP Z,XXXXh",   "extCB",        "CALL Z,XXXXh", "CALL XXXXh",   "ADC XXh",      "RST 08h",
  "RET NC",       "POP DE",       "JP NC,XXXXh",  "OUT (XXh),A",  "CALL NC,XXXXh","PUSH DE",      "SUB XXh",      "RST 10h",
  "RET C",        "EXX",          "JP C,XXXXh",   "IN A,(XXh)",   "CALL C,XXXXh", "extDD",        "SBC XXh",      "RST 18h",
  "RET PO",       "POP HL",       "JP PO,XXXXh",  "EX (SP),HL",   "CALL PO,XXXXh","PUSH HL",      "AND XXh",      "RST 20h",
  "RET PE",       "JP (HL)",      "JP PE,XXXXh",  "EX DE,HL",     "CALL PE,XXXXh","extED",        "XOR XXh",      "RST 28h",
  "RET P",        "POP AF",       "JP P,XXXXh",   "DI",           "CALL P,XXXXh", "PUSH AF",      "OR XXh",       "RST 30h",
  "RET M",        "LD SP,HL",     "JP M,XXXXh",   "EI",           "CALL M,XXXXh", "extFD",        "CP XXh",       "RST 38h",
};

const struct {
  const char *opr_string;
  addrmode_t opr_mode;
} z80_operand_types[] = {
    { "XXXXh",    amz80_u16,     },
    { "XXh",      amz80_u8,      },
    { "+ddd",     amz80_disp8,   },
    { "rrrr",     amz80_pcrel8,  },
    { NULL,       am_invalid,    }, // not really invalid, just "none" or "no more"
};

int
z80_operand_size(addrmode_t mode)
{
  switch (mode) {
    case amz80_u8:
    case amz80_disp8:
    case amz80_pcrel8:
      return 1;

    case amz80_u16:
      return 2;

    default:
      return 0;
  }
}

char *
z80_next_operand(addrmode_t *modep, char **cursor)
{
  char *cp;
  size_t oplen;
  int i;

  for (cp = *cursor; *cp != '\0'; cp++) {
    for (i = 0; z80_operand_types[i].opr_string != NULL; i++) {
      oplen = strlen(z80_operand_types[i].opr_string);
      if (memcmp(cp, z80_operand_types[i].opr_string, oplen) == 0) {
        // Found one!  Advance the cursor beyond it.
        *cursor = cp + oplen;
        if (modep != NULL) {
          *modep = z80_operand_types[i].opr_mode;
        }
        return cp;
      }
    }
  }
  // No more operand substitutions found.
  return NULL;
}

// Substitute a reference to HL in the instruction template
// with a reference to IX or IY as indicated by first opcode
// byte.
bool
z80_hl_to_index(struct insn_decode *id, const char *tmpl, uint8_t opc, uint8_t which)
{
  const char *t1 = tmpl;
  char *cp = id->insn_string;
  bool need_disp = false;
  char c;

  // Copy up to the HL reference.
  for (;;) {
    if (t1[0] == 'H' && t1[1] == 'L') {
      break;
    }
    c = (*cp++ = *t1++);
    if (c == '\0') {
      // No HL to substitute.
      return false;
    }
    if (c == '(' && t1[0] == 'H' && t1[1] == 'L') {
      // Memory reference -- we need a displacement in this case.
      // Except for JP (HL), which is defined as "PC <- HL" and
      // NOT "PC <- (HL)", and thus is, I think, the only irregular
      // Z80 instruction syntax.
      if (opc != 0xe9) {
        need_disp = true;
      }
    }
  }

  // t1 now points at HL; skip past it in the template.
  t1 += 2;

  // Now insert the substition string.
  *cp++ = 'I';
  *cp++ = which == 0xdd ? 'X' : 'Y';
  if (need_disp) {
    *cp++ = '+';
    *cp++ = 'd';
    *cp++ = 'd';
    *cp++ = 'd';
  }

  // Copy the rest of the template.
  for (;;) {
    if ((*cp++ = *t1++) == '\0') {
      break;
    }
  }
  return true;
}

bool
z80_insn_template(struct insn_decode *id)
{                                                       // 6 is special; see CB group
  static const char *ld_regs[8] = { "B", "C", "D", "E", "H", "L", "(HL)", "A" };
  static const char *io_regs[8] = { "B", "C", "D", "E", "H", "L", "?", "A" };
  static const char *ld_regs16[4] = { "BC", "DE", "HL", "SP" };
  char tbuf[INSN_DECODE_MAXSTRING];
  uint8_t opc = id->bytes[0];
  uint8_t reg16, ioreg;

  if ((opc == 0xdd || opc == 0xfd) &&
      // CB sub-group is handled below.
      id->bytes_fetched >= 2 && id->bytes[1] != 0xcb) {
    //
    // Groups DD and FD are all about substituting Ir for HL or
    // (Ir+d) for (HL).  We get the base instruction from the
    // second opcode byte.  Note that we are doing incomplete
    // decoding here; if the base instruction has no HL for us
    // to substitute, we'll simply carry on and decode it as the
    // base instruction.  I don't know if this is what real Z80s
    // do or not.
    //
    const char *tmpl = tbuf;
    opc = id->bytes[1];
    if ((opc & 0b11001111) == 0b00001001) {
      sprintf(tbuf, "ADD I%c,%s", id->bytes[0] == 0xdd ? 'X' : 'Y',
          ld_regs16[(opc >> 4) & 3]);
    } else {
      tmpl = opcodes_z80[opc];
    }
    z80_hl_to_index(id, tmpl, opc, id->bytes[0]);
    // We should now have all the info we need to calculate the
    // required number of instruction bytes.
    return true;
  }

  if (opc == 0xed) {
    // This group is a handful of instruction additions.
    if (id->bytes_fetched < 2) {
      return false;
    }
    opc = id->bytes[1];
    reg16 = (opc >> 4) & 3;
    ioreg = (opc >> 3) & 7;
    if ((opc & 0b11001111) == 0b01001011) {
      sprintf(id->insn_string, "LD %s,(XXXXh)", ld_regs16[reg16]);
    } else if ((opc & 0b11001111) == 0b01000011) {
      sprintf(id->insn_string, "LD (XXXXh),%s", ld_regs16[reg16]);
    } else if ((opc & 0b11001111) == 0b01001010) {
      sprintf(id->insn_string, "ADC HL,%s", ld_regs16[reg16]);
    } else if ((opc & 0b11001111) == 0b01000010) {
      sprintf(id->insn_string, "SBC HL,%s", ld_regs16[reg16]);
    } else if ((opc & 0b11000111) == 0b01000000) {
      sprintf(id->insn_string, "IN %s,(C)", ioreg == 6 ? "Flags" : io_regs[ioreg]);
    } else if ((opc & 0b11000111) == 0b01000001) {
      sprintf(id->insn_string, "OUT (C),%s", io_regs[ioreg]);
    } else {
      const char *cp;
      switch (opc) {
        case 0x57:  cp = "LD A,I";    break;
        case 0x5f:  cp = "LD A,R";    break;
        case 0x47:  cp = "LD I,A";    break;
        case 0x4f:  cp = "LD R,A";    break;
        case 0xa0:  cp = "LDI";       break;
        case 0xb0:  cp = "LDIR";      break;
        case 0xa8:  cp = "LDD";       break;
        case 0xb8:  cp = "LDDR";      break;
        case 0xa1:  cp = "CPI";       break;
        case 0xb1:  cp = "CPIR";      break;
        case 0xa9:  cp = "CPD";       break;
        case 0xb9:  cp = "CPDR";      break;
        case 0x44:  cp = "NEG";       break;
        case 0x46:  cp = "IM 0";      break;
        case 0x56:  cp = "IM 1";      break;
        case 0x5e:  cp = "IM 2";      break;
        case 0x6f:  cp = "RLD";       break;
        case 0x67:  cp = "RRD";       break;
        case 0x4d:  cp = "RETI";      break;
        case 0x45:  cp = "RETN";      break;
        case 0xa2:  cp = "INI";       break;
        case 0xb2:  cp = "INIR";      break;
        case 0xaa:  cp = "IND";       break;
        case 0xba:  cp = "INDR";      break;
        case 0xa3:  cp = "OUTI";      break;
        case 0xb3:  cp = "OUTIR";     break;
        case 0xab:  cp = "OUTD";      break;
        case 0xbb:  cp = "OTDR";      break;
        default:    cp = "?";         break;
      }
      strcpy(id->insn_string, cp);
    }
    return true;
  }

  if (opc == 0xcb ||
      ((opc == 0xdd || opc == 0xfd) && id->bytes_fetched >= 2 && id->bytes[1] == 0xcb)) {
    // This group is a handful of instruction additions.  We also handle
    // the DD and FD group subsitutions for these instructions.
    static const char *opcodes_CB[32] = {
      "RLC ",   "RRC ",   "RL ",    "RR ",    "SLA ",   "SRA ",   "? ",     "SRL ",
      "BIT 0,", "BIT 1,", "BIT 2,", "BIT 3,", "BIT 4,", "BIT 5,", "BIT 6,", "BIT 7,",
      "RES 0,", "RES 1,", "RES 2,", "RES 3,", "RES 4,", "RES 5,", "RES 6,", "RES 7,",
      "SET 0,", "SET 1,", "SET 2,", "SET 3,", "SET 4,", "SET 5,", "SET 6,", "SET 7,",
    };
    if (opc == 0xcb) {
      if (id->bytes_fetched < 2) {
        return false;
      }
      opc = id->bytes[1];
    } else {
      if (id->bytes_fetched < 4) {
        return false;
      }
      opc = id->bytes[3];
    }
    sprintf(tbuf, "%s%s", opcodes_CB[(opc >> 3) & 0x1f], ld_regs[opc & 7]);

    if (id->bytes[0] == 0xdd || id->bytes[0] == 0xfd) {
      z80_hl_to_index(id, tbuf, opc, id->bytes[0]);
    } else {
      strcpy(id->insn_string, tbuf);
    }
    return true;
  }

  if (opc == 0xcb || opc == 0xdd || opc == 0xed || opc == 0xfd) {
    return false;
  }

  strcpy(id->insn_string, opcodes_z80[opc]);
  return true;
}

void
insn_decode_format_z80(struct insn_decode *id)
{
  //
  // All of the heavy lifting has been done for us already, in building
  // up the insn template that's stashed away in id->insn_string.  All
  // we need to do now is enumerate the operands and substitute the
  // values into the template with the specified format.
  //
  // There is one pair of instructions that has 2 operands in the
  // instruction stream: LD (Ir+d),XXXXh.  This is the indexed
  // addressing mode of LD (HL),XXXXh.  Conveniently, because of
  // the way the Z80 itself handles the substitution, the operands
  // appear in the instruction stream in the same left-to-right
  // order that we humans read them, so no special-casing is
  // necessary.
  //
  int opr_byte = 1;
  if (id->bytes[0] == 0xcb || id->bytes[0] == 0xed) {
    opr_byte++;
  } else if (id->bytes[0] == 0xdd || id->bytes[0] == 0xfd) {
    opr_byte++;
    // Don't advance for the CB subgroup here because, in that case,
    // the remainder of the opcode is AFTER the displacement operand.
  }

  // opr_byte now points to the first operand byte in the instruction buffer.
  char *curs, *cp;
  addrmode_t mode;
  uint16_t u16;
  int8_t s8;
  char op[8];
  for (curs = id->insn_string; (cp = z80_next_operand(&mode, &curs)) != NULL;) {
    switch (mode) {
      case amz80_u16:
        u16 = read_u16le(id->bytes, opr_byte);
        sprintf(op, "%04X", u16);
        memcpy(cp, op, 4);
        break;

      case amz80_u8:
        sprintf(op, "%02X", id->bytes[opr_byte]);
        memcpy(cp, op, 2);
        break;

      case amz80_pcrel8:
        // No punctuation after PC-relative offsets.  Also, note that the
        // value stored in the instruction stream is atually "target - 2".
        // Z80 assemblers are expected to make the adjustment, so we do the
        // same for display purposes.
        s8 = (int8_t)id->bytes[opr_byte] + 2; // sign-extend
        sprintf(op, "%-4d", s8);
        memcpy(cp, op, 4);
        id->resolved_address = id->insn_address + s8;
        id->resolved_address_valid = true;
        break;

      case amz80_disp8:
        //
        // This one is a little annoying.  We save it in the template as
        // "+ddd".  But, for a negative displacement (it's defined to be a
        // twos-complement number), we really want to display it as "-ddd".
        // Furthermore, we don't want to display extra digits, but there's
        // always punctuation (the closing ")") after the displacement value.
        //
        s8 = (int8_t)id->bytes[opr_byte];
        if (s8 < 0) {
          sprintf(op, "%-4d", s8);
          memcpy(cp, op, 4);
        } else {
          sprintf(op, "%-3d", s8);
          memcpy(cp + 1, op, 3);
        }
        for (; *cp != ' ' && *cp != '\0'; cp++) {
          // advance to the whitespace.
        }
        if (*cp == ' ') {
          char *ncp;
          for (ncp = cp; *ncp == ' '; ncp++) {
            // advance past the whitespace
          }
          while ((*cp++ = *ncp++) != '\0') {
            // collapse the whitespace.
          }
        }
        break;

      default:
        break;
    }
    opr_byte += z80_operand_size(mode);
  }
}

void
insn_decode_next_state_z80(struct insn_decode *id)
{
  if (id->state != ds_fetching || id->bytes_fetched == 0) {
    return;
  }

  if (id->bytes_required == 0) {
    // Try to get the insn template.  If we can't, we probably just
    // need to fetch another byte or two.
    if (! z80_insn_template(id)) {
      return;
    }

    //
    // OK, we have the insn template, and therefore know all of the
    // operand types.  We can now calculate how many bytes are required
    // to decode the entire instruction.
    //
    // First, account for the opcode bytes.
    //
    id->bytes_required = 1;
    if (id->bytes[0] == 0xcb || id->bytes[0] == 0xed) {
      id->bytes_required++;
    } else if (id->bytes[0] == 0xdd || id->bytes[0] == 0xfd) {
      id->bytes_required++;
      if (id->bytes_fetched >= 2 && id->bytes[1] == 0xcb) {
        id->bytes_required++;
      }
    }

    // Now add up the bytes for the operands.
    char *curs, *cp;
    addrmode_t mode;
    for (curs = id->insn_string; (cp = z80_next_operand(&mode, &curs)) != NULL;) {
      id->bytes_required += z80_operand_size(mode);
    }
  }

  // If we've now fetched the number of required bytes, we can
  // fully decode and format the instruction.
  if (id->bytes_fetched == id->bytes_required) {
    insn_decode_format_z80(id);
    id->state = ds_complete;
  }
}

//
// 6809 instruction decoding
//
#define POSTBYTES(am, n) [(am) - am6809_first] = (n)
const uint8_t
insn_postbytes_6809[] = {
  POSTBYTES(am6809_inherent,        0),
  POSTBYTES(am6809_direct,          1),
  POSTBYTES(am6809_extended,        2),
  POSTBYTES(am6809_rel8,            1),
  POSTBYTES(am6809_rel16,           2),
  POSTBYTES(am6809_imm8,            1),
  POSTBYTES(am6809_imm16,           2),
  POSTBYTES(am6809_zero_off,        1),
  POSTBYTES(am6809_zero_off_ind,    1),
  POSTBYTES(am6809_const_off5,      1),
  POSTBYTES(am6809_const_off8,      2),
  POSTBYTES(am6809_const_off8_ind,  2),
  POSTBYTES(am6809_const_off16,     3),
  POSTBYTES(am6809_const_off16_ind, 3),
  POSTBYTES(am6809_acc_off,         1),
  POSTBYTES(am6809_acc_off_ind,     1),
  POSTBYTES(am6809_post_inc1,       1),
  POSTBYTES(am6809_post_inc2,       1),
  POSTBYTES(am6809_post_inc2_ind,   1),
  POSTBYTES(am6809_pre_dec1,        1),
  POSTBYTES(am6809_pre_dec2,        1),
  POSTBYTES(am6809_pre_dec2_ind,    1),
  POSTBYTES(am6809_pcrel8,          2),
  POSTBYTES(am6809_pcrel8_ind,      2),
  POSTBYTES(am6809_pcrel16,         3),
  POSTBYTES(am6809_pcrel16_ind,     3),
  POSTBYTES(am6809_extended_ind,    3),
  POSTBYTES(am6809_exg_tfr,         1),
  POSTBYTES(am6809_psh_pul,         1),
};
#undef POSTBYTES

const char *opcodes_6809[256] = {
  "NEG",  "?",    "?",    "COM",  "LSR",  "?",    "ROR",  "ASR",
  "ASL",  "ROL",  "DEC",  "?",    "INC",  "TST",  "JMP",  "CLR",
  "(pg2)","(pg3)","NOP",  "SYNC", "?",    "?",    "LBRA", "LBSR",
  "?",    "DAA",  "ORCC", "?",    "ANDCC","SEX",  "EXG",  "TFR",
  "BRA",  "BRN",  "BHI",  "BLS",  "BCC",  "BCS",  "BNE",  "BEQ",
  "BVC",  "BVS",  "BPL",  "BMI",  "BGE",  "BLT",  "BGT",  "BLE",
  "LEAX", "LEAY", "LEAS", "LEAU", "PSHS", "PULS", "PSHU", "PULU",
  "?",    "RTS",  "ABX",  "RTI",  "CWAI", "MUL",  "?",    "SWI",
  "NEGA", "?",    "?",    "COMA", "LSRA", "?",    "RORA", "ASRA",
  "ASLA", "ROLA", "DECA", "?",    "INCA", "TSTA", "?",    "CLRA",
  "NEGB", "?",    "?",    "COMB", "LSRB", "?",    "RORB", "ASRB",
  "ASLB", "ROLB", "DECB", "?",    "INCB", "TSTB", "?",    "CLRB",
  "NEG",  "?",    "?",    "COM",  "LSR",  "?",    "ROR",  "ASR",
  "ASL",  "ROL",  "DEC",  "?",    "INC",  "TST",  "JMP",  "CLR",
  "NEG",  "?",    "?",    "COM",  "LSR",  "?",    "ROR",  "ASR",
  "ASL",  "ROL",  "DEC",  "?",    "INC",  "TST",  "JMP",  "CLR",
  "SUBA", "CMPA", "SBCA", "SUBD", "ANDA", "BITA", "LDA",  "?",
  "EORA", "ADCA", "ORA",  "ADDA", "CMPX", "BSR",  "LDX",  "?",
  "SUBA", "CMPA", "SBCA", "SUBD", "ANDA", "BITA", "LDA",  "STA",
  "EORA", "ADCA", "ORA",  "ADDA", "CMPX", "JSR",  "LDX",  "STX",
  "SUBA", "CMPA", "SBCA", "SUBD", "ANDA", "BITA", "LDA",  "STA",
  "EORA", "ADCA", "ORA",  "ADDA", "CMPX", "JSR",  "LDX",  "STX",
  "SUBA", "CMPA", "SBCA", "SUBD", "ANDA", "BITA", "LDA",  "STA",
  "EORA", "ADCA", "ORA",  "ADDA", "CMPX", "JSR",  "LDX",  "STX",
  "SUBB", "CMPB", "SBCB", "ADDD", "ANDB", "BITB", "LDB",  "?",
  "EORB", "ADCB", "ORB",  "ADDB", "LDD",  "?",    "LDU",  "?",
  "SUBB", "CMPB", "SBCB", "ADDD", "ANDB", "BITB", "LDB",  "STB",
  "EORB", "ADCB", "ORB",  "ADDB", "LDD",  "STD",  "LDU",  "STU",
  "SUBB", "CMPB", "SBCB", "ADDD", "ANDB", "BITB", "LDB",  "STB",
  "EORB", "ADCB", "ORB",  "ADDB", "LDD",  "STD",  "LDU",  "STU",
  "SUBB", "CMPB", "SBCB", "ADDD", "ANDB", "BITB", "LDB",  "STB",
  "EORB", "ADCB", "ORB",  "ADDB", "LDD",  "STD",  "LDU",  "STU"
};

static const char *opcodes_long_cond_branches_6809[] = {
  "?",    "LBRN", "LBHI", "LBLS", "LBCC", "LBCS", "LBNE", "LBEQ",
  "LBVC", "LBVS", "LBPL", "LBMI", "LBGE", "LBLT", "LBGT", "LBLE"
};

addrmode_t
insn_decode_addrmode_indexed_6809(uint8_t pb)
{
  // Refer to "TABLE 2 - INDEXED ADDRESSING MODE" in the 6809 data sheet.
  addrmode_t am;

  // Extended indirect is a slightly special case.
  if ((pb & 0b10011111) == 0b10011111) {
    return am6809_extended_ind;
  }

  // 5-bit constant offset also is a special case.
  if ((pb & 0b10000000) == 0) {
    return am6809_const_off5;
  }

  switch (pb & 0b10001111) {
    case 0b10000100:
      am = am6809_zero_off;
      break;

    case 0b10001000:
      am = am6809_const_off8;
      break;

    case 0b10001001:
      am = am6809_const_off16;
      break;

    case 0b10000110:
    case 0b10000101:
    case 0b10001011:
      am = am6809_acc_off;
      break;

    case 0b10000000:
      am = am6809_post_inc1;
      break;

    case 0b10000001:
      am = am6809_post_inc2;
      break;

    case 0b10000010:
      am = am6809_pre_dec1;
      break;

    case 0b10000011:
      am = am6809_pre_dec2;
      break;

    case 0b10001100:
      am = am6809_pcrel8;
      break;

    case 0b10001101:
      am = am6809_pcrel16;
      break;

    default:
      return am_invalid;
  }

  if (pb & 0b00010000) {
    // Indirect flag.
    if (am == am6809_post_inc1 || am == am6809_pre_dec1) {
      // Indirect not allowed in this case.
      return am_invalid;
    }
    am = (addrmode_t)((int)am + 1);
  }

  return am;
}

addrmode_t
insn_decode_addrmode_6809(struct insn_decode *id)
{
  // Refer to "TABLE 9 - HEXADECIMAL VALUES OF MACHINE CODES" in the 6809 data sheet.
  // We do incomplete decoding here such that we may return a valid addressing mode
  // for an invalid opcode.  The hardware also does incomplete decoding, although not
  // necessarily the same incomplete decoding we do here.

  if (id->bytes_fetched == 0) {
      return am_invalid;
  }

  uint32_t opc = id->bytes[0];

  // Check for Page 2 / Page 3 opcodes.
  if (opc == 0x10 || opc == 0x11) {
    if (id->bytes_fetched < 2) {
      return am_invalid;
    }

    uint32_t extopc = (opc << 8) | id->bytes[1];

    switch (extopc & 0xfff0) {
      case 0x1020:
        return am6809_rel16;

      case 0x1030:
      case 0x1130:
        return am6809_inherent;

      case 0x1080:
      case 0x1180:
      case 0x10c0:
        return am6809_imm16;

      case 0x1090:
      case 0x1190:
      case 0x10d0:
        return am6809_direct;

      case 0x10a0:
      case 0x11a0:
      case 0x10e0:
        // Indexed; need 3 bytes for this.
        if (id->bytes_fetched < 3) {
          return am_invalid;
        }
        return insn_decode_addrmode_indexed_6809(id->bytes[2]);

      case 0x10b0:
      case 0x11b0:
      case 0x10f0:
        return am6809_extended;

      default:
        return am_invalid;
    }
  }

  if ((opc >= 0x00 && opc <= 0x0f) ||
      (opc >= 0x90 && opc <= 0x9f) ||
      (opc >= 0xd0 && opc <= 0xdf)) {
    return am6809_direct;
  }

  if (opc >= 0x10 && opc <= 0x1f) {
    // This one's a bunch of special cases.
    switch (opc) {
      case 0x12:                              // NOP
      case 0x13:                              // SYNC
      case 0x19:                              // DAA
      case 0x1d:                              // SEX
        return am6809_inherent;

      case 0x16:                              // LBRA
      case 0x17:                              // LBSR
        return am6809_rel16;

      case 0x1a:                              // ORCC
      case 0x1c:                              // ANDCC
        return am6809_imm8;

      case 0x1e:                              // EXG
      case 0x1f:                              // TFR
        return am6809_exg_tfr;

      default:
        return am_invalid;
    }
  }

  if (opc >= 0x20 && opc <= 0x2f) {
    return am6809_rel8;
  }

  if (opc >= 0x30 && opc <= 0x3f) {
    if (opc >= 0x30 && opc <= 0x33) {
      goto indexed_need2;
    } else if (opc >= 0x34 && opc <= 0x37) {
      return am6809_psh_pul;
    } else if (opc >= 0x39 && opc <= 0x3f) {
      return am6809_inherent;
    } else {
      return am_invalid;
    }
  }

  if ((opc >= 0x40 && opc <= 0x4f) ||
      (opc >= 0x50 && opc <= 0x5f)) {
    return am6809_inherent;
  }

  if ((opc >= 0x60 && opc <= 0x6f) ||
      (opc >= 0xa0 && opc <= 0xaf) ||
      (opc >= 0xe0 && opc <= 0xef)) {
 indexed_need2:
    // Indexed; need 2 bytes for this.
    if (id->bytes_fetched < 2) {
      return am_invalid;
    }
    return insn_decode_addrmode_indexed_6809(id->bytes[1]);
  }

  if ((opc >= 0x70 && opc <= 0x7f) ||
      (opc >= 0xb0 && opc <= 0xbf) ||
      (opc >= 0xf0 && opc <= 0xff)) {
    return am6809_extended;
  }

  if ((opc >= 0x80 && opc <= 0x8f) ||
      (opc >= 0xc0 && opc <= 0xcf)) {
    if (opc == 0x8d) {
      return am6809_rel8; 
    }
    opc &= 0xf;
    if (opc == 0x3 || opc == 0xc || opc == 0xe) {
      return am6809_imm16;
    } else {
      return am6809_imm8;
    }
  }
  return am_invalid;
}

const char *
insn_decode_format_exg_tfr_regname_6809(uint8_t v)
{
  switch (v) {
    case 0b0000:  return "D";
    case 0b0001:  return "X";
    case 0b0010:  return "Y";
    case 0b0011:  return "U";
    case 0b0100:  return "S";
    case 0b0101:  return "PC";
    case 0b1000:  return "A";
    case 0b1001:  return "B";
    case 0b1010:  return "CCR";
    case 0b1011:  return "DPR";
    default:      return "?";
  }
}

void
insn_decode_format_6809(struct insn_decode *id)
{
  const char *opc, *cp1, *cp2;
  char *bp;
  int i = 1;
  int j;

  if (id->bytes[0] == 0x10 || id->bytes[0] == 0x11) {
    uint32_t extopc = (id->bytes[0] << 8) | id->bytes[1];
    uint32_t extopc_u3 = extopc & 0xfff0;
    uint32_t extopc_b1 = extopc & 0x000f;
    i++;

    // Default to "unknown".
    opc = "?";

    if (extopc >= 0x1020 && extopc <= 0x102f) {
      opc = opcodes_long_cond_branches_6809[extopc & 0xf];
    } else if (extopc == 0x103f) {
      opc = "SWI2";
    } else if (extopc == 0x113f) {
      opc = "SWI3";
    } else if (extopc_u3 == 0x1080 || extopc_u3 == 0x1090 ||
               extopc_u3 == 0x10a0 || extopc_u3 == 0x10b0) {
      if (extopc == 0x108f) {
        // special case (no STY #IMM)
      } else if (extopc_b1 == 0x3) {
        opc = "CMPD";
      } else if (extopc_b1 == 0xc) {
        opc = "CMPY";
      } else if (extopc_b1 == 0xe) {
        opc = "LDY";
      } else if (extopc_b1 == 0xf) {
        opc = "STY";
      }
    } else if (extopc_u3 == 0x10c0 || extopc_u3 == 0x10d0 ||
               extopc_u3 == 0x10e0 || extopc_u3 == 0x10f0) {
      if (extopc == 0x10cf) {
        // special case (no STS #IMM)
      } else if (extopc_b1 == 0xe) {
        opc = "LDS";
      } else if (extopc_b1 == 0xf) {
        opc = "STS";
      }
    } else if (extopc_u3 == 0x1180 || extopc_u3 == 0x1190 ||
               extopc_u3 == 0x11a0 || extopc_u3 == 0x11b0) {
      if (extopc_b1 == 0x3) {
        opc = "CMPU";
      } else if (extopc_b1 == 0xc) {
        opc = "CMPS";
      }
    }
   } else {
    opc = opcodes_6809[id->bytes[0]];
  }

  if (id->addrmode < am6809_first || id->addrmode > am6809_last) {
 unknown_addrmode:
    sprintf(id->insn_string, "<?ADDRMODE?>");
    return;
  }

  uint16_t u16 = 0;
  int16_t s16 = 0;
  uint8_t index_reg = (id->bytes[i] >> 5) & 3;
  uint8_t reg1 = id->bytes[i] >> 4;
  uint8_t reg2 = id->bytes[i] & 0xf;
  int16_t reloff = 0;
  const char *ind_open;
  const char *ind_close;

  if (am6809_indirect_p(id->addrmode)) {
    ind_open = "[";
    ind_close = "]";
  } else {
    ind_open = ind_close = "";
  }

  static const char *index_regnames[] = { "X", "Y", "U", "S" };
  static const struct {
    uint8_t bit;
    const char *reg;
  } psh_pul_regnames[] = {
    { 0b00000001, "CCR" },
    { 0b00000010, "A" },
    { 0b00000100, "B" },
    { 0b00001000, "DPR" },
    { 0b00010000, "X" },
    { 0b00100000, "Y" },
    { 0b01000000, NULL },
    { 0b10000000, "PC" },
    { 0,          NULL },
  };

  // i now points to the first operand byte.
  switch (id->addrmode) {
    case am6809_inherent:
      sprintf(id->insn_string, "%s", opc);
      break;

    case am6809_direct:
      sprintf(id->insn_string, "%s < $%02X", opc, id->bytes[i]);
      break;

    case am6809_extended_ind:
      // This is really an indexed mode; skip the index postbyte.
      i++;
      // FALLTHROUGH
    case am6809_extended:
      u16 = read_u16be(id->bytes, i);
      sprintf(id->insn_string, "%s %s$%04X%s", opc, ind_open, u16, ind_close);
      break;

    case am6809_rel8:
      s16 = (int8_t)id->bytes[i]; // sign-extend;
      goto rel;

    case am6809_rel16:
      s16 = read_s16be(id->bytes, i);
    rel:
      reloff = s16;
      sprintf(id->insn_string, "%s %d", opc, s16);
      break;

    case am6809_imm8:
      sprintf(id->insn_string, "%s #$%02X", opc, id->bytes[i]);
      break;

    case am6809_imm16:
      u16 = read_u16be(id->bytes, i);
      sprintf(id->insn_string, "%s #$%04X", opc, u16);
      break;

    case am6809_zero_off:
    case am6809_zero_off_ind:
      sprintf(id->insn_string, "%s %s,%s%s", opc, ind_open,
              index_regnames[index_reg], ind_close);
      break;

    case am6809_const_off5:
      s16 = id->bytes[i] & 0x1f;
      s16 = (s16 << 11) >> 11;    // sign-extend and discard extra bits
      goto const_off;

    case am6809_const_off8:
    case am6809_const_off8_ind:
      s16 = (int8_t)id->bytes[i + 1]; // sign-extend
      goto const_off;

    case am6809_const_off16:
    case am6809_const_off16_ind:
      s16 = read_s16be(id->bytes, i + i);
    const_off:
      sprintf(id->insn_string, "%s %s%d,%s%s", opc, ind_open,
          (int)s16, index_regnames[index_reg], ind_close);
      break;

    case am6809_acc_off:
    case am6809_acc_off_ind:
      switch (id->bytes[i] & 0b1111) {
        case 0b0110: cp1 = "A"; break;
        case 0b0101: cp1 = "B"; break;
        case 0b1011: cp1 = "D"; break;
        default:     cp1 = "?"; break;
      }
      sprintf(id->insn_string, "%s %s%s,%s%s", opc, ind_open,
              cp1, index_regnames[index_reg], ind_close);
      break;

    case am6809_post_inc1:
      sprintf(id->insn_string, "%s ,%s+", opc, index_regnames[index_reg]);
      break;

    case am6809_post_inc2:
    case am6809_post_inc2_ind:
      sprintf(id->insn_string, "%s %s,%s++%s", opc, ind_open,
              index_regnames[index_reg], ind_close);
      break;

    case am6809_pre_dec1:
      sprintf(id->insn_string, "%s ,-%s", opc, index_regnames[index_reg]);
      break;

    case am6809_pre_dec2:
    case am6809_pre_dec2_ind:
      sprintf(id->insn_string, "%s %s,--%s%s", opc, ind_open,
              index_regnames[index_reg], ind_close);
      break;

    case am6809_pcrel8:
    case am6809_pcrel8_ind:
      s16 = (int8_t)id->bytes[i + i]; // sign-extend;
      goto pcrel;

    case am6809_pcrel16:
    case am6809_pcrel16_ind:
      s16 = read_s16be(id->bytes, i + 1);
    pcrel:
      reloff = s16;
      sprintf(id->insn_string, "%s %s%d,PCR%s", opc, ind_open,
          (int)s16, ind_close);
      break;

    case am6809_exg_tfr:
      cp1 = insn_decode_format_exg_tfr_regname_6809(reg1);
      cp2 = insn_decode_format_exg_tfr_regname_6809(reg2);
      sprintf(id->insn_string, "%s %s,%s", opc, cp1, cp2);
      break;

    case am6809_psh_pul:
      bp = &id->insn_string[sprintf(id->insn_string, "%s ", opc)];
      for (j = 0; psh_pul_regnames[j].bit != 0; j++) {
        if (id->bytes[i] & psh_pul_regnames[j].bit) {
          if ((cp2 = psh_pul_regnames[j].reg) == NULL) {
            // special case for U/S
            if (id->bytes[0] == 0x34 || id->bytes[0] == 0x35) {
                cp2 = "U";
            } else {
                cp2 = "S";
            }
          }
          cp1 = (id->bytes[i] & (psh_pul_regnames[j].bit - 1)) ? "," : "";
          bp += sprintf(bp, "%s%s", cp1, cp2);
        }
      }
      break;

    default:
      goto unknown_addrmode;
  }

  switch (id->addrmode) {
    case am6809_rel8:
    case am6809_rel16:
    case am6809_pcrel8:
    case am6809_pcrel8_ind:
    case am6809_pcrel16:
    case am6809_pcrel16_ind:
      id->resolved_address = id->insn_address + reloff;
      id->resolved_address_valid = true;
      break;

    default:
      break;
  }
}

void
insn_decode_next_state_6809(struct insn_decode *id)
{
  if (id->state != ds_fetching || id->bytes_fetched == 0) {
    return;
  }

  if (id->bytes_required == 0) {
    // Check for Page 2 and Page 3 opcodes.  We'll have to fetch
    // an additional byte before we can determine the addressing mode.
    if (id->bytes_fetched == 1 &&
        (id->bytes[0] == 0x10 || id->bytes[0] == 0x11)) {
      return;
    }

    // We now can try to determine the addressing mode.  It might take
    // multiple passes, since extended opcodes can have indexed modes,
    // and that would require fetching a third byte.  Once we have the
    // addressing mode, we'll know the total number of bytes that will
    // be required to fully decode the instruction.
    id->addrmode = insn_decode_addrmode_6809(id);
    if (id->addrmode >= am6809_first && id->addrmode <= am6809_last) {
      id->bytes_required = 1 + insn_postbytes_6809[id->addrmode - am6809_first];
      if (id->bytes[0] == 0x10 || id->bytes[0] == 0x11) {
        id->bytes_required++;
      }
    }
  }

  // If we've now fetched the number of required bytes, we can
  // fully decode and format the instruction.
  if (id->bytes_fetched == id->bytes_required) {
    insn_decode_format_6809(id);
    id->state = ds_complete;
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
  Serial.println("Type h or ? for help.");
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
    case cpu_6502:
      return "6502";
      break;
    case cpu_65c02:
      return "65C02";
      break;
    case cpu_6800:
      return "6800";
      break;
    case cpu_6809:
      return "6809";
      break;
    case cpu_6809e:
      return "6809E";
      break;
    case cpu_z80:
      return "Z80";
      break;
    default:
      return "not set";
  }
}

void
show_cpu(void)
{
  Serial.print("CPU: ");
  Serial.println(cpu_name());
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

void
show_trigger(void)
{
  Serial.print("Trigger: ");
  switch (triggerMode) {
    case tr_address:
      if (triggerSpace == tr_io) {
        Serial.print("on io ");
      } else {
        Serial.print("on address ");
      }
      Serial.print(triggerAddress, HEX);
      switch (triggerCycle) {
        case tr_read:
          Serial.println(" read");
          break;
        case tr_write:
          Serial.println(" write");
          break;
        case tr_either:
          Serial.println(" read or write");
          break;
      }
      break;
    case tr_data:
      Serial.print("on data ");
      Serial.print(triggerData, HEX);
      switch (triggerCycle) {
        case tr_read:
          Serial.println(" read");
          break;
        case tr_write:
          Serial.println(" write");
          break;
        case tr_either:
          Serial.println(" read or write");
          break;
      }
      break;
    case tr_addr_data:
      if (triggerSpace == tr_io) {
        Serial.print("on io ");
      } else {
        Serial.print("on address ");
      }
      Serial.print(triggerAddress, HEX);
      Serial.print(" and data ");
      Serial.print(triggerData, HEX);
      switch (triggerCycle) {
        case tr_read:
          Serial.println(" read");
          break;
        case tr_write:
          Serial.println(" write");
          break;
        case tr_either:
          Serial.println(" read or write");
          break;
      }
      break;
    case tr_reset:
      Serial.print("on /RESET ");
      Serial.println(triggerLevel ? "high" : "low");
      break;
    case tr_irq:
      if (cpu == cpu_z80) {
        Serial.print("on /INT ");
      } else {
        Serial.print("on /IRQ ");
      }
      Serial.println(triggerLevel ? "high" : "low");
      break;
    case tr_firq:
      if (cpu == cpu_6809 || cpu == cpu_6809e) {
        Serial.print("on /FIRQ ");
        Serial.println(triggerLevel ? "high" : "low");
      }
      break;
    case tr_nmi:
      Serial.print("on /NMI ");
      Serial.println(triggerLevel ? "high" : "low");
      break;
    case tr_none:
      Serial.println("none (freerun)");
      break;
  }
}

void
show_samples(void)
{
  Serial.print("Sample buffer size: ");
  Serial.println(samples);
}

void
show_pretrigger(void)
{
  Serial.print("Pretrigger samples: ");
  Serial.println(pretrigger);
}

// Display settings and help info.
void
help(void)
{
  show_version(true);
  show_cpu();
  show_trigger();
  show_samples();
  show_pretrigger();

  Serial.println("Commands:");
  Serial.println("c <cpu>                     - Set CPU.  Valid types:");
  Serial.println("                                 6502 65C02");
  Serial.println("                                 6800");
  Serial.println("                                 6809");
  Serial.println("                                 6809E");
  Serial.println("                                 Z80");
  Serial.println("c                           - Show current CPU");
  Serial.println("s <number>                  - Set number of samples");
  Serial.println("s                           - Show current number of samples");
  Serial.println("p <samples>                 - Set pre-trigger samples");
  Serial.println("p                           - Show current pre-trigger samples");
  Serial.println("t a <address> [r|w]         - Trigger on address");
  if (cpu_has_iospace(cpu)) {
    Serial.println("t i <address> [r|w]         - Trigger on i/o address");
  }
  Serial.println("t d <data> [r|w]            - Trigger on data");
  Serial.println("t ad <address> <data> [r|w] - Trigger on address and data");
  if (cpu != cpu_none) {
    Serial.println("t reset 0|1                 - Trigger on /RESET level");
    if (cpu == cpu_z80) {
      Serial.println("t int 0|1                   - Trigger on /INT level");
    } else {
      Serial.println("t irq 0|1                   - Trigger on /IRQ level");
    }
    if (cpu == cpu_6809 || cpu == cpu_6809e) {
      Serial.println("t firq 0|1                  - Trigger on /FIRQ level");
    }
    Serial.println("t nmi 0|1                   - Trigger on /NMI level");
  }
  Serial.println("t none                      - Trigger freerun");
  Serial.println("t                           - Show current trigger");
  Serial.println("g                           - Go/start analyzer");
  Serial.println("l [start] [end]             - List samples");
  Serial.println("e                           - Export samples as CSV");
  Serial.println("w                           - Write data to SD card");
  Serial.println("d <address>                 - Decode instruction at address");
#ifdef DEBUG_SAMPLES
  Serial.println("D                           - Load debug sample data");
#endif
  Serial.println("h or ?                      - Show command usage");
  if (cpu == cpu_none) {
    Serial.println("");
    Serial.println("Select a CPU type to see additional trigger options.");
  }
}

void
insn_decode_init(struct insn_decode *id)
{
  id->state = ds_idle;
  switch (cpu) {
    case cpu_6502:
    case cpu_65c02:
      id->next_state = insn_decode_next_state_6502;
      break;

    case cpu_6800:
      id->next_state = insn_decode_next_state_6800;
      break;

    case cpu_6809:
    case cpu_6809e:
      id->next_state = insn_decode_next_state_6809;
      break;

    case cpu_z80:
      id->next_state = insn_decode_next_state_z80;
      break;

    default:
      id->next_state = NULL;
  }
}

bool
insn_decode_next_state(struct insn_decode *id)
{
  decode_state_t ostate = id->state;

  if (id->next_state != NULL) {
    (*id->next_state)(id);
    if (ostate == ds_fetching) {
      if (id->state == ds_complete && id->resolved_address_valid) {
        char *cp = &id->insn_string[strlen(id->insn_string)];
        sprintf(cp, " <%04lX>", id->resolved_address);
      }
      return true;
    }
  }
  return false;
}

void
insn_decode_begin(struct insn_decode *id, uint32_t addr, uint8_t b)
{
  if (id->next_state != NULL &&
      (id->state == ds_idle || id->state == ds_complete)) {
    id->state = ds_fetching;
    id->insn_address = addr;
    id->resolved_address = 0;
    id->resolved_address_valid = false;
    id->addrmode = am_invalid;
    id->bytes_required = 0;
    id->bytes_fetched = 0;
    id->bytes[id->bytes_fetched++] = b;
    insn_decode_next_state(id);
  }
}

bool
insn_decode_continue(struct insn_decode *id, uint8_t b)
{
  bool was_fetching = false;

  if (id->state == ds_fetching) {
    if (id->bytes_fetched == INSN_DECODE_MAXBYTES) {
      strcpy(id->insn_string, "<decode overflow>");
      id->state = ds_complete;
    } else {
      id->bytes[id->bytes_fetched++] = b;
      was_fetching = insn_decode_next_state(id);
    }
  }
  return was_fetching;
}

const char *
insn_decode_complete(struct insn_decode *id)
{
  if (id->state == ds_complete) {
     return id->insn_string;
  }
  return "";
}

// Disassemble a single instrution at the specified address.
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
    Serial.println("Address not found in sample data.");
    return;
  }

  char output[50];

  for (;; i = (i + 1) % samples) {
    switch (id.state) {
      case ds_idle:
        insn_decode_begin(&id, address[i], data[i]);
        goto printit;

      case ds_fetching:
        if (address[i] != where + id.bytes_fetched) {
          Serial.println("!!!! Non-contiguous instruction fetch?");
        }
        // FALLTHROUGH

      default:
        insn_decode_continue(&id, data[i]);
      printit:
        sprintf(output, "%04lX  %02lX  %s",
            address[i], data[i], insn_decode_complete(&id));
        Serial.println(output);
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
    Serial.println("No samples to save.");
    return;
  }

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Unable to initialize internal SD card.");
    return;
  }

  // Remove any existing file
  if (SD.exists(CSV_FILE)) {
    SD.remove(CSV_FILE);
  }

  File file = SD.open(CSV_FILE, FILE_WRITE);
  if (file) {
    Serial.print("Writing ");
    Serial.println(CSV_FILE);
    exportCSV(file, samplesTaken);
    file.close();
  } else {
    Serial.print("Unable to write ");
    Serial.println(CSV_FILE);
  }

  // Remove any existing file
  if (SD.exists(TXT_FILE)) {
    SD.remove(TXT_FILE);
  }

  file = SD.open(TXT_FILE, FILE_WRITE);
  if (file) {
    Serial.print("Writing ");
    Serial.println(TXT_FILE);
    list(file, 0, samples - 1, samplesTaken);
    file.close();
  } else {
    Serial.print("Unable to write ");
    Serial.println(TXT_FILE);
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
    Serial.println("No CPU type selected!");
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

  Serial.println("Waiting for trigger...");

  triggerPressed = false; // Status of trigger button

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

  Serial.print("Data recorded (");
  Serial.print(samples);
  Serial.println(" samples).");
  unscramble();
}

void
loop(void) {
  String cmd;

  while (true) {
    Serial.print("% "); // Command prompt
    Serial.flush();

    cmd = "";
    while (true) {
      int c = Serial.read();
      if ((c == '\r') || (c == '\n')) {
        // End of command line.
        break;
      }

      if ((c == '\b') || (c == 0x7f)) { // Handle backspace or delete
        if (cmd.length() > 0) {
          cmd = cmd.remove(cmd.length() - 1); // Remove last character
          Serial.print("\b \b"); // Backspace over last character entered.
          continue;
        }
      }
      if (c != -1) {
        Serial.write((char)c); // Echo character
        cmd += (char)c; // Append to command string
      }
    }

    Serial.println("");

    // Help
    if ((cmd == "h") || (cmd == "?")) {
      help();

      //CPU
    } else if (cmd == "c") {
      show_cpu();
    } else if (cmd == "c 6502") {
      set_cpu(cpu_6502);
    } else if ((cmd == "c 65c02") || (cmd == "c 65C02")) {
      set_cpu(cpu_65c02);
    } else if (cmd == "c 6800") {
      set_cpu(cpu_6800);
    } else if (cmd == "c 6809") {
      set_cpu(cpu_6809);
    } else if ((cmd == "c 6809e") || (cmd == "c 6809E")) {
      set_cpu(cpu_6809e);
    } else if ((cmd == "c z80") || (cmd == "c Z80")) {
      set_cpu(cpu_z80);

      // Samples
    } else if (cmd == "s") {
      show_samples();
    } else if (cmd.startsWith("s ")) {
      int n = 0;
      n = cmd.substring(2).toInt();

      if ((n > 0) && (n <= BUFFSIZE)) {
        samples = n;
        memset(control, 0, sizeof(control)); // Clear existing data
        memset(address, 0, sizeof(address));
        memset(data, 0, sizeof(data));
      } else {
        Serial.print("Invalid samples, must be between 1 and ");
        Serial.print(BUFFSIZE);
        Serial.println(".");
      }

      // Pretrigger
    } else if (cmd == "p") {
      show_pretrigger();
    } else if (cmd.startsWith("p ")) {
      int n = 0;
      n = cmd.substring(2).toInt();

      if ((n >= 0) && (n <= samples)) {
        pretrigger = n;
      } else {
        Serial.print("Invalid samples, must be between 0 and ");
        Serial.print(samples);
        Serial.println(".");
      }

      // Trigger
    } else if (cmd == "t") {
      show_trigger();
    } else if (cmd == "t none") {
      triggerMode = tr_none;
    } else if (cmd == "t reset 0") {
      triggerMode = tr_reset;
      triggerLevel = false;
    } else if (cmd == "t reset 1") {
      triggerMode = tr_reset;
      triggerLevel = true;
    } else if ((cpu == cpu_z80) && (cmd == "t int 0")) {
      triggerMode = tr_irq;
      triggerLevel = false;
    } else if ((cpu == cpu_z80) && (cmd == "t int 1")) {
      triggerMode = tr_irq;
      triggerLevel = true;
    } else if ((cpu != cpu_z80) && (cmd == "t irq 0")) {
      triggerMode = tr_irq;
      triggerLevel = false;
    } else if ((cpu != cpu_z80) && (cmd == "t irq 1")) {
      triggerMode = tr_irq;
      triggerLevel = true;
    } else if ((cpu == cpu_6809 || cpu == cpu_6809e) && (cmd == "t firq 0")) {
      triggerMode = tr_firq;
      triggerLevel = false;
    } else if ((cpu == cpu_6809 || cpu == cpu_6809e) && (cmd == "t firq 1")) {
      triggerMode = tr_firq;
      triggerLevel = true;
    } else if (cmd == "t nmi 0") {
      triggerMode = tr_nmi;
      triggerLevel = false;
    } else if (cmd == "t nmi 1") {
      triggerMode = tr_nmi;
      triggerLevel = true;
    } else if (cmd.startsWith("t a ")) {
      int n = strtol(cmd.substring(4, 8).c_str(), NULL, 16);
      if ((n >= 0) && (n <= 0xffff)) {
        triggerAddress = n;
        triggerMode = tr_address;
        triggerSpace = tr_mem;
        if ((cmd.length() == 10) && cmd.endsWith('r')) {
          triggerCycle = tr_read;
        } else if ((cmd.length() == 10) && cmd.endsWith('w')) {
          triggerCycle = tr_write;
        } else {
          triggerCycle = tr_either;
        }
      } else {
        Serial.println("Invalid address, must be between 0 and FFFF.");
      }
    } else if (cmd.startsWith("t d ")) {
      int n = strtol(cmd.substring(4, 6).c_str(), NULL, 16);
      if ((n >= 0) && (n <= 0xff)) {
        triggerData = n;
        triggerMode = tr_data;
        triggerSpace = tr_mem;
        if ((cmd.length() == 8) && cmd.endsWith('r')) {
          triggerCycle = tr_read;
        } else if ((cmd.length() == 8) && cmd.endsWith('w')) {
          triggerCycle = tr_write;
        } else {
          triggerCycle = tr_either;
        }
      } else {
        Serial.println("Invalid data, must be between 0 and FF.");
      }
    } else if (cmd.startsWith("t ad ")) {
      int n = strtol(cmd.substring(5, 9).c_str(), NULL, 16);
      if ((n >= 0) && (n <= 0xffff)) {
        triggerAddress = n;
        n = strtol(cmd.substring(10, 12).c_str(), NULL, 16);
        if ((n >= 0) && (n <= 0xff)) {
          triggerData = n;
          triggerMode = tr_addr_data;
          triggerSpace = tr_mem;
          if ((cmd.length() == 14) && cmd.endsWith('r')) {
            triggerCycle = tr_read;
          } else if ((cmd.length() == 14) && cmd.endsWith('w')) {
            triggerCycle = tr_write;
          } else {
            triggerCycle = tr_either;
          }
        } else {
          Serial.println("Invalid data, must be between 0 and FF.");
        }
      } else {
        Serial.println("Invalid address, must be between 0 and FFFF.");
      }
    } else if (cmd.startsWith("t i ") && cpu_has_iospace(cpu)) {
      int n = strtol(cmd.substring(4, 6).c_str(), NULL, 16);
      if ((n >= 0) && (n <= 0xff)) {
        triggerAddress = n;
        triggerMode = tr_address;
        triggerSpace = tr_io;
        if ((cmd.length() == 8) && cmd.endsWith('r')) {
          triggerCycle = tr_read;
        } else if ((cmd.length() == 8) && cmd.endsWith('w')) {
          triggerCycle = tr_write;
        } else {
          triggerCycle = tr_either;
        }
      } else {
        Serial.println("Invalid address, must be between 0 and FF.");
      }

      // Decode instruction
    } else if (cmd.startsWith("d ")) {
      int n = strtol(cmd.substring(2, 6).c_str(), NULL, 16);
      if (n >= 0 && n <= 0xffff) {
        disassemble_one(n);
      } else {
        Serial.println("Invalid address, must be between 0 and FFFF.");
      }

      // Go
    } else if (cmd == "g") {
      go();

      // List
    } else if (cmd == "l") {
      list(Serial, 0, samples - 1, samplesTaken);
    } else if (cmd.startsWith("l ")) {
      if (cmd.indexOf(" ") == cmd.lastIndexOf(" ")) {
        // l <start>
        int start = cmd.substring(2).toInt();
        if ((start < 0) || (start >= samples)) {
          Serial.print("Invalid start, must be between 0 and ");
          Serial.print(samples - 1);
          Serial.println(".");
        } else {
          list(Serial, start, samples - 1, samplesTaken);
        }

      } else {
        // l start end
        int start = cmd.substring(2).toInt();
        int end = cmd.substring(cmd.lastIndexOf(" ")).toInt();
        if ((start < 0) || (start >= samples)) {
          Serial.print("Invalid start, must be between 0 and ");
          Serial.print(samples - 1);
          Serial.println(".");
        } else if ((end < start) || (end >= samples)) {
          Serial.print("Invalid end, must be between ");
          Serial.print(start);
          Serial.print(" and ");
          Serial.print(samples - 1);
          Serial.println(".");
        } else {
          list(Serial, start, end, samplesTaken);
        }
      }

      // Export
    } else if (cmd == "e") {
      exportCSV(Serial, samplesTaken);

      // Write
    } else if (cmd == "w") {
      writeSD();

#ifdef DEBUG_SAMPLES
      // Load debug sample data
    } else if (cmd == "D") {
      samples = samplesTaken = sizeof(debug_data) / sizeof(debug_data[0]);
      cpu = DEBUG_CPU;
      memcpy(data, debug_data, sizeof(debug_data));
      memcpy(address, debug_address, sizeof(debug_address));
      memcpy(control, debug_control, sizeof(debug_control));
#ifdef DEBUG_TRIGGER_POINT
      triggerPoint = DEBUG_TRIGGER_POINT;
      pretrigger = DEBUG_TRIGGER_POINT;
#endif  // DEBUG_TRIGGER_POINT
#endif  // DEBUG_SAMPLES
    } else {
      // Invalid command
      if (cmd != "") {
        Serial.print("Invalid command: '");
        Serial.print(cmd);
        Serial.println("'!");
      }
    }
  }
}
