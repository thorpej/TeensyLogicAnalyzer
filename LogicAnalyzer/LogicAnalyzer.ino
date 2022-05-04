/*

  Logic Analyzer for 6502, 6800, 6809, or Z80 microprocessors based on a
  Teensy 4.1 microcontroller.

  See https://github.com/thorpej/TeensyLogicAnalyzer

  Based on https://github.com/jefftranter/6502/tree/master/LogicAnalyzer

  Copyright (c) 2021-2022 by Jeff Tranter <tranter@pobox.com>
  Copyright (c) 2022 by Jason R. Thorpe <thorpej@me.com>

  To Do:
  - Add support for Z80 control line triggers.
  - Add support for Z80 I/O read or write trigger.
  - Better 6809 disassembly.
  - Make use of all of the control signals snooped by the analyzer board.

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

// Maximum buffer size (in samples). Increase if needed; should be
// able to go up to at least 30,000 before running out of memory.
#define BUFFSIZE 5000

const char *versionString = "TeensyLogicAnalyzer version 0.1 by Jason R. Thorpe <thorpej@me.com>";
const char *origVersionString = "Based on Logic Analyzer version 0.30 by Jeff Tranter <tranter@pobox.com>";

// Type definitions
typedef enum { tr_address, tr_io, tr_data, tr_reset, tr_irq, tr_firq, tr_nmi, tr_none } trigger_t;
typedef enum { tr_read, tr_write, tr_either } cycle_t;
typedef enum { cpu_6502, cpu_65c02, cpu_6800, cpu_6809, cpu_6809e, cpu_z80 } cpu_t;

// addressing mode types
typedef enum {
  // Unrecognized opcodes get this
  am_invalid              = -1,

  //
  // 6809 addressing modes
  // N.B. We have 8- and 16-bit versions of Immediate and Relative merely
  // to account for the numnber of bytes following the opcode; the 6809
  // data sheet makes no such distinction.  Ditto for the sized modes in
  // the Indexed category.
  //
  am6809_first            = 0,

  am6809_inherent         = 0,
  am6809_direct           = 1,
  am6809_extended         = 2,
  am6809_rel8             = 3,
  am6809_rel16            = 4,
  am6809_imm8             = 5,
  am6809_imm16            = 6,

  // indexed addressing modes.  Keep indirect variants
  // ordered immediately after their non-indirect
  // counterparts; other code depends on this ordering.
  am6809_zero_off         = 7,
  am6809_zero_off_ind     = 8,
  am6809_const_off5       = 9,
  am6809_const_off8       = 10,
  am6809_const_off8_ind   = 11,
  am6809_const_off16      = 12,
  am6809_const_off16_ind  = 13,
  am6809_acc_off          = 14,
  am6809_acc_off_ind      = 15,
  am6809_post_inc1        = 16,
  am6809_post_inc2        = 17,
  am6809_post_inc2_ind    = 18,
  am6809_pre_dec1         = 19,
  am6809_pre_dec2         = 20,
  am6809_pre_dec2_ind     = 21,
  am6809_pcrel8           = 22,
  am6809_pcrel8_ind       = 23,
  am6809_pcrel16          = 24,
  am6809_pcrel16_ind      = 25,
  am6809_extended_ind     = 26,
  // Pseudo-modes for special cases
  am6809_exg_tfr          = 27,   // actually Immediate
  am6809_psh_pul          = 28,   // actually Immediate

  am6809_last             = 28,

  //
  // 6502 addressing modes.  These are all pseudo-modes
  // that only represent the number of post-opcode bytes
  // and how to display them.  See commentary below.
  //
  am6502_first            = 29,

  am6502_implied          = 29,
  am6502_u8               = 30, // nn
  am6502_u16              = 31, // nnnn
  am6502_rel8             = 32, // rrrr

  am6502_last             = 32,
} addrmode_t;

#define am6809_indexed_p(am)  ((am) >= am6809_zero_off && (am) <= am6809_extended_ind)
#define am6809_indirect_p(am) ((am) == am6809_zero_off_ind || (am) == am6809_const_off8_ind || \
                               (am) == am6809_const_off16_ind || (am) == am6809_acc_off_ind || \
                               (am) == am6809_post_inc2_ind || (am) == am6809_pre_dec2_ind || \
                               (am) == am6809_pcrel8_ind || (am) == am6809_pcrel16_ind || \
                               (am) == am6809_extended_ind)

#define am6502_65c02_only(am) ((am) == am6502_abs_idx_ind || (am) == am6502_zp_ind)

// Global variables
uint32_t control[BUFFSIZE];           // Recorded control line data
uint32_t address[BUFFSIZE];           // Recorded address data
uint32_t data[BUFFSIZE];              // Recorded data lines
uint32_t triggerAddress = 0;          // Address or data to trigger on
uint32_t aTriggerBits;                // GPIO bit pattern to trigger address on
uint32_t aTriggerMask;                // bitmask of GPIO address bits
uint32_t cTriggerBits;                // GPIO bit pattern to trigger control on
uint32_t cTriggerMask;                // bitmask of GPIO control bits
uint32_t dTriggerBits;                // GPIO bit pattern to trigger data on
uint32_t dTriggerMask;                // bitmask of GPIO data bits
int samples = 20;                     // Total number of samples to record (up to BUFFSIZE)
int pretrigger = 0;                   // Number of samples to record before trigger (up to samples)
int triggerPoint = 0;                 // Sample in buffer corresponding to trigger point
cpu_t cpu = cpu_65c02;                // Current CPU type
trigger_t triggerMode = tr_none;      // Type of trigger
cycle_t triggerCycle = tr_either;     // Trigger on read, write, or either
bool triggerLevel = false;            // Trigger level (false=low, true=high);
volatile bool triggerPressed = false; // Set by hardware trigger button

//
// Helpers to return datums of various types from the data buffer at the
// specified offset.
//
uint16_t
read_u16le(const uint8_t *buf, int i)
{
  return buf[i] | (buf[i + 1] << 8);
}

int16_t
read_s16le(const uint8_t *buf, int i)
{
  return (int16_t)read_u16le(buf, i);
}

uint16_t
read_u16be(const uint8_t *buf, int i)
{
  return (buf[i] << 8) | buf[i + i];
}

int16_t
read_s16be(const uint8_t *buf, int i)
{
  return (int16_t)read_u16be(buf, i);
}

//
// Instruction decoding.
//
// The decode buffer runs through a small state machine, gathering bytes until it
// has a valid opcode that can be displayed.
//
typedef enum { ds_idle, ds_fetching, ds_complete } decode_state_t;
#define INSN_DECODE_MAXBYTES    8
#define INSN_DECODE_MAXSTRING   20
struct insn_decode {
  decode_state_t      state;
  uint32_t            insn_address;
  uint32_t            resolved_address;
  bool                resolved_address_valid;
  void                (*next_state)(struct insn_decode *);
  int                 bytes_required;
  int                 bytes_fetched;
  addrmode_t          addrmode;
  uint8_t             bytes[INSN_DECODE_MAXBYTES];
  char                insn_string[INSN_DECODE_MAXSTRING];
};

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
  "PHP",       "ORA #$nn",    "ASLA",      "?",   "TSB XXXX",    "ORA $nnnn",   "ASL $nnnn",   "BBR0 $nn",
  "BPL rr",    "ORA ($nn),Y", "ORA ($nn)", "?",   "TRB $nn",     "ORA $nn,X",   "ASL $nn,X",   "RMB1 $nn",
  "CLC",       "ORA $nnnn,Y", "INCA",      "?",   "TRB $nn",     "ORA $nnnn,X", "ASL $nnnn,X", "BBR1 $nn",
  "JSR $nnnn", "AND ($nn,X)", "?",         "?",   "BIT $nn",     "AND $nn",     "ROL $nn",     "RMB2 $nn",
  "PLP",       "AND #$nn",    "ROLA",      "?",   "BIT $nnnn",   "AND $nnnn",   "ROL $nnnn",   "BBR2 $nn",
  "BMI rr",    "AND ($nn),Y", "AND ($nn)", "?",   "BIT $nn,X",   "AND $nn,X",   "ROL $nn,X",   "RMB3 $nn",
  "SEC",       "AND $nnnn,Y", "DECA",      "?",   "BIT $nn,X",   "AND $nnnn,X", "ROL $nnnn,X", "BBR3 $nn",
  "RTI",       "EOR ($nn,X)", "?",         "?",   "?",           "EOR $nn",     "LSR $nn",     "RMB4 $nn",
  "PHA",       "EOR #$nn",    "LSRA",      "?",   "JMP $nnnn",   "EOR $nnnn",   "LSR $nnnn",   "BBR4 $nn",
  "BVC rr",    "EOR ($nn),Y", "EOR ($nn)", "?",   "?",           "EOR $nn,X",   "LSR $nn,X",   "RMB5 $nn",
  "CLI",       "EOR $nnnn,Y", "PHY",       "?",   "?",           "EOR $nnnn,X", "LSR $nnnn,X", "BBR5 $nn",
  "RTS",       "ADC ($nn,X)", "?",         "?",   "STZ $nn",     "ADC $nn",     "ROR $nn",     "RMB6 $nn",
  "PLA",       "ADC #$nn",    "RORA",      "?",   "JMP ($nnnn)", "ADC $nnnn",   "ROR $nnnn",   "BBR6 $nn",
  "BVS rr",    "ADC ($nn),Y", "ADC ($nn)", "?",   "STZ $nn,X",   "ADC $nn,X",   "ROR $nn,X",   "RMB7 $nn",
  "SEI",       "ADC $nnnn,Y", "PLY",       "?",   "JMP ($nn,X)", "ADC $nnnn,X", "ROR $nnnn,X", "BBR7 $nn",
  "BRA rr",    "STA ($nn,X)", "?",         "?",   "STY $nn",     "STA $nn",     "STX $nn",     "SMB0 $nn",
  "DEY",       "BIT #$nn",    "TXA",       "?",   "STY $nnnn",   "STA $nnnn",   "STX $nnnn",   "BBS0 $nn",
  "BCC rr",    "STA ($nn),Y", "STA ($nn)", "?",   "STY $nn,X",   "STA $nn,X",   "STX ($nn),Y", "SMB1 $nn",
  "TYA",       "STA $nnnn,Y", "TXS",       "?",   "STZ $nn",     "STA $nnnn,X", "STZ $nn,X",   "BBS1 $nn",
  "LDY #$nn",  "LDA ($nn,X)", "LDX #$nn",  "?",   "LDY $nn",     "LDA $nnnn",   "LDX $nn",     "SMB2 $nn",
  "TAY",       "LDA #$nn",    "TAX",       "?",   "LDY $nnnn",   "LDA $nnnn",   "LDX $nnnn",   "BBS2 $nn",
  "BCS rr",    "LDA ($nn),Y", "LDA ($nn)", "?",   "LDY $nn,X",   "LDA $nn,X",   "LDX ($nn),Y", "SMB3 $nn",
  "CLV",       "LDA $nnnn,Y", "TSX",       "?",   "LDY $nnnn,X", "LDA $nnnn,X", "LDX $nnnn,Y", "BBS3 $nn",
  "CPY #$nn",  "CMP ($nn,X)", "?",         "?",   "CPY $nnnn",   "CMP $nnnn",   "DEC $nnnn",   "SMB4 $nn",
  "INY",       "CMP #$nn",    "DEX",       "WAI", "CPY $nn",     "CMP $nn",     "DEC $nn",     "BBS4 $nn",
  "BNE rr",    "CMP ($nn),Y", "CMP ($nn)", "?",   "?",           "CMP $nn,X",   "DEC $nn,X",   "SMB5 $nn",
  "CLD",       "CMP $nnnn,Y", "PHX",       "STP", "?",           "CMP $nnnn,X", "DEC $nnnn,X", "BBS5 $nn",
  "CPX #$nn",  "SBC ($nn,X)", "?",         "?",   "CPX $nn",     "SBC $nn",     "INC $nn",     "SMB6 $nn",
  "INX",       "SBC #$nn",    "NOP",       "?",   "CPX $nnnn",   "SBC $nnnn",   "INC $nnnn",   "BBS6 $nn",
  "BEQ rr",    "SBC ($nn),Y", "SBC ($nn)", "?",   "?",           "SBC $nn,X",   "INC $nn,X",   "SMB7 $nn",
  "SED",       "SBC $nnnn,Y", "PLX",       "?",   "?",           "SBC $nnnn,X", "INC $nnnn,X", "BBS7 $nnnn"
};

const char *opcodes_6502[256] = {
  "BRK",       "ORA ($nn,X)", "?",        "?", "?",           "ORA $nn",     "ASL $nn",     "?",
  "PHP",       "ORA #$nn",    "ASLA",     "?", "?",           "ORA $nnnn",   "ASL $nnnn",   "?",
  "BPL rr",    "ORA ($nn),Y", "?",        "?", "?",           "ORA $nn,X",   "ASL $nn,X",   "?",
  "CLC",       "ORA $nnnn,Y", "?",        "?", "?",           "ORA $nnnn,X", "ASL $nnnn,X", "?",
  "JSR $nnnn", "AND ($nn,X)", "?",        "?", "BIT $nn",     "AND $nn",     "ROL $nn",     "?",
  "PLP",       "AND #$nn",    "ROLA",     "?", "BIT $nnnn",   "AND $nnnn",   "ROL $nnnn",   "?",
  "BMI rr",    "AND ($nn),Y", "?",        "?", "?",           "AND $nn,X",   "ROL $nn,X",   "?",
  "SEC",       "AND $nnnn,Y", "?",        "?", "?",           "AND $nnnn,X", "ROL $nnnn,X", "?",
  "RTI",       "EOR ($nn,X)", "?",        "?", "?",           "EOR nn",      "LSR $nn",     "?",
  "PHA",       "EOR #$nn",    "LSRA",     "?", "JMP $nnnn",   "EOR $nnnn",   "LSR $nnnn",   "?",
  "BVC rr",    "EOR ($nn),Y", "?",        "?", "?",           "EOR $nn,X",   "LSR $nn,X",   "?",
  "CLI",       "EOR $nnnn,Y", "?",        "?", "?",           "EOR $nnnn,X", "LSR $nnnn,X", "?",
  "RTS",       "ADC ($nn,X)", "?",        "?", "?",           "ADC $nn",     "ROR $nn",     "?",
  "PLA",       "ADC #$nn",    "RORA",     "?", "JMP ($nnnn)", "ADC $nnnn",   "ROR $nnnn",   "?",
  "BVS rr",    "ADC ($nn),Y", "?",        "?", "?",           "ADC $nn,X",   "ROR $nn,X",   "?",
  "SEI",       "ADC $nnnn,Y", "?",        "?", "?",           "ADC $nnnn,X", "ROR $nnnn,X", "?",
  "?",         "STA ($nn,X)", "?",        "?", "STY $nn",     "STA $nn",     "STX $nn",     "?",
  "DEY",       "?",           "TXA",      "?", "STY $nnnn",   "STA $nnnn",   "STX $nnnn",   "?",
  "BCC rr",    "STA ($nn),Y", "?",        "?", "STY $nn,X",   "STA $nn,X",   "STX $nn,Y",   "?",
  "TYA",       "STA $nnnn,Y", "TXS",      "?", "?",           "STA $nnnn,X", "?",           "?",
  "LDY #$nn",  "LDA ($nn,X)", "LDX #$nn", "?", "LDY $nn",     "LDA $nn",     "LDX $nn",     "?",
  "TAY",       "LDA #$nn",    "TAX",      "?", "LDY $nnnn",   "LDA $nnnn",   "LDX $nnnn",   "?",
  "BCS rr",    "LDA ($nn),Y", "?",        "?", "LDY $nn,X",   "LDA $nn,X",   "LDX $nn,Y",   "?",
  "CLV",       "LDA $nnnn,Y", "TSX",      "?", "LDY $nnnn,X", "LDA $nnnn,X", "LDX $nnnn,Y", "?",
  "CPY #$nn",  "CMP ($nn,X)", "?",        "?", "CPY $nn",     "CMP $nn",     "DEC $nn",     "?",
  "INY",       "CMP #$nn",    "DEX",      "?", "CPY $nnnn",   "CMP $nnnn",   "DEC $nnnn",   "?",
  "BNE rr",    "CMP ($nn),Y", "?",        "?", "?",           "CMP $nn,X",   "DEC $nn,X",   "?",
  "CLD",       "CMP $nnnn,Y", "?",        "?", "?",           "CMP $nnnn,X", "DEC $nnnn,X", "?",
  "CPX #$nn",  "SBC ($nn,X)", "?",        "?", "CPX $nn",     "SBC $nn",     "INC $nn",     "?",
  "INX",       "SBC #$nn",    "NOP",      "?", "CPX $nnnn",   "SBC $nnnn",   "INC $nnnn",   "?",
  "BEQ rr",    "SBC ($nn),Y", "?",        "?", "?",           "SBC $nn,X",   "INC $nn,X",   "?",
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
        sprintf(op, "%4d", val);
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

#if 0
// Instructions for Z80 disassembler.
// Two-byte extended instructions are not yet supported.
const char *opcodes_z80[256] = {
  "NOP", "LD", "LD", "INC", "INC", "DEC", "LD", "RLCA",
  "EX", "ADD", "LD", "DEC", "INC", "DEC", "LD", "RRCA",
  "DJNZ", "LD", "LD", "INC", "INC", "DEC", "LD", "RLA",
  "JR", "ADD", "LD", "DEC", "INC", "DEC", "LD", "RRA",
  "JR", "LD", "LD", "INC", "INC", "DEC", "LD", "DAA",
  "JR", "ADD", "LD", "DEC", "INC", "DEC", "LD", "CPL",
  "JR", "LD", "LD", "INC", "INC", "DEC", "LD", "SCF",
  "JR", "ADD", "LD", "DEC", "INC", "DEC", "LD", "CCF",
  "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
  "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
  "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
  "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
  "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
  "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
  "LD", "LD", "LD", "LD", "LD", "LD", "HALT", "LD",
  "LD", "LD", "LD", "LD", "LD", "LD", "LD", "LD",
  "ADD", "ADD", "ADD", "ADD", "ADD", "ADD", "ADD", "ADD",
  "ADC", "ADC", "ADC", "ADC", "ADC", "ADC", "ADC", "ADC",
  "SUB", "SUB", "SUB", "SUB", "SUB", "SUB", "SUB", "SUB",
  "SBC", "SBC", "SBC", "SBC", "SBC", "SBC", "SBC", "SBC",
  "AND", "AND", "AND", "AND", "AND", "AND", "AND", "AND",
  "XOR", "XOR", "XOR", "XOR", "XOR", "XOR", "XOR", "XOR",
  "OR", "OR", "OR", "OR", "OR", "OR", "OR", "OR",
  "CP", "CP", "CP", "CP", "CP", "CP", "CP", "CP",
  "RET", "POP", "JP", "JP", "CALL", "PUSH", "ADD", "RST",
  "RET", "RET", "JP", "(extended)", "CALL", "CALL", "ADC", "RST",
  "RET", "POP", "JP", "OUT", "CALL", "PUSH", "SUB", "RST",
  "RET", "EXX", "JP", "IN", "CALL", "(extended)", "SBC", "RST",
  "RET", "POP", "JP", "EX", "CALL", "PUSH", "AND", "RST",
  "RET", "JP", "JP", "EX", "CALL", "(extended)", "XOR", "RST",
  "RET", "POP", "JP", "DI", "CALL", "PUSH", "OR", "RST",
  "RET", "LD", "JP", "EI", "CALL", "(extended)", "CP", "RST"
};
#endif

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
  if (pb == 0b10011111) {
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
      sprintf(id->insn_string, "%s < $%02x", opc, id->bytes[i]);
      break;

    case am6809_extended_ind:
      // This is really an indexed mode; skip the index postbyte.
      i++;
      // FALLTHROUGH
    case am6809_extended:
      u16 = read_u16be(id->bytes, i);
      sprintf(id->insn_string, "%s %s$%04x%s", opc, ind_open, u16, ind_close);
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
      sprintf(id->insn_string, "%s #$%02x", opc, id->bytes[i]);
      break;

    case am6809_imm16:
      u16 = read_u16be(id->bytes, i);
      sprintf(id->insn_string, "%s #$%04x", opc, u16);
      break;

    case am6809_zero_off:
    case am6809_zero_off_ind:
      sprintf(id->insn_string, "%s %s,%s%s", opc, ind_open,
              index_regnames[index_reg], ind_close);
      break;

    case am6809_const_off5:
      s16 = id->bytes[i];
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
        case 0b0100: cp1 = "A"; break;
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
    // and that would require fetching a third byte.  Once we have  the
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

// Macros
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
  Serial.println(versionString);
  Serial.println(origVersionString);
  Serial.println("Type h or ? for help.");
}


// Interrupt handler for trigger button.
void
triggerButton(void)
{
  triggerPressed = true;
}

// Display settings and help info.
void
help(void)
{
  Serial.println(versionString);
  Serial.println(origVersionString);

  Serial.print("CPU: ");
  switch (cpu) {
    case cpu_6502:
      Serial.println("6502");
      break;
    case cpu_65c02:
      Serial.println("65C02");
      break;
    case cpu_6800:
      Serial.println("6800");
      break;
    case cpu_6809:
      Serial.println("6809");
      break;
    case cpu_6809e:
      Serial.println("6809E");
      break;
    case cpu_z80:
      Serial.println("Z80");
      break;
  }

  Serial.print("Trigger: ");
  switch (triggerMode) {
    case tr_address:
      Serial.print("on address ");
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
    case tr_io:
      Serial.print("on io ");
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

  Serial.print("Sample buffer size: ");
  Serial.println(samples);
  Serial.print("Pretrigger samples: ");
  Serial.println(pretrigger);
  Serial.println("Commands:");
  Serial.println("c <cpu>              - Set CPU to 6502, 65C02, 6800, 6809, or Z80");
  Serial.println("s <number>           - Set number of samples");
  Serial.println("p <samples>          - Set pre-trigger samples");
  Serial.println("t a <address> [r|w]  - Trigger on address");
  if (cpu == cpu_z80) {
    Serial.println("t i <address> [r|w]  - Trigger on i/o address");
  }
  Serial.println("t d <data> [r|w]     - Trigger on data");
  Serial.println("t reset 0|1          - Trigger on /RESET level");
  if (cpu == cpu_z80) {
    Serial.println("t int 0|1            - Trigger on /INT level");
  } else {
    Serial.println("t irq 0|1            - Trigger on /IRQ level");
  }
  if (cpu == cpu_6809 || cpu == cpu_6809e) {
    Serial.println("t firq 0|1           - Trigger on /FIRQ level");
  }
  Serial.println("t nmi 0|1            - Trigger on /NMI level");
  Serial.println("t none               - Trigger freerun");
  Serial.println("g                    - Go/start analyzer");
  Serial.println("l [start] [end]      - List samples");
  Serial.println("e                    - Export samples as CSV");
  Serial.println("w                    - Write data to SD card");
  Serial.println("h or ?               - Show command usage");
}

void
insn_decode_init(struct insn_decode *id)
{
  id->state = ds_idle;
  if (cpu == cpu_6809 || cpu == cpu_6809e) {
    id->next_state = insn_decode_next_state_6809;
  } else {
    id->next_state = NULL;
  }
}

void
insn_decode_begin(struct insn_decode *id, uint32_t addr, uint8_t b)
{
  if (id->next_state != NULL && id->state == ds_idle) {
    id->state = ds_fetching;
    id->insn_address = addr;
    id->resolved_address = 0;
    id->resolved_address_valid = false;
    id->addrmode = am_invalid;
    id->bytes_required = 0;
    id->bytes_fetched = 0;
    id->bytes[id->bytes_fetched++] = b;
    (*id->next_state)(id);
  }
}

bool
insn_decode_continue(struct insn_decode *id, uint8_t b)
{
  if (id->state == ds_fetching) {
    if (id->bytes_fetched == INSN_DECODE_MAXBYTES) {
      strcpy(id->insn_string, "<decode overflow>");
      id->state = ds_complete;
    } else {
      id->bytes[id->bytes_fetched++] = b;
      if (id->bytes_required == 0 || id->bytes_fetched == id->bytes_required) {
        (*id->next_state)(id);
      }
    }
  }
  return id->state == ds_fetching;
}

const char *
insn_decode_complete(struct insn_decode *id)
{
  if (id->state == ds_complete) {
     id->state = ds_idle;
     return id->insn_string;
  }
  return "";
}

// List recorded data from start to end.
void
list(Stream &stream, int start, int end)
{
  char output[80];

  int first = (triggerPoint - pretrigger + samples) % samples;
  int last = (triggerPoint - pretrigger + samples - 1) % samples;

  bool seen_lic = false;

  const char *cycle, *comment;

  struct insn_decode id;
  insn_decode_init(&id);

  // Display data
  int i = first;
  int j = 0;
  while (true) {
    cycle = "";
    comment = "";

    if ((j >= start) && (j <= end)) {

      // 6502 SYNC high indicates opcode/instruction fetch, otherwise
      // show as read or write.
      if ((cpu == cpu_65c02) || (cpu == cpu_6502)) {
        if (control[i] & CC_6502_SYNC) {
          insn_decode_begin(&id, address[i], data[i]);
          cycle = "F";
        } else if (control[i] & CC_6502_RW) {
          insn_decode_continue(&id, data[i]);
          cycle = "R";
        } else {
          cycle = "W";
        }
      }

      if (cpu == cpu_6809 || cpu == cpu_6809e) {
        // 6809 doens't have a VMA signal like the 6800, but the
        // data sheet describes how to detect a so-called "dummy
        // cycle" (which is also calls "/VMA").
        if (address[i] == 0xffff &&
            (control[i] & (CC_6809_RW | CC_6809_BS)) == CC_6809_RW) {
          cycle = "-";
        } else if (control[i] & CC_6809_RW) {
          // On 6809E, if we saw LIC on the previous cycle, then
          // this is an insn fetch.
          cycle = "R";
          if (cpu == cpu_6809e) {
            if (seen_lic) {
              cycle = "F";
              insn_decode_begin(&id, address[i], data[i]);
              seen_lic = false;
            } else {
              insn_decode_continue(&id, data[i]);
            }
          }
        } else {
          cycle = "W";
        }
        if (cpu == cpu_6809e && (control[i] & CC_6809E_LIC)) {
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
          cycle = "R";
          insn_decode_continue(&id, data[i]);
        } else if (!(control[i] & CC_Z80_MREQ) && !(control[i] & CC_Z80_WR)) {
          cycle = "W";
        } else if (!(control[i] & CC_Z80_IORQ) && !(control[i] & CC_Z80_RD)) {
          cycle = "IR";
        } else if (!(control[i] & CC_Z80_IORQ) && !(control[i] & CC_Z80_WR)) {
          cycle = "IW";
        } else {
          cycle = " ";
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

      // Check for 6502 /RESET, /IRQ, or /NMI active, vector address, or
      // stack access
      if ((cpu == cpu_65c02) || (cpu == cpu_6502)) {
        if (!(control[i] & CC_6502_RESET)) {
          comment = "RESET ACTIVE";
        } else if (!(control[i] & CC_6502_IRQ)) {
          comment = "IRQ ACTIVE";
        } else if (!(control[i] & CC_6502_NMI)) {
          comment = "NMI ACTIVE";
        } else if ((address[i] == 0xfffa) || (address[i] == 0xfffb)) {
          comment = "NMI VECTOR";
        } else if ((address[i] == 0xfffc) || (address[i] == 0xfffd)) {
          comment = "RESET VECTOR";
        } else if ((address[i] == 0xfffe) || (address[i] == 0xffff)) {
          comment = "IRQ/BRK VECTOR";
        } else if ((address[i] >= 0x0100) && (address[i] <= 0x01ff)) {
          comment = "STACK ACCESS";
        } else {
          comment = "";
        }
      }

      // Check for 6800 /RESET, /IRQ, or /NMI active, vector address.
      if (cpu == cpu_6800) {
        if (!(control[i] & CC_6800_RESET)) {
          comment = "RESET ACTIVE";
        } else if (!(control[i] & CC_6800_IRQ)) {
          comment = "IRQ ACTIVE";
        } else if (!(control[i] & CC_6800_NMI)) {
          comment = "NMI ACTIVE";
        } else if ((address[i] == 0xfff8) || (address[i] == 0xfff8)) {
          comment = "IRQ VECTOR";
        } else if ((address[i] == 0xfffa) || (address[i] == 0xfffb)) {
          comment = "SWI VECTOR";
        } else if ((address[i] == 0xfffc) || (address[i] == 0xfffd)) {
          comment = "NMI VECTOR";
        } else if (address[i] == 0xfffe) { // Not 0xffff since it commonly occurs when bus is tri-state
          comment = "RESET VECTOR";
        } else {
          comment = "";
        }
      }

      // Check for 6809 /RESET, /IRQ, or /NMI active, vector address.
      if (cpu == cpu_6809 || cpu == cpu_6809e) {
        if (!(control[i] & CC_6809_RESET)) {
          comment = "RESET ACTIVE";
        } else if (!(control[i] & CC_6809_IRQ)) {
          comment = "IRQ ACTIVE";
        } else if (!(control[i] & CC_6809_FIRQ)) {
          comment = "FIRQ ACTIVE";
        } else if (!(control[i] & CC_6809_NMI)) {
          comment = "NMI ACTIVE";
        } else if ((address[i] == 0xfff2) || (address[i] == 0xfff3)) {
          comment = "SWI3 VECTOR";
        } else if ((address[i] == 0xfff4) || (address[i] == 0xfff5)) {
          comment = "SWI2 VECTOR";
        } else if ((address[i] == 0xfff6) || (address[i] == 0xfff7)) {
          comment = "FIRQ VECTOR";
        } else if ((address[i] == 0xfff8) || (address[i] == 0xfff8)) {
          comment = "IRQ VECTOR";
        } else if ((address[i] == 0xfffa) || (address[i] == 0xfffb)) {
          comment = "SWI VECTOR";
        } else if ((address[i] == 0xfffc) || (address[i] == 0xfffd)) {
          comment = "NMI VECTOR";
        } else if (address[i] == 0xfffe) { // Not 0xffff since it commonly occurs when bus is tri-state
          comment = "RESET VECTOR";
        } else {
          comment = "";
        }
      }

      // Check for Z80 /RESET or /INT active
      if (cpu == cpu_z80) {
        if (!(control[i] & CC_Z80_RESET)) {
          comment = "RESET ACTIVE";
        } else if (!(control[i] & CC_Z80_INT)) {
          comment = "INT ACTIVE";
        } else {
          comment = "";
        }
      }

      // Indicate when trigger happened
      if (i == triggerPoint) {
        comment = "<--- TRIGGER ----";
      }

      sprintf(output,
          "%04lX  %-2s  %02lX  %-20s  %s",
          address[i], cycle, data[i], insn_decode_complete(&id),
          comment);

      stream.println(output);
    }

    if (i == last) {
      break;
    }

    i = (i + 1) % samples;
    j++;
  }
}


// Show the recorded data in CSV format (e.g. to export to spreadsheet or other program).
void
exportCSV(Stream &stream)
{
  bool sync;
  bool rw = false;
  bool reset = false;
  bool irq = false;
  bool firq = false;
  bool nmi = false;
  bool vma = false;
  bool ba = false;
  bool bs = false;
  bool lic = false;
  bool wr = false;
  bool rd = false;
  bool iorq = false;
  bool mreq = false;
  bool m1 = false;
  bool intr = false;

  // Output header
  if ((cpu == cpu_65c02) || (cpu == cpu_6502)) {
    stream.println("Index,SYNC,R/W,/RESET,/IRQ,/NMI,Address,Data");
  }
  if (cpu == cpu_6809) {
    stream.println("Index,BA,BS,R/W,/RESET,/IRQ,/FIRQ,/NMI,Address,Data");
  }
  if (cpu == cpu_6809e) {
    stream.println("Index,BA,BS,LIC,R/W,/RESET,/IRQ,/FIRQ,/NMI,Address,Data");
  }
  if (cpu == cpu_6800) {
    stream.println("Index,VMA,R/W,/RESET,/IRQ,/NMI,Address,Data");
  }
  if (cpu == cpu_z80) {
    stream.println("Index,/M1,/RD,/WR,/MREQ,/IORQ,/RESET,/INT,Address,Data");
  }

  int first = (triggerPoint - pretrigger + samples) % samples;
  int last = (triggerPoint - pretrigger + samples - 1) % samples;

  // Display data
  int i = first;
  int j = 0;
  while (true) {
    char output[50]; // Holds output string
    if ((cpu == cpu_65c02) || (cpu == cpu_6502)) {
      sync = control[i] & CC_6502_SYNC;
    }
    if ((cpu == cpu_65c02) || (cpu == cpu_6502) || (cpu == cpu_6800) || (cpu == cpu_6809) || (cpu == cpu_6809e)) {
      rw = control[i] & CC_6502_RW;
      reset = control[i] & CC_6502_RESET;
      irq = control[i] & CC_6502_IRQ;
      nmi = control[i] & CC_6502_NMI;
    }
    if (cpu == cpu_6800) {
      vma = control[i] & CC_6800_VMA;
    }
    if ((cpu == cpu_6809) || (cpu == cpu_6809e)) {
      ba = control[i] & CC_6809_BA;
      bs = control[i] & CC_6809_BS;
      firq = control[i] & CC_6809_FIRQ;
    }
    if (cpu == cpu_6809e) {
      lic = control[i] & CC_6809E_LIC;
    }
    if (cpu == cpu_z80) {
      wr = control[i] & CC_Z80_WR;
      rd = control[i] & CC_Z80_RD;
      iorq = control[i] & CC_Z80_IORQ;
      mreq = control[i] & CC_Z80_MREQ;
      m1 = control[i] & CC_Z80_M1;
      reset = control[i] & CC_Z80_RESET;
      intr = control[i] & CC_Z80_INT;
    }

    if ((cpu == cpu_65c02) || (cpu == cpu_6502)) {
      sprintf(output, "%d,%c,%c,%c,%c,%c,%04lX,%02lX",
              j,
              sync ? '1' : '0',
              rw ? '1' : '0',
              reset ? '1' : '0',
              irq ? '1' : '0',
              nmi ? '1' : '0',
              address[i],
              data[i]
             );
    }
    if (cpu == cpu_6800) {
      sprintf(output, "%d,%c,%c,%c,%c,%c,%04lX,%02lX",
              j,
              vma ? '1' : '0',
              rw ? '1' : '0',
              reset ? '1' : '0',
              irq ? '1' : '0',
              nmi ? '1' : '0',
              address[i],
              data[i]
             );
    }
    if (cpu == cpu_6809) {
      sprintf(output, "%d,%c,%c,%c,%c,%c,%c,%c,%04lX,%02lX",
              j,
              ba ? '1' : '0',
              bs ? '1' : '0',
              rw ? '1' : '0',
              reset ? '1' : '0',
              irq ? '1' : '0',
              firq ? '1' : '0',
              nmi ? '1' : '0',
              address[i],
              data[i]
             );
    }
    if (cpu == cpu_6809e) {
      sprintf(output, "%d,%c,%c,%c,%c,%c,%c,%c,%c,%04lX,%02lX",
              j,
              ba ? '1' : '0',
              bs ? '1' : '0',
              lic ? '1' : '0',
              rw ? '1' : '0',
              reset ? '1' : '0',
              irq ? '1' : '0',
              firq ? '1' : '0',
              nmi ? '1' : '0',
              address[i],
              data[i]
             );

    }
    if (cpu == cpu_z80) {
      sprintf(output, "%d,%c,%c,%c,%c,%c,%c,%c,%04lX,%02lX",
              j,
              m1 ? '1' : '0',
              rd ? '1' : '0',
              wr ? '1' : '0',
              mreq ? '1' : '0',
              iorq ? '1' : '0',
              reset ? '1' : '0',
              intr ? '1' : '0',
              address[i],
              data[i]
             );
    }

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
    exportCSV(file);
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
    list(file, 0, samples - 1);
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
  
  // Scramble the trigger address, control, and data lines to match what we will read on the ports.
  if (triggerMode == tr_address) {
    aTriggerBits = scramble_CAxx(triggerAddress);
    aTriggerMask = scramble_CAxx(0xffff);

    // Check for r/w qualifer
    if (cpu != cpu_z80) {
      // 6502, 6800, 6809 -- all 6800-like
      cTriggerMask = scramble_CCxx(CC_6800_RW, &aTriggerMask, &dTriggerMask);
      if (triggerCycle == tr_read) {
        cTriggerBits = scramble_CCxx(CC_6800_RW, &aTriggerBits, &dTriggerBits);
      }
    } else {
      // TODO: r/w qualifier for Z80
    }
  } else if (triggerMode == tr_data) {
    dTriggerBits = scramble_CDxx(triggerAddress);
    dTriggerMask = scramble_CDxx(0xff);

    // Check for r/w qualifier
    if (cpu != cpu_z80) {
      // 6502, 6800, 6809 -- all 6800-like
      cTriggerMask = scramble_CCxx(CC_6800_RW, &aTriggerMask, &dTriggerMask);
      if (triggerCycle == tr_read) {
        cTriggerBits = scramble_CCxx(CC_6800_RW, &aTriggerBits, &dTriggerBits);
      }
    } else {
      // TODO: r/w qualifier for Z80
    }

    // TODO: Add support for Z80 I/O read or write trigger.

    // TODO: Add support for Z80 I/O control line triggers.

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
  int samplesTaken = 0; // Number of samples taken
  bool triggered = false; // Set when triggered

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
    } else if (cmd == "c 6502") {
      cpu = cpu_6502;
    } else if ((cmd == "c 65c02") || (cmd == "c 65C02")) {
      cpu = cpu_65c02;
    } else if (cmd == "c 6800") {
      cpu = cpu_6800;
    } else if (cmd == "c 6809") {
      cpu = cpu_6809;
    } else if ((cmd == "c 6809e") || (cmd == "c 6809E")) {
      cpu = cpu_6809e;
    } else if ((cmd == "c z80") || (cmd == "c Z80")) {
      cpu = cpu_z80;

      // Samples
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
        triggerAddress = n;
        triggerMode = tr_data;
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
    } else if (cmd.startsWith("t i ")) {
      int n = strtol(cmd.substring(4, 6).c_str(), NULL, 16);
      if ((n >= 0) && (n <= 0xff)) {
        triggerAddress = n;
        triggerMode = tr_io;
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

      // Go
    } else if (cmd == "g") {
      go();

      // List
    } else if (cmd == "l") {
      list(Serial, 0, samples - 1);
    } else if (cmd.startsWith("l ")) {
      if (cmd.indexOf(" ") == cmd.lastIndexOf(" ")) {
        // l <start>
        int start = cmd.substring(2).toInt();
        if ((start < 0) || (start >= samples)) {
          Serial.print("Invalid start, must be between 0 and ");
          Serial.print(samples - 1);
          Serial.println(".");
        } else {
          list(Serial, start, samples - 1);
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
          list(Serial, start, end);
        }
      }

      // Export
    } else if (cmd == "e") {
      exportCSV(Serial);

      // Write
    } else if (cmd == "w") {
      writeSD();

      // Invalid command
    } else {
      if (cmd != "") {
        Serial.print("Invalid command: '");
        Serial.print(cmd);
        Serial.println("'!");
      }
    }
  }
}
