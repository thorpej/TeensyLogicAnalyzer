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

#include "tla.h"
#include "insn_decode.h"

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
static const char *opcodes_65c02[256] = {
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

static const char *opcodes_6502[256] = {
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

static void
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
