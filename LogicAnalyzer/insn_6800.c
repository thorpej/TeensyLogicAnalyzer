/*

  Teensy Logic Analyzer
  Logic Analyzer for 6502, 6800, 6809, or Z80 microprocessors based on a
  Teensy 4.1 microcontroller.

  See https://github.com/thorpej/TeensyLogicAnalyzer

  Copyright (c) 2022 by Jason R. Thorpe <thorpej@me.com>

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
// 6800 instruction decoding
//
static const char *opcodes_6800[256] = {
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

static addrmode_t
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

static void
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
