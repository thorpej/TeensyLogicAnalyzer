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
