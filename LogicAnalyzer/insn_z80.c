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
