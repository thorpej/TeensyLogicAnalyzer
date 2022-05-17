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

#ifndef insn_decode_h_included
#define insn_decode_h_included

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

  //
  // 6800 addressing modes.
  //
  am6800_first            = 33,

  am6800_inherent         = 33,
  am6800_rel              = 34,
  am6800_indexed          = 35,
  am6800_imm8             = 36,
  am6800_imm16            = 37,
  am6800_direct           = 38,
  am6800_extended         = 39,

  am6800_last             = 39,

  //
  // Z80 addressing modes.  As with 6502, these are pseudo-modes
  // that represent the post-opcode bytes and display substitutions.
  // Note that some Z80 instructions have multiple substitutions.
  //
  amz80_first             = 40,

  amz80_implied           = 40,
  amz80_u8                = 41, // XXh
  amz80_u16               = 42, // XXXXh
  amz80_disp8             = 43, // +ddd (signed displacement)
  amz80_pcrel8            = 44, // rrrr (signed displacement, add 2 to displayed value)

  amz80_last              = 44,
} addrmode_t;

#define am6809_indirect_p(am) ((am) == am6809_zero_off_ind || (am) == am6809_const_off8_ind || \
                               (am) == am6809_const_off16_ind || (am) == am6809_acc_off_ind || \
                               (am) == am6809_post_inc2_ind || (am) == am6809_pre_dec2_ind || \
                               (am) == am6809_pcrel8_ind || (am) == am6809_pcrel16_ind || \
                               (am) == am6809_extended_ind)

//
// Instruction decoding.
//
// The decode buffer runs through a small state machine, gathering bytes until it
// has a valid opcode that can be displayed.
//
typedef enum { ds_idle, ds_fetching, ds_complete } decode_state_t;
#define INSN_DECODE_MAXBYTES    8
#define INSN_DECODE_MAXSTRING   28  // See also printf format in list().
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
// Helpers to return datums of various types from the data buffer at the
// specified offset.
//
static inline uint16_t
read_u16le(const uint8_t *buf, int i)
{
  return buf[i] | (buf[i + 1] << 8);
}

static inline int16_t
read_s16le(const uint8_t *buf, int i)
{
  return (int16_t)read_u16le(buf, i);
}

static inline uint16_t
read_u16be(const uint8_t *buf, int i)
{
  return (buf[i] << 8) | buf[i + 1];
}

static inline int16_t
read_s16be(const uint8_t *buf, int i)
{
  return (int16_t)read_u16be(buf, i);
}

#if defined(__cplusplus)
extern "C" {
#endif

void insn_decode_init(struct insn_decode *);
bool insn_decode_next_state(struct insn_decode *);
void insn_decode_begin(struct insn_decode *, uint32_t, uint8_t);
bool insn_decode_continue(struct insn_decode *, uint8_t);
const char *insn_decode_complete(struct insn_decode *);

void insn_decode_next_state_6502(struct insn_decode *);
void insn_decode_next_state_6800(struct insn_decode *);
void insn_decode_next_state_6809(struct insn_decode *);
void insn_decode_next_state_z80(struct insn_decode *);

#if defined(__cplusplus)
}
#endif

#endif /* insn_decode_h_included */
