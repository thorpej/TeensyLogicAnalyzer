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

//
// FAKE SAMPLE DATA FOR TEST AND DEBUGGING PURPOSES.  You can only enable
// one of these at a time.
//

#ifdef DEBUG_6502
#define DEBUG_CPU   cpu_6502
// Main goal of the 6502 debug data is to exercise the instruction decoder.
// Because of the simplicity of the 6502 addressing modes, we only need to
// exercise a handful of real addressing modes.
const uint32_t debug_data[] = {
  // BRK
  0x00,

  // ORA $20
  0x05, 0x20,

  // LDY $3000
  0xac, 0x00, 0x30,

  // BPL 16
  0x10, 0x10,
};

const uint32_t debug_address[] = {
  // BRK
  0x1000,

  // ORA $20
  0x1001, 0x1002,

  // LDY $3000
  0x1003, 0x1004, 0x1005,

  // BPL 16
  0x1006, 0x1007,
};

#define N   (CC_6502_RW | CC_6502_RESET | CC_6502_NMI | CC_6502_IRQ)
#define FN  (CC_6502_SYNC | N)

const uint32_t debug_control[] = {
  // BRK
  FN,

  // ORA $20
  FN, N,

  // LDY $3000
  FN, N, N,

  // BPL 16
  FN, N,
};

#undef N
#undef F
#endif // DEBUG_6502

#ifdef DEBUG_6809
#endif // DEBUG_6809

#ifdef DEBUG_6809E
#define DEBUG_CPU             cpu_6809e
#define DEBUG_TRIGGER_POINT   84

// Main goal of the 6809E debug data is to exercise the instruction decoder.
// Most of these samples are not cycle-accurate in any way, shape, or form.
const uint32_t debug_data[] = {
  // Dummy sample, just to set LIC
  0,

  // Inherent addressing mode: NEGA
  0x40,

  // Direct addressing mode: INC < $10
  0x0c, 0x10,

  // Relative addressing mode: BRA -3
  0x20, 0xfd,

  // Extended addressing mode: ASR $CAFE
  0x77, 0xca, 0xfe,

  // Immediate addressing mode: ORA #$5A
  0x8a, 0x5a,

  // Indexed zero-offset: LEAX ,X
  0x30, 0b10000100,

  // Indexed zero-offset indirect: LEAX [,X]
  0x30, 0b10010100,

  // Indexed 5-bit constant offset: LEAY 1,Y
  0x31, 0b00100001,

  // Indexed 8-bit constant offset: LEAS -64,S
  0x32, 0b11101000, 0xc0,

  // Indexed 8-bit constant offset indirect: LEAU [-64,U]
  0x33, 0b11011000, 0xc0,

  // Indexed 16-bit constant offset: SUBA 384,Y
  0xa0, 0b10101001, 0x01, 0x80,

  // Indexed 16-bit constant offset indirect: LDA [1024,X]
  0xa6, 0b10011001, 0x04, 0x00,

  // Indexed accumulator offset: STA A,U
  0xa7, 0b11000110,

  // Indexed accumulator offset indirect: STA [A,U]
  0xa7, 0b11010110,

  // Indexed accumulator offset: ORA B,Y
  0xaa, 0b10100101,

  // Indexed accumulator offset: CMPX D,S
  0xac, 0b11101011,

  // Indexed Auto Increment by 1: LDA ,X+
  0xa6, 0b10000000,

  // Indexed Auto Increment by 2: LDY ,X++
  0x10, 0xae, 0b10000001,

  // Indexed Auto Increment by 2 indirect: LDY [,X++]
  0x10, 0xae, 0b10010001,

  // Indexed Auto Decrement by 1: LDA ,-X
  0xa6, 0b10000010,

  // Indexed Auto Decrement by 2: LDY ,--X
  0x10, 0xae, 0b10000011,

  // Indexed Auto Decrement by 2 indirect: LDY [,--X]
  0x10, 0xae, 0b10010011,

  // Indexed constant 8-bit offset from PC: LDB 10,PCR
  0xe6, 0b10001100, 0x0a,

  // Indexed constant 8-bit offset from PC indirect: LDB [10,PCR]
  0xe6, 0b10011100, 0x0a,

  // Indexed constant 16-bit offset from PC: LDB 32767,PCR
  0xe6, 0b10001101, 0x7f, 0xff,

  // Indexed constant 16-bit offset from PC indirect: LDB 32767,PCR
  0xe6, 0b10011101, 0x7f, 0xff,

  // Extended indirect addressing mode: BITA [$CAFE]
  0xa5, 0b10011111, 0xca, 0xfe,

  // TFR D,U
  0x1f, (0b0000 << 4) | 0b0011,

  // EXG X,DPR
  0x1e, (0b0001 << 4) | 0b1011,

  // PSHS A,B,U
  0x34, 0b01000110,

  // PULU S
  0x37, 0b01000000,

  // Dummy IRQ vector
  0xc0, 0x00,

  // Dummy IRQ handler (NOP)
  0x12,

  // LBSR example cycle-by-cycle flow from 6809E data sheet
  0x17, 0x20, 0x00,
  0xff, 0xff,     // VMA, VMA
  0x12,           // NOP
  0xff,           // VMA
  0x88, 0x03,

  // DEC example cycle-by-cycle flow from 6809E data sheet
  0x7a, 0xa0, 0x00,
  0xff,
  0x80,
  0xff,
  0x7f,
};

const uint32_t debug_address[] = {
  // Dummy sample, just to set LIC
  0,

  // Inherent addressing mode: NEGA
  0x1000,

  // Direct addressing mode: INC < $10
  0x1001, 0x1002,

  // Relative addressing mode: BRA -3
  0x1003, 0x1004,

  // Extended addressing mode: ASR $CAFE
  0x1005, 0x1006, 0x1007,

  // Immediate addressing mode: ORA #$5A
  0x1008, 0x1009,

  // Indexed zero-offset: LEAX ,X
  0x100a, 0x100b,

  // Indexed zero-offset indirect: LEAX [,X]
  0x100c, 0x100d,

  // Indexed 5-bit constant offset: LEAY 1,Y
  0x100e, 0x100f,

  // Indexed 8-bit constant offset: LEAS -64,S
  0x1010, 0x1011, 0x1012,

  // Indexed 8-bit constant offset indirect: LEAU [-64,U]
  0x1013, 0x1014, 0x1015,

  // Indexed 16-bit constant offset: SUBA 384,Y
  0x1016, 0x1017, 0x1018, 0x1019,

  // Indexed 16-bit constant offset indirect: LDA [1024,X]
  0x101a, 0x101b, 0x101c, 0x101d,

  // Indexed accumulator offset: STA A,U
  0x101e, 0x101f,

  // Indexed accumulator offset indirect: STA [A,U]
  0x1020, 0x1021,

  // Indexed accumulator offset: ORA B,Y
  0x1022, 0x1023,

  // Indexed accumulator offset: CMPX D,S
  0x1024, 0x1025,

  // Indexed Auto Increment by 1: LDA ,X+
  0x1026, 0x1027,

  // Indexed Auto Increment by 2: LDY ,X++
  0x1028, 0x1029, 0x102a,

  // Indexed Auto Increment by 2 indirect: LDY [,X++]
  0x102b, 0x102c, 0x102d,

  // Indexed Auto Decrement by 1: LDA ,-X
  0x102e, 0x102f,

  // Indexed Auto Decrement by 2: LDY ,--X
  0x1030, 0x1031, 0x1032,

  // Indexed Auto Decrement by 2 indirect: LDY [,--X]
  0x1033, 0x1034, 0x1035,

  // Indexed constant 8-bit offset from PC: LDB 10,PCR
  0x1036, 0x1037, 0x1038,

  // Indexed constant 8-bit offset from PC indirect: LDB [10,PCR]
  0x1039, 0x103a, 0x103b,

  // Indexed constant 16-bit offset from PC: LDB 32767,PCR
  0x103c, 0x103d, 0x103e, 0x103f,

  // Indexed constant 16-bit offset from PC indirect: LDB 32767,PCR
  0x1040, 0x1041, 0x1042, 0x1043,

  // Extended indirect addressing mode: BITA [$CAFE]
  0x1044, 0x1045, 0x1047, 0x1048,

  // TFR D,U
  0x1049, 0x1050,

  // EXG X,DPR
  0x1051, 0x1052,

  // PSHS A,B,U
  0x1053, 0x1054,

  // PULU S
  0x1055, 0x1056,

  // Dummy IRQ vector
  0xfff8, 0xfff9,

  // Dummy IRQ handler (NOP)
  0xc000,

  // LBSR example cycle-by-cycle flow from 6809E data sheet
  0x8000, 0x8001, 0x8002,
  0xffff, 0xffff,   // VMA, VMA
  0xa000,
  0xffff,           // VMA
  0xefff,
  0xeffe,

  // DEC example cycle-by-cycle flow from 6809E data sheet
  0x8000, 0x8001, 0x8002,
  0xffff,
  0xa000,
  0xffff,
  0xa000,   // data sheet contains a paste-o (FFFF) here
};

#define N   (CC_6809_RW | CC_6809_IRQ | CC_6809_FIRQ | CC_6809_NMI | CC_6809_RESET)
#define NW  (CC_6809_IRQ | CC_6809_FIRQ | CC_6809_NMI | CC_6809_RESET)
#define VMA (CC_6809_RW | CC_6809_IRQ | CC_6809_FIRQ | CC_6809_NMI | CC_6809_RESET)
#define L   (CC_6809E_LIC | N)
#define NWL (CC_6809E_LIC | NW)

const uint32_t debug_control[] = {
  // Dummy sample, just to set LIC
  L,

  // Inherent addressing mode: NEGA
  L,

  // Direct addressing mode: INC < $10
  N, L,

  // Relative addressing mode: BRA -3
  N, L,

  // Extended addressing mode: ASR $CAFE
  N, N, L,

  // Immediate addressing mode: ORA #$5A
  N, L,

  // Indexed zero-offset: LEAX ,X
  N, L,

  // Indexed zero-offset indirect: LEAX [,X]
  N, L,

  // Indexed 5-bit constant offset: LEAY 1,Y
  N, L,

  // Indexed 8-bit constant offset: LEAS -64,S
  N, N, L,

  // Indexed 8-bit constant offset indirect: LEAU [-64,U]
  N, N, L,

  // Indexed 16-bit constant offset: SUBA 384,Y
  N, N, N, L,

  // Indexed 16-bit constant offset indirect: LDA [1024,X]
  N, N, N, L,

  // Indexed accumulator offset: STA A,U
  N, L,

  // Indexed accumulator offset indirect: STA [A,U]
  N, L,

  // Indexed accumulator offset: ORA B,Y
  N, L,

  // Indexed accumulator offset: CMPX D,S
  N, L,

  // Indexed Auto Increment by 1: LDA ,X+
  N, L,

  // Indexed Auto Increment by 2: LDY ,X++
  N, N, L,

  // Indexed Auto Increment by 2 indirect: LDY [,X++]
  N, N, L,

  // Indexed Auto Decrement by 1: LDA ,-X
  N, L,

  // Indexed Auto Decrement by 2: LDY ,--X
  N, N, L,

  // Indexed Auto Decrement by 2 indirect: LDY [,--X]
  N, N, L,

  // Indexed constant 8-bit offset from PC: LDB 10,PCR
  N, N, L,

  // Indexed constant 8-bit offset from PC indirect: LDB [10,PCR]
  N, N, L,

  // Indexed constant 16-bit offset from PC: LDB 32767,PCR
  N, N, N, L,

  // Indexed constant 16-bit offset from PC indirect: LDB 32767,PCR
  N, N, N, L,

  // Extended indirect addressing mode: BITA [$CAFE]
  N, N, N, L,

  // TFR D,U
  N, L,

  // EXG X,DPR
  N, L,

  // PSHS A,B,U
  N, L,

  // PULU S
  N, L,

  // Dummy IRQ vector
  N & ~CC_6809_IRQ, N & ~CC_6809_IRQ,

  // Dummy IRQ handler (NOP)
  (N & ~CC_6809_IRQ) | CC_6809E_LIC,

  // LBSR example cycle-by-cycle flow from 6809E data sheet
  N, N, N,
  VMA, VMA,
  N,
  VMA,
  NW, NWL,

  // DEC example cycle-by-cycle flow from 6809E data sheet
  N, N, N,
  VMA,
  N,
  VMA,
  NWL,
};

#undef N
#undef VMA
#undef L
#undef NWL
#endif // DEBUG_6809E

#ifdef DEBUG_6800
#endif // DEBUG_6800

#ifdef DEBUG_Z80
#define DEBUG_CPU             cpu_z80

// Main goal of the Z80 debug data is to exercise the instruction decoder.
// Most of these samples are not cycle-accurate in any way, shape, or form.
//
// Pretty much just going down the list from the data sheet "UM008011-0816".
const uint32_t debug_data[] = {
  // LD A,B
  0b01111000,

  // LD C,55h
  0b00001110,
  0x55,

  // LD D,(HL)
  0b01010110,

  // LD E,(IX+5)
  0xdd,
  0b01011110,
  5,

  // LD H,(IY-10)
  0xfd,
  0b01100110,
  0xf6,

  // LD (HL),A
  0b01110111,

  // LD (IX+127),L
  0xdd,
  0b01110101,
  127,

  // LD (IY-100),A
  0xfd,
  0b01110111,
  0x9c,

  // LD (HL),FFh
  0x36,
  0xff,

  // LD (IX+10),A5h
  0xdd,
  0x36,
  10,
  0xa5,

  // LD A,(BC)
  0x0a,

  // LD A,(DE)
  0x1a,

  // LD A,(1234h)
  0x3a,
  0x34,
  0x12,

  // LD (BC),A
  0x02,

  // LD (DE),A
  0x12,

  // LD (4567h),A
  0x32,
  0x67,
  0x45,

  // LD A,I
  0xed,
  0x57,

  // LD A,R
  0xed,
  0x5f,

  // LD I,A
  0xed,
  0x47,

  // LD R,A
  0xed,
  0x4f,

  // LD BC,1234h
  0b00000001,
  0x34,
  0x12,

  // LD IX,4567h
  0xdd,
  0b00100001,
  0x67,
  0x45,

  // LD IY,CAFEh
  0xfd,
  0b00100001,
  0xfe,
  0xca,

  // LD HL,(CAFEh)
  0x2a,
  0xfe,
  0xca,

  // LD HL,(BABEh) (alternate HL encoding)
  0xed,
  0b01101011,
  0xbe,
  0xba,

  // LD IX,(1234h)
  0xdd,
  0x2a,
  0x34,
  0x12,

  // LD IY,(1234h)
  0xfd,
  0x2a,
  0x34,
  0x12,

  // LD (1234h),HL
  0x22,
  0x34,
  0x12,

  // LD (4567h),SP
  0xed,
  0b01110011,
  0x67,
  0x45,

  // LD (CAFEh),IX XXX
  0xdd,
  0x22,
  0xfe,
  0xca,

  // LD (CAFEh),IY
  0xfd,
  0x22,
  0xfe,
  0xca,

  // LD SP,HL
  0xf9,

  // LD SP,IX
  0xdd,
  0xf9,

  // LD SP,IY
  0xfd,
  0xf9,

  // PUSH BC
  0b11000101,

  // PUSH IX
  0xdd,
  0xe5,

  // PUSH IY
  0xfd,
  0xe5,

  // POP AF
  0b11110001,

  // POP IX
  0xdd,
  0xe1,

  // POP IY
  0xfd,
  0xe1,

  // EX DE,HL
  0xeb,

  // EX AF,AF'
  0x08,

  // EXX
  0xd9,

  // EX (SP),HL
  0xe3,

  // EX (SP),IX
  0xdd,
  0xe3,

  // EX (SP),IY
  0xfd,
  0xe3,

  // LDI
  0xed,
  0xa0,

  // LDIR
  0xed,
  0xb0,

  // LDD
  0xed,
  0xa8,

  // LDDR
  0xed,
  0xb8,

  // CPI
  0xed,
  0xa1,

  // CPIR
  0xed,
  0xb1,

  // CPD
  0xed,
  0xa9,

  // CPDR
  0xed,
  0xb9,

  // ADD B
  0b10000000,

  // ADD 10h
  0xc6,
  0x10,

  // ADD (HL)
  0x86,

  // ADD (IX+100)
  0xdd,
  0x86,
  100,

  // ADC C
  0b10001001,

  // SUB (HL)
  0x96,

  // SBC (IY+8)
  0xfd,
  0x9e,
  8,

  // ADD HL,SP
  0b00111001,

  // ADC HL,SP
  0xed,
  0b01111010,

  // SBC HL,SP
  0xed,
  0b01110010,

  // ADD IX,IX
  0xdd,
  0b00101001,

  // ADD IY,IY
  0xfd,
  0b00101001,

  // INC BC
  0b00000011,

  // INC IY
  0xfd,
  0x23,

  // RLCA
  0x07,

  // RLC B
  0xcb,
  0b00000000,

  // RLC (IX+10)
  0xdd,
  0xcb,
  10,
  0x06,

  // BIT 1,L
  0xcb,
  0b01001101,

  // BIT 6,(IX+0)
  0xdd,
  0xcb,
  0,
  0b01110110,

  // JP BEEFh
  0xc3,
  0xef,
  0xbe,

  // JP Z,DEADh
  0b11001010,
  0xad,
  0xde,

  // JR 2 <10A6h>
  0x18,
  0x00,

  // JR 0 <10A6h>
  0x18,
  0xfe,

  // JR -2 <10A6h>
  0x18,
  0xfc,

  // JP (HL)
  0xe9,

  // JP (IX)
  0xdd,
  0xe9,

  // RETI
  0xed,
  0x4d,

  // RST 18h
  0b11011111,

  // IN A,(10h)
  0xdb,
  0x10,

  // IN C,(C)
  0xed,
  0b01001000,

  // OUT (C),H
  0xed,
  0b01100001,
};

const uint32_t debug_address[] = {
  // LD A,B
  0x1000,

  // LD C,55h
  0x1001,
  0x1002,

  // LD D,(HL)
  0x1003,

  // LD E,(IX+5)
  0x1004,
  0x1005,
  0x1006,

  // LD H,(IY-10)
  0x1007,
  0x1008,
  0x1009,

  // LD (HL),A
  0x100a,

  // LD (IX+127),L
  0x100b,
  0x100c,
  0x100d,

  // LD (IY-100),A
  0x100e,
  0x100f,
  0x1010,

  // LD (HL),FFh
  0x1011,
  0x1012,

  // LD (IX+10),A5h
  0x1013,
  0x1014,
  0x1015,
  0x1016,

  // LD A,(BC)
  0x1017,

  // LD A,(DE)
  0x1018,

  // LD A,(1234h)
  0x1019,
  0x101a,
  0x101b,

  // LD (BC),A
  0x101c,

  // LD (DE),A
  0x101d,

  // LD (4567h),A
  0x101e,
  0x101f,
  0x1020,

  // LD A,I
  0x1021,
  0x1022,

  // LD A,R
  0x1023,
  0x1024,

  // LD I,A
  0x1025,
  0x1026,

  // LD R,A
  0x1027,
  0x1028,

  // LD BC,1234h
  0x1029,
  0x102a,
  0x102b,

  // LD IX,4567h
  0x102c,
  0x102d,
  0x102e,
  0x102f,

  // LD IY,CAFEh
  0x1030,
  0x1031,
  0x1032,
  0x1033,

  // LD HL,(CAFEh)
  0x1034,
  0x1035,
  0x1036,

  // LD HL,(BABEh) (alternate HL encoding)
  0x1037,
  0x1038,
  0x1039,
  0x103a,

  // LD IX,(1234h)
  0x103b,
  0x103c,
  0x103d,
  0x103e,

  // LD IY,(1234h)
  0x103f,
  0x1040,
  0x1041,
  0x1042,

  // LD (1234h),HL
  0x1043,
  0x1044,
  0x1045,

  // LD (4567h),SP
  0x1046,
  0x1047,
  0x1048,
  0x1049,

  // LD (CAFEh),IX
  0x104a,
  0x104b,
  0x104c,
  0x104d,

  // LD (CAFEh),IY
  0x104e,
  0x104f,
  0x1050,
  0x1051,

  // LD SP,HL
  0x1052,

  // LD SP,IX
  0x1053,
  0x1054,

  // LD SP,IY
  0x1055,
  0x1056,

  // PUSH BC
  0x1057,

  // PUSH IX
  0x1058,
  0x1059,

  // PUSH IY
  0x105a,
  0x105b,

  // POP AF
  0x105c,

  // POP IX
  0x105d,
  0x105e,

  // POP IY
  0x105f,
  0x1060,

  // EX DE,HL
  0x1061,

  // EX AF,AF'
  0x1062,

  // EXX
  0x1063,

  // EX (SP),HL
  0x1064,

  // EX (SP),IX
  0x1065,
  0x1066,

  // EX (SP),IY
  0x1067,
  0x1068,

  // LDI
  0x1069,
  0x106a,

  // LDIR
  0x106b,
  0x106c,

  // LDD
  0x106d,
  0x106e,

  // LDDR
  0x106f,
  0x1070,

  // CPI
  0x1071,
  0x1072,

  // CPIR
  0x1073,
  0x1074,

  // CPD
  0x1075,
  0x1076,

  // CPDR
  0x1077,
  0x1078,

  // ADD B
  0x1079,

  // ADD 10h
  0x107a,
  0x107b,

  // ADD (HL)
  0x107c,

  // ADD (IX+100)
  0x107d,
  0x107e,
  0x107f,

  // ADC C
  0x1080,

  // SUB (HL)
  0x1081,

  // SBC (IY+8),
  0x1082,
  0x1083,
  0x1084,

  // ADD HL,SP
  0x1085,

  // ADC HL,SP
  0x1086,
  0x1087,

  // SBC HL,SP
  0x1088,
  0x1089,

  // ADD IX,IX
  0x108a,
  0x108b,

  // ADD IY,IY
  0x108c,
  0x108d,

  // INC BC
  0x108e,

  // INC IY
  0x108f,
  0x1090,

  // RLCA
  0x0191,

  // RLC B
  0x1092,
  0x1093,

  // RLC (IX+10)
  0x1094,
  0x1095,
  0x1096,
  0x1097,

  // BIT 1,L
  0x1098,
  0x1099,

  // BIT 6,(IX+0)
  0x109a,
  0x109b,
  0x109c,
  0x109d,

  // JP BEEFh
  0x109e,
  0x109f,
  0x10a0,

  // JP Z,DEADh
  0x10a1,
  0x10a2,
  0x10a3,

  // JR 2 <10A6h>
  0x10a4,
  0x10a5,

  // JR 0 <10A6h>
  0x10a6,
  0x10a7,

  // JR -2 <10A6h>
  0x10a8,
  0x10a9,

  // JP (HL)
  0x10aa,

  // JP (IX)
  0x10ab,
  0x10ac,

  // RETI
  0x10ad,
  0x10ae,

  // RST 18h
  0x10af,

  // IN A,(10h)
  0x10b0,
  0x10b1,

  // IN C,(C)
  0x10b2,
  0x10b3,

  // OUT (C),H
  0x10b4,
  0x10b5,
};

#define FN  (CC_Z80_IORQ | CC_Z80_WR | CC_Z80_RESET | CC_Z80_INT)
#define N   (CC_Z80_IORQ | CC_Z80_WR | CC_Z80_RESET | CC_Z80_INT | CC_Z80_M1)
#define NW  (CC_Z80_IORQ | CC_Z80_RD | CC_Z80_RESET | CC_Z80_INT | CC_Z80_M1)

const uint32_t debug_control[] = {
  // LD A,B
  FN,

  // LD C,55h
  FN, N,

  // LD C,(HL)
  FN,

  // LD E(IX+5)
  FN, N, N,

  // LD H,(IY-10)
  FN, N, N,

  // LD (HL),A
  FN,

  // LD (IX+127),L
  FN, N, N,

  // LD (IY-100),A
  FN, N, N,

  // LD (HL),FFh
  FN, N,

  // LD (IX+10),A5h
  FN, N, N, N,

  // LD A,(BC)
  FN,

  // LD A,(DE)
  FN,

  // LD A,(1234h)
  FN, N, N,

  // LD (BC),A
  FN,

  // LD (DE),A
  FN,

  // LD (4567h),A
  FN, N, N,

  // LD A,I
  FN, N,

  // LD A,R
  FN, N,

  // LD I,A
  FN, N,

  // LD R,A
  FN, N,

  // LD BC,1234h
  FN, N, N,

  // LD IX,4567h
  FN, N, N, N,

  // LD IY,CAFEh
  FN, N, N, N,

  // LD HL,(CAFEh)
  FN, N, N,

  // LD HL,(BABEh) (alternate HL encoding)
  FN, N, N, N,

  // LD IX,(1234h)
  FN, N, N, N,

  // LD IY,(1234h)
  FN, N, N, N,

  // LD (1234h),HL
  FN, N, N,

  // LD (4567h),SP
  FN, N, N, N,

  // LD (CAFEh),IX
  FN, N, N, N,

  // LD (CAFEh),IY
  FN, N, N, N,

  // LD SP,HL
  FN,

  // LD SP,IX
  FN, N,

  // LD SP,IY
  FN, N,

  // PUSH BC
  FN,

  // PUSH IX
  FN, N,

  // PUSH IY
  FN, N,

  // POP AF
  FN,

  // POP IX
  FN, N,

  // POP IY
  FN, N,

  // EX DE,HL
  FN,

  // EX AF,AF'
  FN,

  // EXX
  FN,

  // EX (SP),HL
  FN,

  // EX (SP),IX
  FN, N,

  // EX (SP),IY
  FN, N,

  // LDI
  FN, N,

  // LDIR
  FN, N,

  // LDD
  FN, N,

  // LDDR
  FN, N,

  // CPI
  FN, N,

  // CPIR
  FN, N,

  // CPD
  FN, N,

  // CPDR
  FN, N,

  // ADD B
  FN,

  // ADD 10h
  FN, N,

  // ADD (HL)
  FN,

  // ADD (IX+100)
  FN, N, N,

  // ADC C
  FN,

  // SUB (HL)
  FN,

  // SBC (IY+8)
  FN, N, N,

  // ADD HL,SP
  FN,

  // ADC HL,SP
  FN, N,

  // SBC HL,SP
  FN, N,

  // ADD IX,IX
  FN, N,

  // ADD IY,IY
  FN, N,

  // INC BC
  FN,

  // INC IY
  FN, N,

  // RLCA
  FN,

  // RLC B
  FN, N,

  // RLC (IX+10)
  FN, N, N, N,

  // BIT 1,L
  FN, N,

  // BIT 6,(IX+0)
  FN, N, N, N,

  // JP BEEFh
  FN, N, N,

  // JP Z,DEADh
  FN, N, N,

  // JR 2 <10A6h>
  FN, N,

  // JR 0 <10A6h>
  FN, N,

  // JR -2 <10A6h>
  FN, N,

  // JP (HL)
  FN,

  // JP (IX)
  FN, N,

  // RETI
  FN, N,

  // RST 18h
  FN,

  // IN A,(10h)
  FN, N,

  // IN C,(C)
  FN, N,

  // OUT (C),H
  FN, N,
};

#undef F
#undef N
#undef NW
#endif // DEBUG_Z80

#if defined(DEBUG_6502) || defined(DEBUG_6809) || defined(DEBUG_6809E) || \
    defined(DEBUG_6800) || defined(DEBUG_Z80)
#define DEBUG_SAMPLES
#endif
