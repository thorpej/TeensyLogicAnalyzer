/*

  Logic Analyzer for 6502, 6800, 6809, or Z80 microprocessors based on a
  Teensy 4.1 microcontroller.

  See https://github.com/thorpej/TeensyLogicAnalyzer

  Based on https://github.com/jefftranter/6502/tree/master/LogicAnalyzer

  Copyright (c) 2021-2022 by Jeff Tranter <tranter@pobox.com>
  Copyright (c) 2022 by Jason R. Thorpe <thorpej@me.com>

  To Do:
  - Add support for BS, BA, /HALT, and /FIRQ lines on 6809 & 6809E.
  - Add support for LIC on 6809E.
  - Add support for Z80 control line triggers.
  - Add support for Z80 I/O read or write trigger.

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
typedef enum { tr_address, tr_io, tr_data, tr_reset, tr_irq, tr_nmi, tr_none } trigger_t;
typedef enum { tr_read, tr_write, tr_either } cycle_t;
typedef enum { cpu_6502, cpu_65c02, cpu_6800, cpu_6809, cpu_z80 } cpu_t;

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

// Instructions for 65C02 disassembler.
const char *opcodes_65c02[256] = {
  "BRK", "ORA (nn,X)", "?", "?", "TSB nn", "ORA nn", "ASL nn", "RMB0 nn",
  "PHP", "ORA #nn", "ASLA", "?", "TSB XXXX", "ORA nnnn", "ASL nnnn", "BBR0 nn",
  "BPL rr", "ORA (nn),Y", "ORA (nn)", "?", "TRB nn", "ORA nn,X", "ASL nn,X", "RMB1 nn",
  "CLC", "ORA nnnn,Y", "INCA", "?", "TRB nn", "ORA nnnn,X", "ASL nnnn,X", "BBR1 nn",
  "JSR nnnn", "AND (nn,X)", "?", "?", "BIT nn", "AND nn", "ROL nn", "RMB2 nn",
  "PLP", "AND #nn", "ROLA", "?", "BIT nnnn", "AND nnnn", "ROL nnnn", "BBR2 nn",
  "BMI rr", "AND (nn),Y", "AND (nn)", "?", "BIT nn,X", "AND nn,X", "ROL nn,X", "RMB3 nn",
  "SEC", "AND nnnn,Y", "DECA", "?", "BIT nn,X", "AND nnnn,X", "ROL nnnn,X", "BBR3 nn",
  "RTI", "EOR (nn,X)", "?", "?", "?", "EOR nn", "LSR nn", "RMB4 nn",
  "PHA", "EOR #nn", "LSRA", "?", "JMP nnnn", "EOR nnnn", "LSR nnnn", "BBR4 nn",
  "BVC rr", "EOR (nn),Y", "EOR (nn)", "?", "?", "EOR nn,X", "LSR nn,X", "RMB5 nn",
  "CLI", "EOR nnnn,Y", "PHY", "?", "?", "EOR nnnn,X", "LSR nnnn,X", "BBR5 nn",
  "RTS", "ADC (nn,X)", "?", "?", "STZ nn", "ADC nn", "ROR nn", "RMB6 nn",
  "PLA", "ADC #nn", "RORA", "?", "JMP (nnnn)", "ADC nnnn", "ROR nnnn", "BBR6 nn",
  "BVS rr", "ADC (nn),Y", "ADC (nn)", "?", "STZ nn,X", "ADC nn,X", "ROR nn,X", "RMB7 nn",
  "SEI", "ADC nnnn,Y", "PLY", "?", "JMP (nn,X)", "ADC nnnn,X", "ROR nnnn,X", "BBR7 nn",
  "BRA rr", "STA (nn,X)", "?", "?", "STY nn", "STA nn", "STX nn", "SMB0 nn",
  "DEY", "BIT #nn", "TXA", "?", "STY nnnn", "STA nnnn", "STX nnnn", "BBS0 nn",
  "BCC rr", "STA (nn),Y", "STA (nn)", "?", "STY nn,X", "STA nn,X", "STX (nn),Y", "SMB1 nn",
  "TYA", "STA nnnn,Y", "TXS", "?", "STZ nn", "STA nnnn,X", "STZ nn,X", "BBS1 nn",
  "LDY #nn", "LDA (nn,X)", "LDX #nn", "?", "LDY nn", "LDA nnnn", "LDX nn", "SMB2 nn",
  "TAY", "LDA #nn", "TAX", "?", "LDY nnnn", "LDA nnnn", "LDX nnnn", "BBS2 nn",
  "BCS rr", "LDA (nn),Y", "LDA (nn)", "?", "LDY nn,X", "LDA nn,X", "LDX (nn),Y", "SMB3 nn",
  "CLV", "LDA nnnn,Y", "TSX", "?", "LDY nnnn,X", "LDA nnnn,X", "LDX nnnn,Y", "BBS3 nn",
  "CPY #nn", "CMP (nn,X)", "?", "?", "CPY nnnn", "CMP nnnn", "DEC nnnn", "SMB4 nn",
  "INY", "CMP #nn", "DEX", "WAI", "CPY nn", "CMP nn", "DEC nn", "BBS4 nn",
  "BNE rr", "CMP (nn),Y", "CMP (nn)", "?", "?", "CMP nn,X", "DEC nn,X", "SMB5 nn",
  "CLD", "CMP nnnn,Y", "PHX", "STP", "?", "CMP nnnn,X", "DEC nnnn,X", "BBS5 nn",
  "CPX #nn", "SBC (nn,X)", "?", "?", "CPX nn", "SBC nn", "INC nn", "SMB6 nn",
  "INX", "SBC #nn", "NOP", "?", "CPX nnnn", "SBC nnnn", "INC nnnn", "BBS6 nn",
  "BEQ rr", "SBC (nn),Y", "SBC (nn)", "?", "?", "SBC nn,X", "INC nn,X", "SMB7 nn",
  "SED", "SBC nnnn,Y", "PLX", "?", "?", "SBC nnnn,X", "INC nnnn,X", "BBS7 nnnn"
};

// Instructions for 6502 disassembler.
const char *opcodes_6502[256] = {
  "BRK", "ORA (nn,X)", "?", "?", "?", "ORA nn", "ASL nn", "?",
  "PHP", "ORA #nn", "ASLA", "?", "?", "ORA nnnn", "ASL nnnn", "?",
  "BPL rr", "ORA (nn),Y", "?", "?", "?", "ORA nn,X", "ASL nn,X", "?",
  "CLC", "ORA nnnn,Y", "?", "?", "?", "ORA nnnn,X", "ASL nnnn,X", "?",
  "JSR nnnn", "AND (nn,X)", "?", "?", "BIT nn", "AND nn", "ROL nn", "?",
  "PLP", "AND #nn", "ROLA", "?", "BIT nnnn", "AND nnnn", "ROL nnnn", "?",
  "BMI rr", "AND (nn),Y", "?", "?", "?", "AND nn,X", "ROL nn,X", "?",
  "SEC", "AND nnnn,Y", "?", "?", "?", "AND nnnn,X", "ROL nnnn,X", "?",
  "RTI", "EOR (nn,X)", "?", "?", "?", "EOR nn", "LSR nn", "?",
  "PHA", "EOR #nn", "LSRA", "?", "JMP nnnn", "EOR nnnn", "LSR nnnn", "?",
  "BVC rr", "EOR (nn),Y", "?", "?", "?", "EOR nn,X", "LSR nn,X", "?",
  "CLI", "EOR nnnn,Y", "?", "?", "?", "EOR nnnn,X", "LSR nnnn,X", "?",
  "RTS", "ADC (nn,X)", "?", "?", "?", "ADC nn", "ROR nn", "?",
  "PLA", "ADC #nn", "RORA", "?", "JMP (nnnn)", "ADC nnnn", "ROR nnnn", "?",
  "BVS rr", "ADC (nn),Y", "?", "?", "?", "ADC nn,X", "ROR nn,X", "?",
  "SEI", "ADC nnnn,Y", "?", "?", "?", "ADC nnnn,X", "ROR nnnn,X", "?",
  "?", "STA (nn,X)", "?", "?", "STY nn", "STA nn", "STX nn", "?",
  "DEY", "?", "TXA", "?", "STY nnnn", "STA nnnn", "STX nnnn", "?",
  "BCC rr", "STA (nn),Y", "?", "?", "STY nn,X", "STA nn,X", "STX nn,Y", "?",
  "TYA", "STA nnnn,Y", "TXS", "?", "?", "STA nnnn,X", "?", "?",
  "LDY #nn", "LDA (nn,X)", "LDX #nn", "?", "LDY nn", "LDA nn", "LDX nn", "?",
  "TAY", "LDA #nn", "TAX", "?", "LDY nnnn", "LDA nnnn", "LDX nnnn", "?",
  "BCS rr", "LDA (nn),Y", "?", "?", "LDY nn,X", "LDA nn,X", "LDX nn,Y", "?",
  "CLV", "LDA nnnn,Y", "TSX", "?", "LDY nnnn,X", "LDA nnnn,X", "LDX nnnn,Y", "?",
  "CPY #nn", "CMP (nn,X)", "?", "?", "CPY nn", "CMP nn", "DEC nn", "?",
  "INY", "CMP #nn", "DEX", "?", "CPY nnnn", "CMP nnnn", "DEC nnnn", "?",
  "BNE rr", "CMP (nn),Y", "?", "?", "?", "CMP nn,X", "DEC nn,X", "?",
  "CLD", "CMP nnnn,Y", "?", "?", "?", "CMP nnnn,X", "DEC nnnn,X", "?",
  "CPX #nn", "SBC (nn,X)", "?", "?", "CPX nn", "SBC nn", "INC nn", "?",
  "INX", "SBC #nn", "NOP", "?", "CPX nnnn", "SBC nnnn", "INC nnnn", "?",
  "BEQ rr", "SBC (nn),Y", "?", "?", "?", "SBC nn,X", "INC nn,X", "?",
  "SED", "SBC nnnn,Y", "?", "?", "?", "SBC nnnn,X", "INC nnnn,X", "?"
};

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

// Instructions for 6800 disassembler.
const char *opcodes_6800[256] = {
  "?", "NOP", "?", "?", "?", "?", "TAP", "TPA",
  "INX", "DEX", "CLV", "SEV", "CLC", "SEC", "CLI", "SEI",
  "SBA", "CBA", "?", "?", "?", "?", "TAB", "TBA",
  "?", "DAA", "?", "ABA", "?", "?", "?", "?",
  "BRA", "?", "BHI", "BLS", "BCC", "BCS", "BNE", "BEQ",
  "BVC", "BVS", "BPL", "BMI", "BGE", "BLT", "BGT", "BLE",
  "TSX", "INS", "PULA", "PULB", "DES", "TXS", "PSHA", "PSHB",
  "?", "RTS", "?", "RTI", "?", "?", "WAI", "SWI",
  "NEGA", "?", "?", "COMA", "LSRA", "?", "RORA", "ASRA",
  "ASLA", "ROLA", "DECA", "?", "INCA", "TSTA", "?", "CLRA",
  "NEGB", "?", "?", "COMB", "LSRB", "?", "RORB", "ASRB",
  "ASLB", "ROLB", "DECB", "?", "INCB", "TSTB", "?", "CLRB",
  "NEG", "?", "?", "COM", "LSR", "?", "ROR", "ASR",
  "ASL", "ROL", "DEC", "?", "INC", "TST", "JMP", "CLR",
  "NEG", "?", "?", "COM", "LSR", "?", "ROR", "ASR",
  "ASL", "ROL", "DEC", "?", "INC", "TST", "JMP", "CLR",
  "SUBA", "CMPA", "SBCA", "?", "ANDA", "BITA", "LDAA", "?",
  "EORA", "ADCA", "ORAA", "ADDA", "CPX", "BSR", "LDS", "?",
  "SUBA", "CMPA", "SBCA", "?", "ANDA", "BITA", "LDAA", "STAA",
  "EORA", "ADCA", "ORAA", "ADDA", "CPX", "?", "LDS", "STS",
  "SUBA", "CMPA", "SBCA", "?", "ANDA", "BITA", "LDAA", "STAA",
  "EORA", "ADCA", "ORAA", "ADDA", "CPX", "JSR", "LDS", "STS",
  "SUBA", "CMPA", "SBCA", "?", "ANDA", "BITA", "LDAA", "STAA",
  "EORA", "ADCA", "ORAA", "ADDA", "CPX", "JSR", "LDS", "STS",
  "SUBB", "CMPB", "SBCB", "?", "ANDB", "BITB", "LDAB", "?",
  "EORB", "ADCB", "ORAB", "ADDB", "?", "?", "LDX", "?",
  "SUBB", "CMPB", "SBCB", "?", "ANDB", "BITB", "LDAB", "STAB",
  "EORB", "ADCB", "ORAB", "ADDB", "?", "?", "LDX", "STX",
  "SUBB", "CMPB", "SBCB", "?", "ANDB", "BITB", "LDAB", "STAB",
  "EORB", "ADCB", "ORAB", "ADDB", "?", "?", "LDX", "STX",
  "SUBB", "CMPB", "SBCB", "?", "ANDB", "BITB", "LDAB", "STAB",
  "EORB", "ADCB", "ORAB", "ADDB", "?", "?", "LDX", "STX"
};

// Instructions for 6809 disassembler.
const char *opcodes_6809[256] = {
  "NEG", "?", "?", "COMB", "LSR", "?", "ROR", "ASR",
  "ASL", "ROL", "DEC", "?", "INC", "TST", "JMP", "CLR",
  "(extended)", "(extended)", "NOP", "SYNC", "?", "?", "LBRA", "LBSR",
  "?", "DAA", "ORCC", "?", "ANDCC", "SEX", "EXG", "TFR",
  "BRA", "BRN", "BHI", "BLS", "BCC", "BCS", "BNE", "BEQ",
  "BVC", "BVS", "BPL", "BMI", "BGE", "BLT", "BGT", "BLE",
  "LEAX", "LEAY", "LEAS", "LEAU", "PSHS", "PULS", "PSHU", "PULU",
  "?", "RTS", "ABX", "RTI", "CWAI", "MUL", "?", "SWI",
  "NEGA", "?", "?", "COMA", "LSRA", "?", "RORA", "ASRA",
  "ASLA", "ROLA", "DECA", "?", "INCA", "TSTA", "?", "CLRA",
  "NEGB", "?", "?", "COMB", "LSRB", "?", "RORB", "ASRB",
  "ASLB", "ROLB", "DECB", "?", "INCB", "TSTB", "?", "CLRB",
  "NEG", "?", "?", "COMB", "LSR", "?", "ROR", "ASR",
  "ASL", "ROL", "DEC", "?", "INC", "TST", "JMP", "CLR",
  "NEG", "?", "?", "COMB", "LSR", "?", "ROR", "ASR",
  "ASL", "ROL", "DEC", "?", "INC", "TST", "JMP", "CLR",
  "SUBA", "CMPA", "SBCA", "SUBD", "ANDA", "BITA", "LDA", "?",
  "EORA", "ADCA", "ORA", "ADDA", "CMPX", "BSR", "LDX", "?",
  "SUBA", "CMPA", "SBCA", "SUBD", "ANDA", "BITA", "LDA", "STA",
  "EORA", "ADCA", "ORA", "ADDA", "CMPX", "JSR", "LDX", "STX",
  "SUBA", "CMPA", "SBCA", "SUBD", "ANDA", "BITA", "LDA", "STA",
  "EORA", "ADCA", "ORA", "ADDA", "CMPX", "JSR", "LDX", "STX",
  "SUBA", "CMPA", "SBCA", "SUBD", "ANDA", "BITA", "LDA", "STA",
  "EORA", "ADCA", "ORA", "ADDA", "CMPX", "JSR", "LDX", "STX",
  "SUBB", "CMPB", "SBCB", "ADDD", "ANDB", "BITB", "LDB", "?",
  "EORB", "ADCB", "ORB", "ADDB", "LDD", "?", "LDU", "?",
  "SUBB", "CMPB", "SBCB", "ADDD", "ANDB", "BITB", "LDB", "STB",
  "EORB", "ADCB", "ORB", "ADDB", "LDD", "STD", "LDU", "STU",
  "SUBB", "CMPB", "SBCB", "ADDD", "ANDB", "BITB", "LDB", "STB",
  "EORB", "ADCB", "ORB", "ADDB", "LDD", "STD", "LDU", "STU",
  "SUBB", "CMPB", "SBCB", "ADDD", "ANDB", "BITB", "LDB", "STB",
  "EORB", "ADCB", "ORB", "ADDB", "LDD", "STD", "LDU", "STU"
};

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
// 28 (3.18)
// 29 (4.31) - CC4
// 30 -- data bus direction
// 31 -- trigger button
// 32 (2.12) - CD7
// 33 (4.7)  - CC5
// 34 (2.29) - CC7
// 35 (2.28)
// 36 (2.18)
// 37 (2.19)
// 38 (1.28) - CC6
// 39 (1.29)
// 40 (1.20)
// 41 (1.21)

#define DBUS_DIR_PIN      30
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

#define CC0_BITMASK       (1U << 0)
#define CC1_BITMASK       (1U << 1)
#define CC2_BITMASK       (1U << 2)
#define CC3_BITMASK       (1U << 3)
#define CC4_BITMASK       (1U << 4)
#define CC5_BITMASK       (1U << 5)
#define CC6_BITMASK       (1U << 6)
#define CC7_BITMASK       (1U << 7)
// XXX  CC8_BITMASK       (1U << 8)

#define CC_6502_PHI2      CC0_BITMASK
#define CC_6502_SYNC      CC1_BITMASK
#define CC_6502_RW        CC2_BITMASK
#define CC_6502_RESET     CC3_BITMASK
#define CC_6502_IRQ       CC4_BITMASK
#define CC_6502_NMI       CC5_BITMASK

#define CC_6502_PHI2_PIN  CC0_PIN
#define CC_6502_RW_PIN    CC2_PIN
#define CC_6502_RESET_PIN CC3_PIN

#define CC_6800_PHI2      CC0_BITMASK         // same as 6502
#define CC_6800_VMA       CC1_BITMASK
#define CC_6800_RW        CC2_BITMASK         // same as 6502
#define CC_6800_RESET     CC3_BITMASK         // same as 6502
#define CC_6800_IRQ       CC4_BITMASK         // same as 6502
#define CC_6800_NMI       CC5_BITMASK         // same as 6502

#define CC_6800_PHI2_PIN  CC0_PIN
#define CC_6800_RW_PIN    CC2_PIN
#define CC_6800_RESET_PIN CC3_PIN

#define CC_6809_E         CC0_BITMASK
#define CC_6809_Q         CC1_BITMASK
#define CC_6809_RW        CC2_BITMASK         // same as 6502
#define CC_6809_RESET     CC3_BITMASK         // same as 6502
#define CC_6809_IRQ       CC4_BITMASK         // same as 6502
#define CC_6809_NMI       CC5_BITMASK         // same as 6502
// XXX  CC_6809_FIRQ      CC6_BITMASK
// XXX  CC_6809_BA        CC7_BITMASK
// XXX  CC_6809E_LIC      CC8_BITMASK

#define CC_6809_E_PIN     CC0_PIN
#define CC_6809_Q_PIN     CC1_PIN
#define CC_6809_RW_PIN    CC2_PIN
#define CC_6809_RESET_PIN CC3_PIN

#define CC_Z80_CLK        CC0_BITMASK
#define CC_Z80_M1         CC1_BITMASK
#define CC_Z80_MREQ       CC2_BITMASK
#define CC_Z80_IORQ       CC3_BITMASK
#define CC_Z80_RD         CC4_BITMASK
#define CC_Z80_WR         CC5_BITMASK
#define CC_Z80_RESET      CC6_BITMASK
#define CC_Z80_INT        CC7_BITMASK
// XXX  CC_Z80_NMI        CC8_BITMASK

#define CC_Z80_CLK_PIN    CC0_PIN
#define CC_Z80_M1_PIN     CC1_PIN
#define CC_Z80_MREQ_PIN   CC2_PIN
#define CC_Z80_IORQ_PIN   CC3_PIN
#define CC_Z80_RD_PIN     CC4_PIN
#define CC_Z80_WR_PIN     CC5_PIN
#define CC_Z80_INT_PIN    CC7_PIN

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

int32_t
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
  if (cc & CC7_BITMASK) {
    *dregp |= CC7_PIN_BITMASK;
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
  return ((creg & CC0_PIN_BITMASK) ? CC0_BITMASK : 0) |
         ((creg & CC1_PIN_BITMASK) ? CC1_BITMASK : 0) |
         ((creg & CC2_PIN_BITMASK) ? CC2_BITMASK : 0) |
         ((creg & CC3_PIN_BITMASK) ? CC3_BITMASK : 0) |
         ((creg & CC4_PIN_BITMASK) ? CC4_BITMASK : 0) |
         ((creg & CC5_PIN_BITMASK) ? CC5_BITMASK : 0) |
         // CC6 is in the areg
         ((areg & CC6_PIN_BITMASK) ? CC6_BITMASK : 0) |
         // CC7 is in the dreg
         ((dreg & CC7_PIN_BITMASK) ? CC7_BITMASK : 0);
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

// Startup function
void setup() {

  // Enable pullups so unused pins go to a known (high) level.
  for (int i = 0; i <= 41; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  // Will use on-board LED to indicate triggering.
  pinMode(CORE_LED0_PIN, OUTPUT);

  // Manual trigger button - low on this pin forces a trigger.
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), triggerButton, FALLING);

  // Data bus direction - output low to default to reading data bus.
  pinMode(DBUS_DIR_PIN, OUTPUT);
  digitalWriteFast(DBUS_DIR_PIN, LOW);

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
void triggerButton()
{
  triggerPressed = true;
}


// Display settings and help info.
void help()
{
  Serial.println(versionString);

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
  if (cpu != cpu_z80) {
    Serial.println("t nmi 0|1            - Trigger on /NMI level");
    Serial.println("t spare1 0|1         - Trigger on SPARE1 level");
    Serial.println("t spare2 0|1         - Trigger on SPARE2 level");
  }
  Serial.println("t none               - Trigger freerun");
  Serial.println("g                    - Go/start analyzer");
  Serial.println("l [start] [end]      - List samples");
  Serial.println("e                    - Export samples as CSV");
  Serial.println("w                    - Write data to SD card");
  Serial.println("h or ?               - Show command usage");
}


// List recorded data from start to end.
void list(Stream &stream, int start, int end)
{
  char output[50]; // Holds output string

  int first = (triggerPoint - pretrigger + samples) % samples;
  int last = (triggerPoint - pretrigger + samples - 1) % samples;

  // Display data
  int i = first;
  int j = 0;
  while (true) {
    const char *cycle = "";
    const char *opcode = "";
    const char *comment = "";

    if ((j >= start) && (j <= end)) {

      // 6502 SYNC high indicates opcode/instruction fetch, otherwise
      // show as read or write.
      if ((cpu == cpu_65c02) || (cpu == cpu_6502)) {
        if  (control[i] & 0x10) {
          cycle = "F";
          if (cpu == cpu_65c02) {
            opcode = opcodes_65c02[data[i]];
          } else {
            opcode = opcodes_6502[data[i]];
          }
          String s = opcode;
          // Fill in operands
          if (s.indexOf("nnnn") != -1) { // absolute
            char op[5];
            sprintf(op, "$%04lX", data[i + 1] + 256 * data[i + 2]);
            s.replace("nnnn", op);
          }
          if (s.indexOf("nn") != -1) { // page zero
            char op[3];
            sprintf(op, "$%02lX", data[i + 1]);
            s.replace("nn", op);
          }
          if (s.indexOf("rr") != -1) { // relative branch
            char op[3];
            if (data[i + 1] < 0x80) {
              sprintf(op, "$%04lX", address[i] + 2 + data[i + 1]);
            } else {
              sprintf(op, "$%04lX", address[i] + 2 - (256 - data[i + 1]));
            }
            s.replace("rr", op);
          }
          opcode = s.c_str();

        } else if (control[i] & 0x08) {
          cycle = "R";
          opcode = "";
        } else {
          cycle = "W";
          opcode = "";
        }
      }

      if (cpu == cpu_6809) {
        if (control[i] & 0x08) {
          cycle = "R";
          opcode = opcodes_6809[data[i]];
        } else {
          cycle = "W";
          opcode = "";
        }
      }

      if (cpu == cpu_z80) {
        // /M1 /MREQ  /IORQ /RD /WR
        //  1    0      1    0   1   Memory read
        //  1    0      1    1   0   Memory write
        //  0    0      1    0   1   Instruction fetch
        //  1    1      0    0   1   I/O read
        //  1    1      0    1   0   I/O write

        if (!(control[i] & 0x10)) {
          cycle = "F";
          opcode = opcodes_z80[data[i]];
        } else if (!(control[i] & 0x08) && !(control[i] & 0x02)) {
          cycle = "R";
          opcode = "";
        } else if (!(control[i] & 0x08) && !(control[i] & 0x01)) {
          cycle = "W";
          opcode = "";
        } else if (!(control[i] & 0x04) && !(control[i] & 0x02)) {
          cycle = "IR";
          opcode = "";
        } else if (!(control[i] & 0x04) && !(control[i] & 0x01)) {
          cycle = "IW";
          opcode = "";
        } else {
          cycle = " ";
          opcode = "";
        }
      }

      if (cpu == cpu_6800) {
        // VMA R/W
        //  0   X  Internal cycle
        //  1   0  Memory read
        //  1   1  Memory write
        if (!(control[i] & 0x10)) {
          cycle = "-";
          opcode = "";
        } else {
          if (control[i] & 0x08) {
            cycle = "R";
            opcode = opcodes_6800[data[i]];
          } else {
            cycle = "W";
            opcode = "";
          }
        }
      }

      // Check for 6502 /RESET, /IRQ, or /NMI active, vector address, or
      // stack access
      if ((cpu == cpu_65c02) || (cpu == cpu_6502)) {
        if (!(control[i] & 0x04)) {
          comment = "RESET ACTIVE";
        } else if (!(control[i] & 0x02)) {
          comment = "IRQ ACTIVE";
        } else if (!(control[i] & 0x01)) {
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
        if (!(control[i] & 0x04)) {
          comment = "RESET ACTIVE";
        } else if (!(control[i] & 0x02)) {
          comment = "IRQ ACTIVE";
        } else if (!(control[i] & 0x01)) {
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
      if (cpu == cpu_6809) {
        if (!(control[i] & 0x04)) {
          comment = "RESET ACTIVE";
        } else if (!(control[i] & 0x02)) {
          comment = "IRQ ACTIVE";
        } else if (!(control[i] & 0x01)) {
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
        if (!(control[i] & 0x20)) {
          comment = "RESET ACTIVE";
        } else if (!(control[i] & 0x40)) {
          comment = "INT ACTIVE";
        } else {
          comment = "";
        }
      }

      // Indicate when trigger happened
      if (i == triggerPoint) {
        comment = "<--- TRIGGER ----";
      }

      sprintf(output, "%04lX  %-2s  %02lX  %-12s  %s",
              address[i], cycle, data[i], opcode, comment
             );

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
void exportCSV(Stream &stream)
{
  bool sync;
  bool rw = false;
  bool reset = false;
  bool irq = false;
  bool nmi = false;
  bool vma = false;
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
    stream.println("Index,R/W,/RESET,/IRQ,/NMI,Address,Data");
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
      sync = control[i] & 0x10;
    }
    if ((cpu == cpu_65c02) || (cpu == cpu_6502) || (cpu == cpu_6800) || (cpu == cpu_6809)) {
      rw = control[i] & 0x08;
      reset = control[i] & 0x04;
      irq = control[i] & 0x02;
      nmi = control[i] & 0x01;
    }
    if (cpu == cpu_6800) {
      vma = control[i] & 0x10;
    }
    if (cpu == cpu_z80) {
      wr = control[i] & 0x01;
      rd = control[i] & 0x02;
      iorq = control[i] & 0x04;
      mreq = control[i] & 0x08;
      m1 = control[i] & 0x10;
      reset = control[i] & 0x20;
      intr = control[i] & 0x40;
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
      sprintf(output, "%d,%c,%c,%c,%c,%04lX,%02lX",
              j,
              rw ? '1' : '0',
              reset ? '1' : '0',
              irq ? '1' : '0',
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
void writeSD()
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
void go()
{
  aTriggerBits = 0;
  aTriggerMask = 0;
  dTriggerBits = 0;
  dTriggerMask = 0;
  cTriggerBits = 0;
  cTriggerMask = 0;

  uint32_t which_c_trigger = 0;
  
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
  } else if (triggerMode == tr_nmi) {
    if (cpu != cpu_z80) {
      // 6502, 6800, 6809 -- all 6800-like
      which_c_trigger = CC_6800_NMI;
    } else {
      // which_c_trigger = CC_Z80_NMI;
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
    if (cpu == cpu_6809) {
      // Wait for Q to go from low to high
      WAIT_Q_LOW;
      WAIT_Q_HIGH;
    }
    if (cpu == cpu_z80) {
      // Wait CLK to go from high to low
      WAIT_CLK_HIGH;
      WAIT_CLK_LOW;
    }

    // Read address and control lines
    control[i] = CCxx_PSR;
    address[i] = CAxx_PSR;

    if ((cpu == cpu_65c02) || (cpu == cpu_6502) || (cpu == cpu_6800)) {
      // Wait for PHI2 to go from high to low
      WAIT_PHI2_HIGH;
      WAIT_PHI2_LOW;
    }
    if (cpu == cpu_6809) {
      // Wait for E to go from high to low
      WAIT_E_HIGH;
      WAIT_E_LOW;
    }
    if (cpu == cpu_z80) {
      // Wait CLK to go from low to high
      WAIT_CLK_LOW;
      WAIT_CLK_HIGH;
    }

    // Read data lines
    data[i] = CDxx_PSR;

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

  Serial.print("Data recorded (");
  Serial.print(samples);
  Serial.println(" samples).");
  unscramble();
}

void loop() {
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
