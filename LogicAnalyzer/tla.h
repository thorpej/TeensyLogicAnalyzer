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

#ifndef tla_h_included
#define tla_h_included

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// Trigger and CPU type definitions
typedef enum { tr_address, tr_data, tr_addr_data, tr_reset, tr_irq, tr_firq, tr_nmi, tr_none } trigger_t;
typedef enum { tr_mem, tr_io } space_t;
typedef enum { tr_read, tr_write, tr_either } cycle_t;
typedef enum { cpu_none, cpu_6502, cpu_65c02, cpu_6800, cpu_6809, cpu_6809e, cpu_z80 } cpu_t;

#if defined(__cplusplus)
extern "C" {
#endif

extern cpu_t cpu;

#if defined(__cplusplus)
}
#endif

#endif /* tla_h_included */
