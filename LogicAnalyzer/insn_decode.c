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
