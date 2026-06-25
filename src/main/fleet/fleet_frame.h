/*
 * This file is part of DroneMesh.
 *
 * DroneMesh is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * DroneMesh is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Framing for the fleet mesh link. The UART carries a byte stream, so messages
// are delimited by a small frame:
//
//   [SYNC=0xFE][len][payload[len]][crc8]
//
// crc8 (DVB-S2, the same polynomial CRSF uses) is computed over the length byte
// and the payload, so both corruption and a wrong length are caught. The codec
// is pure (no I/O) so it can be unit-tested on its own.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FLEET_FRAME_SYNC        0xFE
#define FLEET_FRAME_MAX_PAYLOAD 64
#define FLEET_FRAME_OVERHEAD    3   // sync + len + crc
#define FLEET_FRAME_MAX_SIZE    (FLEET_FRAME_MAX_PAYLOAD + FLEET_FRAME_OVERHEAD)

// Encode payload into out (which must hold at least len + FLEET_FRAME_OVERHEAD
// bytes). Returns the total frame length, or 0 if len exceeds the max payload.
uint8_t fleetFrameEncode(const uint8_t *payload, uint8_t len, uint8_t *out);

typedef enum {
    FLEET_FRAME_STATE_SYNC = 0,
    FLEET_FRAME_STATE_LEN,
    FLEET_FRAME_STATE_PAYLOAD,
    FLEET_FRAME_STATE_CRC,
} fleetFrameState_e;

typedef struct {
    fleetFrameState_e state;
    uint8_t len;
    uint8_t index;
    uint8_t crc;
    uint8_t payload[FLEET_FRAME_MAX_PAYLOAD];
} fleetFrameParser_t;

void fleetFrameParserReset(fleetFrameParser_t *parser);

// Feed one received byte. Returns true when a complete, CRC-valid frame is ready
// in parser->payload (parser->len bytes). The parser resyncs automatically on a
// bad length or CRC.
bool fleetFrameParseByte(fleetFrameParser_t *parser, uint8_t byte);
