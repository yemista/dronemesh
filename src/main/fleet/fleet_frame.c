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

#include <stdbool.h>
#include <stdint.h>

#include "common/crc.h"

#include "fleet/fleet_frame.h"

uint8_t fleetFrameEncode(const uint8_t *payload, uint8_t len, uint8_t *out)
{
    if (len > FLEET_FRAME_MAX_PAYLOAD) {
        return 0;
    }

    uint8_t i = 0;
    out[i++] = FLEET_FRAME_SYNC;
    out[i++] = len;

    uint8_t crc = crc8_dvb_s2_update(0, &len, 1);
    for (uint8_t j = 0; j < len; j++) {
        out[i++] = payload[j];
    }
    crc = crc8_dvb_s2_update(crc, payload, len);

    out[i++] = crc;
    return i; // len + FLEET_FRAME_OVERHEAD
}

void fleetFrameParserReset(fleetFrameParser_t *parser)
{
    parser->state = FLEET_FRAME_STATE_SYNC;
    parser->len = 0;
    parser->index = 0;
    parser->crc = 0;
}

bool fleetFrameParseByte(fleetFrameParser_t *parser, uint8_t byte)
{
    switch (parser->state) {
    case FLEET_FRAME_STATE_SYNC:
        if (byte == FLEET_FRAME_SYNC) {
            parser->state = FLEET_FRAME_STATE_LEN;
        }
        break;

    case FLEET_FRAME_STATE_LEN:
        if (byte > FLEET_FRAME_MAX_PAYLOAD) {
            // Bogus length. A SYNC here is most likely the real start of a frame
            // (e.g. the previous SYNC was spurious), so resync to read a length
            // next; otherwise wait for a fresh SYNC.
            parser->state = (byte == FLEET_FRAME_SYNC) ? FLEET_FRAME_STATE_LEN : FLEET_FRAME_STATE_SYNC;
            break;
        }
        parser->len = byte;
        parser->index = 0;
        parser->crc = crc8_dvb_s2_update(0, &byte, 1); // crc covers the length byte
        parser->state = (byte == 0) ? FLEET_FRAME_STATE_CRC : FLEET_FRAME_STATE_PAYLOAD;
        break;

    case FLEET_FRAME_STATE_PAYLOAD:
        parser->payload[parser->index++] = byte;
        parser->crc = crc8_dvb_s2_update(parser->crc, &byte, 1);
        if (parser->index >= parser->len) {
            parser->state = FLEET_FRAME_STATE_CRC;
        }
        break;

    case FLEET_FRAME_STATE_CRC:
        if (byte == parser->crc) {
            parser->state = FLEET_FRAME_STATE_SYNC;
            return true; // complete, valid frame in parser->payload
        }
        // CRC mismatch (e.g. the frame was truncated before its CRC). If this
        // byte is a SYNC it likely starts the next frame, so don't swallow it.
        parser->state = (byte == FLEET_FRAME_SYNC) ? FLEET_FRAME_STATE_LEN : FLEET_FRAME_STATE_SYNC;
        break;
    }

    return false;
}
