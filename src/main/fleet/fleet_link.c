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

#include "platform.h"

#include "drivers/serial.h"

#include "io/serial.h"

#include "fleet/fleet_frame.h"
#include "fleet/fleet_id.h"
#include "fleet/fleet_leader.h"
#include "fleet/fleet_link.h"

static serialPort_t *port = NULL;
static fleetFrameParser_t parser;

// Route a complete inbound message to the right sub-protocol by its type byte
// (the first payload byte, a fleetMsgType_e). The ID-assignment protocol is the
// bootstrap and is always dispatched; gating on having an ID happens inside the
// higher-level mesh features, not here.
static void fleetLinkDispatch(const uint8_t *payload, uint8_t len)
{
    if (len < 1) {
        return;
    }

    switch (payload[0]) { // fleetMsgType_e
    case FLEET_MSG_ID_CLAIM:
        fleetIdReceive(payload, len);
        break;
    case FLEET_MSG_LEADER_HEARTBEAT:
        fleetLeaderReceiveHeartbeat(payload, len);
        break;
    default:
        break; // unknown / not-yet-handled message type
    }
}

void fleetLinkSend(const uint8_t *payload, uint8_t len)
{
    if (!port) {
        return;
    }

    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(payload, len, frame);
    if (frameLen == 0) {
        return; // payload too large to frame
    }

    serialWriteBuf(port, frame, frameLen);
}

void fleetLinkUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!port) {
        return;
    }

    uint32_t waiting = serialRxBytesWaiting(port);
    while (waiting--) {
        const uint8_t b = serialRead(port);
        if (fleetFrameParseByte(&parser, b)) {
            fleetLinkDispatch(parser.payload, parser.len);
        }
    }
}

void fleetLinkInit(void)
{
    fleetFrameParserReset(&parser);

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_FLEET_MESH);
    if (!portConfig) {
        return; // no UART assigned -> link stays down, sends are no-ops
    }

    port = openSerialPort(portConfig->identifier, FUNCTION_FLEET_MESH, NULL, NULL,
                          FLEET_LINK_BAUD, MODE_RXTX, SERIAL_NOT_INVERTED);
    if (!port) {
        return;
    }

    // The UART is now the outbound path for fleet sub-protocols. Their *Init()
    // functions run before us and reset their send hooks to stubs, so install
    // ours here.
    fleetIdSetSendFn(fleetLinkSend);
    fleetLeaderSetSendFn(fleetLinkSend);
}
