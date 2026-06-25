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

#include "common/utils.h"

#include "rx/rx.h"
#include "rx/rx_relay.h"

#include "fleet/fleet_id.h"

// Snapshot of the most recently received raw channels, waiting to be forwarded.
static float relayChannels[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static uint8_t relayChannelCount = 0;
static timeUs_t relayFrameTimeUs = 0;
static bool relayFramePending = false;

// Stub transmitter: drops the frame. Replaced by a real outbound link later via
// rxRelaySetTransmitFn().
static void nullTransmit(const float *channels, uint8_t channelCount, timeUs_t currentTimeUs)
{
    UNUSED(channels);
    UNUSED(channelCount);
    UNUSED(currentTimeUs);
}

static rxRelayTransmitFnPtr transmitFn = nullTransmit;

void rxRelayInit(void)
{
    relayChannelCount = 0;
    relayFramePending = false;
    transmitFn = nullTransmit;
}

void rxRelaySetTransmitFn(rxRelayTransmitFnPtr fn)
{
    transmitFn = fn ? fn : nullTransmit;
}

void rxRelayOnFrame(timeUs_t currentTimeUs)
{
    // Mesh policy: ignore mesh input until this drone holds an assigned fleet ID.
    if (!fleetHasId()) {
        return;
    }

    // Read the unprocessed values directly from the receiver driver, in wire
    // order, with no channel remapping, calibration or failsafe applied.
    const uint8_t count = MIN(rxRuntimeState.channelCount, (uint8_t)MAX_SUPPORTED_RC_CHANNEL_COUNT);
    for (uint8_t i = 0; i < count; i++) {
        relayChannels[i] = rxRuntimeState.rcReadRawFn(&rxRuntimeState, i);
    }
    relayChannelCount = count;
    relayFrameTimeUs = currentTimeUs;
    relayFramePending = true;
}

bool rxRelayUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(currentDeltaTimeUs);

    return relayFramePending;
}

void rxRelayUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Mesh policy: never emit on the mesh without an assigned fleet ID (the ID
    // may have been lost since the frame was captured).
    if (!fleetHasId()) {
        relayFramePending = false;
        return;
    }

    if (!relayFramePending) {
        return;
    }
    relayFramePending = false;

    transmitFn(relayChannels, relayChannelCount, relayFrameTimeUs);
}
