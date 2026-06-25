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
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "fleet/fleet_id.h"

// Protocol timing.
#define FLEET_ID_PROBE_INTERVAL_US   (200 * 1000)   // gap between probes while claiming
#define FLEET_ID_PROBE_COUNT         3              // uncontested probes needed to settle
#define FLEET_ID_ANNOUNCE_INTERVAL_US (1000 * 1000) // re-announce/defend cadence once claimed

// Our identity.
static uint32_t nodeUid;                // hardware tiebreaker (lowest wins)
static uint8_t  nodeId = FLEET_ID_UNASSIGNED;
static fleetIdState_e state = FLEET_ID_STATE_PROBING;

// Probing/announcing timers.
static uint8_t  probesSent;
static timeUs_t lastTxUs;

// Bitset of IDs we have heard claimed by *other* drones, used to bias candidate
// selection toward an apparently-free ID. Bit (id-1) corresponds to ID id.
static uint32_t observedIds;

// Link layer.
static void nullSend(const uint8_t *data, uint8_t len)
{
    UNUSED(data);
    UNUSED(len);
}
static fleetSendFn sendFn = nullSend;

static void markObserved(uint8_t id)
{
    if (id >= 1 && id <= FLEET_ID_MAX) {
        observedIds |= (1u << (id - 1));
    }
}

static bool isObserved(uint8_t id)
{
    return (id >= 1 && id <= FLEET_ID_MAX) && (observedIds & (1u << (id - 1)));
}

// Deterministically pick a candidate ID, preferring one we have not seen taken.
// Starts from a UID-derived offset so different drones spread out, then linear
// probes the ID space. Falls back to the UID-derived slot if everything looks
// taken (conflicts will then sort it out).
static uint8_t pickCandidateId(void)
{
    const uint8_t start = (uint8_t)(nodeUid % FLEET_ID_MAX) + 1; // 1..FLEET_ID_MAX
    for (uint8_t i = 0; i < FLEET_ID_MAX; i++) {
        const uint8_t candidate = (uint8_t)(((start - 1 + i) % FLEET_ID_MAX) + 1);
        if (!isObserved(candidate)) {
            return candidate;
        }
    }
    return start;
}

static void sendClaim(timeUs_t currentTimeUs)
{
    const fleetIdMessage_t msg = {
        .type   = FLEET_MSG_ID_CLAIM,
        .nodeId = nodeId,
        .state  = (uint8_t)state,
        .uid    = nodeUid,
    };
    sendFn((const uint8_t *)&msg, sizeof(msg));
    lastTxUs = currentTimeUs;
}

static void enterProbing(uint8_t candidate, timeUs_t currentTimeUs)
{
    nodeId = candidate;
    state = FLEET_ID_STATE_PROBING;
    probesSent = 0;
    // Stagger the first probe so freshly-conflicting nodes don't resend in lockstep.
    lastTxUs = currentTimeUs - (timeUs_t)(nodeUid % FLEET_ID_PROBE_INTERVAL_US);
}

void fleetIdInit(void)
{
#ifdef U_ID_0
    nodeUid = U_ID_0 ^ U_ID_1 ^ U_ID_2;
#else
    nodeUid = 0;    // no MCU UID (e.g. SITL); override via fleetIdSetUid()
#endif
    observedIds = 0;
    sendFn = nullSend;
    enterProbing(pickCandidateId(), 0);
}

void fleetIdSetSendFn(fleetSendFn fn)
{
    sendFn = fn ? fn : nullSend;
}

void fleetIdSetUid(uint32_t uid)
{
    nodeUid = uid;
    // Re-derive a candidate from the new identity if we have not settled yet.
    if (state == FLEET_ID_STATE_PROBING) {
        enterProbing(pickCandidateId(), micros());
    }
}

// Decide whether an incoming claim for *our* ID outranks us. Lower UID wins; a
// settled (CLAIMED) drone outranks a still-probing one regardless of UID.
static bool remoteOutranksUs(const fleetIdMessage_t *msg)
{
    const fleetIdState_e remoteState = (fleetIdState_e)msg->state;

    if (remoteState == FLEET_ID_STATE_CLAIMED && state == FLEET_ID_STATE_PROBING) {
        return true;
    }
    if (remoteState == FLEET_ID_STATE_PROBING && state == FLEET_ID_STATE_CLAIMED) {
        return false;
    }
    return msg->uid < nodeUid;  // same lifecycle stage: lowest UID wins
}

void fleetIdReceive(const uint8_t *data, uint8_t len)
{
    if (len < sizeof(fleetIdMessage_t)) {
        return;
    }

    fleetIdMessage_t msg;
    memcpy(&msg, data, sizeof(msg));

    if (msg.type != FLEET_MSG_ID_CLAIM) {
        return;
    }
    if (msg.uid == nodeUid) {
        return; // our own claim echoed back by the link layer
    }

    markObserved(msg.nodeId);

    if (msg.nodeId != nodeId) {
        return; // no conflict with us
    }

    // Conflict on our ID.
    if (remoteOutranksUs(&msg)) {
        // Yield: that ID is taken, pick another and start over.
        markObserved(nodeId);
        enterProbing(pickCandidateId(), micros());
    } else if (state == FLEET_ID_STATE_CLAIMED) {
        // We hold the ID and outrank them: defend it immediately so they yield.
        sendClaim(micros());
    }
    // If we are probing and outrank them, we simply keep probing; they will yield.
}

void fleetIdUpdate(timeUs_t currentTimeUs)
{
    switch (state) {
    case FLEET_ID_STATE_PROBING:
        if (cmpTimeUs(currentTimeUs, lastTxUs) >= FLEET_ID_PROBE_INTERVAL_US) {
            sendClaim(currentTimeUs);
            if (++probesSent >= FLEET_ID_PROBE_COUNT) {
                // Survived probing uncontested -> claim it.
                state = FLEET_ID_STATE_CLAIMED;
                sendClaim(currentTimeUs); // initial announce
            }
        }
        break;

    case FLEET_ID_STATE_CLAIMED:
        if (cmpTimeUs(currentTimeUs, lastTxUs) >= FLEET_ID_ANNOUNCE_INTERVAL_US) {
            sendClaim(currentTimeUs);
        }
        break;
    }
}

bool fleetIdIsResolved(void)
{
    return state == FLEET_ID_STATE_CLAIMED;
}

uint8_t fleetIdGet(void)
{
    return nodeId;
}
