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
#include "fleet/fleet_leader.h"

// Leader broadcasts at this cadence; a follower declares the leader gone after
// missing a few of them.
#define FLEET_HEARTBEAT_INTERVAL_US (100 * 1000) // 10 Hz
#define FLEET_LEADER_TIMEOUT_US     (300 * 1000) // ~3 missed heartbeats

static bool     isLeader;
static uint8_t  leaderId;         // who we believe leads; FLEET_ID_UNASSIGNED = none
static uint32_t currentTerm;
static uint32_t cachedLiveMask;   // last membership the leader published
static timeUs_t lastHeartbeatRxUs; // last accepted leader heartbeat
static timeUs_t lastHeartbeatTxUs; // last heartbeat we sent (as leader)

static void nullSend(const uint8_t *data, uint8_t len)
{
    UNUSED(data);
    UNUSED(len);
}
static fleetSendFn sendFn = nullSend;

void fleetLeaderInit(void)
{
    isLeader = false;
    leaderId = FLEET_ID_UNASSIGNED;
    currentTerm = 0;
    cachedLiveMask = 0;
    lastHeartbeatRxUs = 0;
    lastHeartbeatTxUs = 0;
    sendFn = nullSend;
}

void fleetLeaderSetSendFn(fleetSendFn fn)
{
    sendFn = fn ? fn : nullSend;
}

static void sendHeartbeat(timeUs_t currentTimeUs)
{
    const fleetHeartbeatMessage_t msg = {
        .type     = FLEET_MSG_LEADER_HEARTBEAT,
        .leaderId = fleetIdGet(),
        .term     = currentTerm,
        .liveMask = fleetMemberMask(), // leader is the membership authority
    };
    sendFn((const uint8_t *)&msg, sizeof(msg));
    lastHeartbeatTxUs = currentTimeUs;
}

void fleetLeaderBecomeLeader(uint32_t term)
{
    isLeader = true;
    currentTerm = term;
    leaderId = fleetIdGet();
    lastHeartbeatRxUs = micros();
    // Make the next update emit a heartbeat immediately.
    lastHeartbeatTxUs = micros() - FLEET_HEARTBEAT_INTERVAL_US;
}

void fleetLeaderReceiveHeartbeat(const uint8_t *data, uint8_t len)
{
    // Mesh policy: ignore mesh input until we hold an assigned fleet ID.
    if (!fleetHasId()) {
        return;
    }
    if (len < sizeof(fleetHeartbeatMessage_t)) {
        return;
    }

    fleetHeartbeatMessage_t msg;
    memcpy(&msg, data, sizeof(msg));

    if (msg.type != FLEET_MSG_LEADER_HEARTBEAT) {
        return;
    }
    if (msg.leaderId == fleetIdGet()) {
        return; // our own heartbeat echoed back by the link layer
    }
    if (msg.term < currentTerm) {
        return; // stale leader from an older term
    }

    if (msg.term > currentTerm) {
        // A newer term supersedes us: adopt it and step down if we were leading.
        currentTerm = msg.term;
        isLeader = false;
    }

    // Accept this leader for the current term.
    leaderId = msg.leaderId;
    cachedLiveMask = msg.liveMask;
    lastHeartbeatRxUs = micros();
}

void fleetLeaderUpdate(timeUs_t currentTimeUs)
{
    // Mesh policy: take no part in leadership without an assigned fleet ID.
    if (!fleetHasId()) {
        return;
    }

    if (isLeader) {
        if (cmpTimeUs(currentTimeUs, lastHeartbeatTxUs) >= FLEET_HEARTBEAT_INTERVAL_US) {
            sendHeartbeat(currentTimeUs);
        }
        return;
    }

    // Follower: drop the leader if its heartbeats have stopped. (Election will
    // hook in here to start a new term once there is no leader.)
    if (leaderId != FLEET_ID_UNASSIGNED &&
        cmpTimeUs(currentTimeUs, lastHeartbeatRxUs) >= FLEET_LEADER_TIMEOUT_US) {
        leaderId = FLEET_ID_UNASSIGNED;
    }
}

bool fleetHasLeader(void)
{
    if (isLeader) {
        return true;
    }
    return leaderId != FLEET_ID_UNASSIGNED &&
           cmpTimeUs(micros(), lastHeartbeatRxUs) < FLEET_LEADER_TIMEOUT_US;
}

bool fleetIsLeader(void)
{
    return isLeader;
}

uint8_t fleetLeaderId(void)
{
    return leaderId;
}

uint32_t fleetLeaderTerm(void)
{
    return currentTerm;
}

uint32_t fleetLeaderLiveMask(void)
{
    // As leader we are the source of truth; otherwise report what the leader last
    // published (the cached value election uses while the leader is gone).
    return isLeader ? fleetMemberMask() : cachedLiveMask;
}
