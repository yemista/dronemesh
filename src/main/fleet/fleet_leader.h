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

// Fleet leadership state.
//
// One drone is the leader. It periodically broadcasts a heartbeat carrying its
// fleet ID, the current election term, and the authoritative live-member bitmap
// (the leader owns membership; followers just cache what it publishes). Every
// drone tracks how long since it last heard the leader, so it can tell whether a
// leader exists and notice when one dies.
//
// Raft term rules apply: a higher term always wins, and hearing a higher term
// makes a leader step down. Election (which actually promotes a drone to leader)
// is built on top of this in a later step; for now leadership is entered via
// fleetLeaderBecomeLeader(), which the election will call after winning a term.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#include "fleet/fleet_id.h" // fleetSendFn, fleetMsgType_e

// On-wire leader heartbeat.
typedef struct __attribute__((packed)) {
    uint8_t  type;      // FLEET_MSG_LEADER_HEARTBEAT
    uint8_t  leaderId;  // leader's fleet ID
    uint32_t term;      // current election term
    uint32_t liveMask;  // authoritative live-member bitmask (bit id-1)
} fleetHeartbeatMessage_t;

void fleetLeaderInit(void);
void fleetLeaderSetSendFn(fleetSendFn sendFn);

void fleetLeaderUpdate(timeUs_t currentTimeUs);
void fleetLeaderReceiveHeartbeat(const uint8_t *data, uint8_t len);

// Enter leadership for the given term. Election calls this after winning; it can
// also be used directly in tests/bring-up.
void fleetLeaderBecomeLeader(uint32_t term);

// Queries.
bool     fleetHasLeader(void);       // a leader is currently known/alive
bool     fleetIsLeader(void);        // this drone is the current leader
uint8_t  fleetLeaderId(void);        // current leader's fleet ID, or FLEET_ID_UNASSIGNED
uint32_t fleetLeaderTerm(void);      // current election term
uint32_t fleetLeaderLiveMask(void);  // authoritative live-member bitmask
