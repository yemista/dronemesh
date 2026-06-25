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

// Fleet node-ID consensus.
//
// The first piece of fleet consensus: every drone agrees on a unique node ID
// without any central coordinator, ARP-style. Each drone tentatively claims an
// ID, broadcasts that claim, and listens for conflicts. When two drones claim
// the same ID, a deterministic tiebreak (lowest hardware UID wins) decides who
// keeps it; the loser picks another ID and tries again. Once a claim goes
// uncontested for a short settling period the ID is considered resolved.
//
// This is modelled loosely on IPv4 link-local address selection (RFC 3927):
// probe -> announce -> defend, with conflict-driven re-selection.
//
// The transport is deliberately abstracted. The fleet logic only produces and
// consumes opaque byte buffers; a concrete link (mesh radio, DroneCAN, ...) is
// plugged in later via fleetIdSetSendFn() / fleetIdReceive().

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

// Node IDs run 1..FLEET_ID_MAX. 0 means "not yet assigned"; 0xFF is reserved
// for broadcast addressing by higher layers.
#define FLEET_ID_UNASSIGNED 0
#define FLEET_ID_BROADCAST  0xFF
#define FLEET_ID_MAX        32

typedef enum {
    FLEET_MSG_ID_CLAIM = 1,     // "I intend to use / am defending this node ID"
} fleetMsgType_e;

// Claimant lifecycle, also carried on the wire so a settled (defending) drone
// can win against a still-probing one.
typedef enum {
    FLEET_ID_STATE_PROBING = 0, // tentatively claiming, listening for conflicts
    FLEET_ID_STATE_CLAIMED,     // holding and defending the ID
} fleetIdState_e;

// On-wire claim message. Packed and endian-naive for now; a real link layer can
// (de)serialise explicitly once the transport is chosen.
typedef struct __attribute__((packed)) {
    uint8_t  type;              // fleetMsgType_e
    uint8_t  nodeId;            // the ID being claimed
    uint8_t  state;            // fleetIdState_e of the claimant
    uint32_t uid;              // claimant's hardware UID, the conflict tiebreaker
} fleetIdMessage_t;

// Link-layer broadcast hook. Sends len bytes to every other drone in the fleet.
typedef void (*fleetSendFn)(const uint8_t *data, uint8_t len);

void fleetIdInit(void);
void fleetIdSetSendFn(fleetSendFn sendFn);

// Override the tiebreaker UID. Useful for tests/SITL where the MCU UID is not
// available or where several simulated nodes must differ.
void fleetIdSetUid(uint32_t uid);

// Feed an inbound claim message from the link layer.
void fleetIdReceive(const uint8_t *data, uint8_t len);

// Scheduler entry point (TASK_FLEET): drives probing/announcing timers.
void fleetIdUpdate(timeUs_t currentTimeUs);

// Results.
bool    fleetIdIsResolved(void);    // true once we hold an uncontested ID
uint8_t fleetIdGet(void);           // our node ID, or FLEET_ID_UNASSIGNED

// Canonical gate for all mesh functionality. Mesh code must ignore every mesh
// command/input unless this returns true. The one exception is the ID-assignment
// protocol itself (fleetIdReceive/fleetIdUpdate), which runs in order to *get*
// an ID and therefore cannot gate on already having one.
bool    fleetHasId(void);
