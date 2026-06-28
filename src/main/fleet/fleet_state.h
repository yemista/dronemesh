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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "fleet/fleet_id.h"

typedef enum {
    FLEET_STATE_MODE_INVALID = 0,
    FLEET_STATE_MODE_NORMAL = 1,
    FLEET_STATE_MODE_AVOID = 2,
    FLEET_STATE_MODE_HOLD = 3,
} fleetStateMode_e;

typedef struct {
    uint8_t nodeId;
    bool valid;
    int32_t posX;
    int32_t posY;
    int32_t posZ;
    int32_t velX;
    int32_t velY;
    int32_t velZ;
    int16_t yawDecideg;
    uint8_t mode;
} fleetStateSample_t;

typedef struct __attribute__((packed)) {
    uint8_t type;              // fleetMsgType_e
    uint8_t nodeId;
    uint8_t mode;
    uint8_t reserved;
    int32_t posX;
    int32_t posY;
    int32_t posZ;
    int32_t velX;
    int32_t velY;
    int32_t velZ;
    int16_t yawDecideg;
    uint32_t timestampMs;
} fleetStateMessage_t;

void fleetStateInit(void);
void fleetStateSetSendFn(fleetSendFn sendFn);

// Update the local state sample used for broadcasting and collision checks.
void fleetStateSetLocalState(int32_t posX, int32_t posY, int32_t posZ,
                             int32_t velX, int32_t velY, int32_t velZ,
                             int16_t yawDecideg, uint8_t mode, bool valid);

// Feed an inbound fleet-state message from the link layer.
void fleetStateReceive(const uint8_t *data, uint8_t len);

// Scheduler entry point: broadcast local state, age remote entries, and check collisions.
void fleetStateUpdate(timeUs_t currentTimeUs);

// Pure geometry check for two state samples. Returns true if they are predicted to
// come within safetyDistanceCm over the next horizonS seconds.
bool fleetStateEvaluateCollision(const fleetStateSample_t *selfState,
                                 const fleetStateSample_t *otherState,
                                 float safetyDistanceCm,
                                 float horizonS,
                                 float *minDistanceCm,
                                 float *timeToCollisionS);

// Latest conflict information from the last update pass.
bool fleetStateHasCollision(void);
uint8_t fleetStateCollisionNodeId(void);
float fleetStateCollisionDistanceCm(void);
float fleetStateCollisionTimeS(void);
