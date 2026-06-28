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
#include <math.h>

#include "platform.h"

#include "common/time.h"
#include "common/utils.h"
#include "drivers/time.h"

#include "flight/imu.h"
#include "flight/position_estimator.h"

#include "fleet/fleet_id.h"
#include "fleet/fleet_state.h"

#define FLEET_STATE_BROADCAST_INTERVAL_US (200 * 1000) // 5 Hz
#define FLEET_STATE_ENTRY_TIMEOUT_MS      1000
#define FLEET_STATE_COLLISION_HORIZON_S   3.0f
#define FLEET_STATE_SAFETY_DISTANCE_CM   200.0f

static fleetSendFn sendFn = NULL;
static fleetStateSample_t localState;
static fleetStateSample_t remoteStates[8];
static uint32_t remoteLastSeenMs[8];
static bool collisionTriggered;
static uint8_t collisionNodeId;
static float collisionDistanceCm;
static float collisionTimeS;

static void clearRemoteState(uint8_t index)
{
    remoteStates[index].valid = false;
    remoteStates[index].nodeId = 0;
}

static bool isRemoteSlotFree(uint8_t index)
{
    return !remoteStates[index].valid;
}

static void markRemoteSeen(uint8_t index, const fleetStateSample_t *sample)
{
    remoteStates[index] = *sample;
    remoteLastSeenMs[index] = millis();
}

static void ageRemoteStates(void)
{
    const uint32_t nowMs = millis();
    for (uint8_t i = 0; i < 8; i++) {
        if (remoteStates[i].valid && (nowMs - remoteLastSeenMs[i]) >= FLEET_STATE_ENTRY_TIMEOUT_MS) {
            clearRemoteState(i);
        }
    }
}

static void sendStateBroadcast(timeUs_t currentTimeUs)
{
    static timeUs_t lastTxUs = 0;
    if (cmpTimeUs(currentTimeUs, lastTxUs) < FLEET_STATE_BROADCAST_INTERVAL_US) {
        return;
    }

    lastTxUs = currentTimeUs;

    if (!sendFn || !localState.valid) {
        return;
    }

    fleetStateMessage_t msg = {
        .type = FLEET_MSG_STATE_SAMPLE,
        .nodeId = fleetIdGet(),
        .mode = localState.mode,
        .posX = localState.posX,
        .posY = localState.posY,
        .posZ = localState.posZ,
        .velX = localState.velX,
        .velY = localState.velY,
        .velZ = localState.velZ,
        .yawDecideg = localState.yawDecideg,
        .timestampMs = millis(),
    };
    sendFn((const uint8_t *)&msg, sizeof(msg));
}

void fleetStateInit(void)
{
    memset(&localState, 0, sizeof(localState));
    memset(remoteStates, 0, sizeof(remoteStates));
    memset(remoteLastSeenMs, 0, sizeof(remoteLastSeenMs));
    collisionTriggered = false;
    collisionNodeId = 0;
    collisionDistanceCm = 0.0f;
    collisionTimeS = 0.0f;
    sendFn = NULL;
}

void fleetStateSetSendFn(fleetSendFn sendFnArg)
{
    sendFn = sendFnArg;
}

void fleetStateSetLocalState(int32_t posX, int32_t posY, int32_t posZ,
                             int32_t velX, int32_t velY, int32_t velZ,
                             int16_t yawDecideg, uint8_t mode, bool valid)
{
    localState.nodeId = fleetIdGet();
    localState.valid = valid;
    localState.posX = posX;
    localState.posY = posY;
    localState.posZ = posZ;
    localState.velX = velX;
    localState.velY = velY;
    localState.velZ = velZ;
    localState.yawDecideg = yawDecideg;
    localState.mode = mode;
}

void fleetStateReceive(const uint8_t *data, uint8_t len)
{
    if (!data || len < sizeof(fleetStateMessage_t)) {
        return;
    }

    fleetStateMessage_t msg;
    memcpy(&msg, data, sizeof(msg));

    if (msg.type != FLEET_MSG_STATE_SAMPLE) {
        return;
    }

    if (!fleetHasId()) {
        return;
    }

    if (msg.nodeId == fleetIdGet()) {
        return;
    }

    fleetStateSample_t sample = {
        .nodeId = msg.nodeId,
        .valid = true,
        .posX = msg.posX,
        .posY = msg.posY,
        .posZ = msg.posZ,
        .velX = msg.velX,
        .velY = msg.velY,
        .velZ = msg.velZ,
        .yawDecideg = msg.yawDecideg,
        .mode = msg.mode,
    };

    for (uint8_t i = 0; i < 8; i++) {
        if (remoteStates[i].valid && remoteStates[i].nodeId == msg.nodeId) {
            markRemoteSeen(i, &sample);
            return;
        }
    }

    for (uint8_t i = 0; i < 8; i++) {
        if (isRemoteSlotFree(i)) {
            markRemoteSeen(i, &sample);
            return;
        }
    }
}

void fleetStateUpdate(timeUs_t currentTimeUs)
{
    const positionEstimate3d_t *estimate = positionEstimatorGetEstimate();
    if (estimate) {
        const bool validXY = positionEstimatorIsValidXY();
        const bool validZ = positionEstimatorIsValidZ();
        const uint8_t mode = fleetStateHasCollision() ? FLEET_STATE_MODE_AVOID : FLEET_STATE_MODE_NORMAL;
        fleetStateSetLocalState(lrintf(estimate->position.x),
                                lrintf(estimate->position.y),
                                lrintf(estimate->position.z),
                                lrintf(estimate->velocity.x),
                                lrintf(estimate->velocity.y),
                                lrintf(estimate->velocity.z),
                                attitude.values.yaw,
                                mode,
                                validXY || validZ);
    }

    ageRemoteStates();
    sendStateBroadcast(currentTimeUs);

    collisionTriggered = false;
    collisionNodeId = 0;
    collisionDistanceCm = 0.0f;
    collisionTimeS = 0.0f;

    if (!localState.valid) {
        return;
    }

    for (uint8_t i = 0; i < 8; i++) {
        fleetStateSample_t *remote = &remoteStates[i];
        if (!remote->valid) {
            continue;
        }
        float minDistanceCm = 0.0f;
        float timeToCollisionS = 0.0f;
        if (fleetStateEvaluateCollision(&localState, remote, FLEET_STATE_SAFETY_DISTANCE_CM,
                                         FLEET_STATE_COLLISION_HORIZON_S,
                                         &minDistanceCm, &timeToCollisionS)) {
            collisionTriggered = true;
            collisionNodeId = remote->nodeId;
            collisionDistanceCm = minDistanceCm;
            collisionTimeS = timeToCollisionS;
            break;
        }
    }
}

bool fleetStateEvaluateCollision(const fleetStateSample_t *selfState,
                                 const fleetStateSample_t *otherState,
                                 float safetyDistanceCm,
                                 float horizonS,
                                 float *minDistanceCm,
                                 float *timeToCollisionS)
{
    if (!selfState || !otherState || !selfState->valid || !otherState->valid || !minDistanceCm || !timeToCollisionS) {
        return false;
    }

    const float dx = (float)(selfState->posX - otherState->posX);
    const float dy = (float)(selfState->posY - otherState->posY);
    const float dz = (float)(selfState->posZ - otherState->posZ);
    const float vx = (float)(selfState->velX - otherState->velX);
    const float vy = (float)(selfState->velY - otherState->velY);
    const float vz = (float)(selfState->velZ - otherState->velZ);

    const float v2 = vx * vx + vy * vy + vz * vz;
    if (v2 < 1.0e-6f) {
        *minDistanceCm = sqrtf(dx * dx + dy * dy + dz * dz);
        *timeToCollisionS = 0.0f;
        return *minDistanceCm <= safetyDistanceCm;
    }

    const float tca = -(dx * vx + dy * vy + dz * vz) / v2;
    if (tca < 0.0f || tca > horizonS) {
        *minDistanceCm = sqrtf(dx * dx + dy * dy + dz * dz);
        *timeToCollisionS = tca;
        return false;
    }

    const float closestDx = dx + vx * tca;
    const float closestDy = dy + vy * tca;
    const float closestDz = dz + vz * tca;
    *minDistanceCm = sqrtf(closestDx * closestDx + closestDy * closestDy + closestDz * closestDz);
    *timeToCollisionS = tca;
    return *minDistanceCm <= safetyDistanceCm;
}

bool fleetStateHasCollision(void)
{
    return collisionTriggered;
}

uint8_t fleetStateCollisionNodeId(void)
{
    return collisionNodeId;
}

float fleetStateCollisionDistanceCm(void)
{
    return collisionDistanceCm;
}

float fleetStateCollisionTimeS(void)
{
    return collisionTimeS;
}
