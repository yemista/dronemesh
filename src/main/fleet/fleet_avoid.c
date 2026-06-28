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

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fleet/fleet_id.h"
#include "fleet/fleet_avoid.h"

static bool commandActive;            // active flag from the last command
static uint32_t lastRxMs;             // when that command arrived
static float commandAngleCd[2];       // [FD_ROLL], [FD_PITCH], centidegrees
static uint8_t conflictNodeId;
static uint16_t ttcMs;

// A command is "live" only if it asked for avoidance AND arrived recently. The
// freshness test is evaluated at query time so a stalled scheduler cannot leave
// avoidance latched on -- it decays on its own.
static bool avoidanceIsLive(void)
{
    return commandActive && (millis() - lastRxMs) < FLEET_AVOID_TIMEOUT_MS;
}

void fleetAvoidInit(void)
{
    commandActive = false;
    lastRxMs = 0;
    commandAngleCd[FD_ROLL] = 0.0f;
    commandAngleCd[FD_PITCH] = 0.0f;
    conflictNodeId = 0;
    ttcMs = 0;
}

void fleetAvoidReceive(const uint8_t *data, uint8_t len)
{
    if (!data || len < sizeof(fleetAvoidMessage_t)) {
        return;
    }

    fleetAvoidMessage_t msg;
    memcpy(&msg, data, sizeof(msg));

    if (msg.type != FLEET_MSG_AVOID_COMMAND) {
        return;
    }

    // Honor the fleet-wide rule: ignore mesh traffic until we hold a node ID.
    if (!fleetHasId()) {
        return;
    }

    commandActive = (msg.flags & FLEET_AVOID_FLAG_ACTIVE) != 0;
    commandAngleCd[FD_ROLL] = constrainf(msg.rollAngleCd, -FLEET_AVOID_MAX_ANGLE_CD, FLEET_AVOID_MAX_ANGLE_CD);
    commandAngleCd[FD_PITCH] = constrainf(msg.pitchAngleCd, -FLEET_AVOID_MAX_ANGLE_CD, FLEET_AVOID_MAX_ANGLE_CD);
    conflictNodeId = msg.conflictNodeId;
    ttcMs = msg.ttcMs;
    lastRxMs = millis();
}

void fleetAvoidUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    const bool live = avoidanceIsLive();
    DEBUG_SET(DEBUG_FLEET_AVOID, 0, live ? 1 : 0);
    DEBUG_SET(DEBUG_FLEET_AVOID, 1, lrintf(commandAngleCd[FD_ROLL]));
    DEBUG_SET(DEBUG_FLEET_AVOID, 2, lrintf(commandAngleCd[FD_PITCH]));
    DEBUG_SET(DEBUG_FLEET_AVOID, 3, conflictNodeId);
}

bool fleetAvoidIsActive(void)
{
    return avoidanceIsLive();
}

float fleetAvoidGetAngleCd(int axis)
{
    if ((axis != FD_ROLL && axis != FD_PITCH) || !avoidanceIsLive()) {
        return 0.0f;
    }
    return commandAngleCd[axis];
}

uint8_t fleetAvoidGetConflictNodeId(void)
{
    return conflictNodeId;
}
