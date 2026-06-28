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

// Collision avoidance, "act" half. Under the current architecture the ESP32
// owns the EKF and fleet-state mesh: it predicts conflicts and decides the
// escape maneuver, then sends this flight controller a compact command. The FC
// does not detect anything here; it only applies the commanded lean.
//
// The command carries a body-frame escape attitude (roll/pitch lean) plus an
// active flag. If the ESP32 stops sending (link loss, ESP32 reset), the command
// ages out and control returns to the pilot -- avoidance fails safe to manual.

#define FLEET_AVOID_FLAG_ACTIVE   (1 << 0)
#define FLEET_AVOID_TIMEOUT_MS    500    // refresh deadline; past this we disengage
#define FLEET_AVOID_MAX_ANGLE_CD  3000   // clamp commanded lean to +/-30 deg, centidegrees

// ESP32 -> FC avoidance command. Packed and endian-naive to match the other
// fleet messages; the link layer hands the payload straight through.
typedef struct __attribute__((packed)) {
    uint8_t  type;           // fleetMsgType_e == FLEET_MSG_AVOID_COMMAND
    uint8_t  flags;          // FLEET_AVOID_FLAG_*
    int16_t  rollAngleCd;    // body-frame roll lean, centidegrees (+ = roll right)
    int16_t  pitchAngleCd;   // body-frame pitch lean, centidegrees (+ = nose up / pitch back)
    uint8_t  severity;       // 0..255 urgency, for diagnostics / future ramping
    uint8_t  conflictNodeId; // node we are avoiding, for diagnostics
    uint16_t ttcMs;          // predicted time-to-closest-approach, ms, for diagnostics
} fleetAvoidMessage_t;

void fleetAvoidInit(void);

// Feed an inbound avoid command from the link layer.
void fleetAvoidReceive(const uint8_t *data, uint8_t len);

// Scheduler entry point: publishes diagnostics; command freshness is evaluated
// lazily so the result is correct even between task ticks.
void fleetAvoidUpdate(timeUs_t currentTimeUs);

// True when a fresh, active avoid command is in effect.
bool fleetAvoidIsActive(void);

// Commanded lean for FD_ROLL / FD_PITCH in centidegrees. Returns 0 when no
// fresh command is active, so callers can add it unconditionally.
float fleetAvoidGetAngleCd(int axis);

// Diagnostics for telemetry / blackbox.
uint8_t fleetAvoidGetConflictNodeId(void);
