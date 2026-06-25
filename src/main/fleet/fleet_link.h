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

// Fleet mesh link: the concrete transport for inter-drone fleet traffic.
//
// It owns a UART (serial function FUNCTION_FLEET_MESH) and moves framed fleet
// messages over it. The radio itself lives off-board -- e.g. an ESP32 running
// ESP-NOW -- and bridges this UART to the air, so the firmware stays radio
// agnostic. Outbound messages from the fleet sub-protocols are framed and
// written to the UART; inbound bytes are de-framed and dispatched back to the
// sub-protocols by message type.

#pragma once

#include "common/time.h"

// Baud to the off-board radio bridge. Fixed for now; promote to a PG setting if
// a target needs to change it.
#define FLEET_LINK_BAUD 115200

void fleetLinkInit(void);

// Drain the UART, de-frame, and dispatch any complete messages. Called from the
// fleet task.
void fleetLinkUpdate(timeUs_t currentTimeUs);

// Frame and transmit a fleet message. Matches fleetSendFn so it can be installed
// via fleetIdSetSendFn(). No-op if the link has no UART assigned.
void fleetLinkSend(const uint8_t *payload, uint8_t len);
