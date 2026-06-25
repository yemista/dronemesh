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

// RX relay: forwards raw RC channel values out over a configured transmitter as
// they are received. It is a deliberately thin counterpart to rx.c -- rx.c reads
// and processes the incoming link for flight control, while the relay simply
// mirrors the freshly received raw values to an outbound link (e.g. a mesh radio).
// The transmitter itself is abstracted behind a function pointer and stubbed for
// now; the real link is wired in later.

#pragma once

#include "common/time.h"

#include "rx/rx.h"

// A transmitter takes a snapshot of raw RC channel values and sends them out.
// channels[] holds channelCount raw values in the usual [1000;2000] us range.
typedef void (*rxRelayTransmitFnPtr)(const float *channels, uint8_t channelCount, timeUs_t currentTimeUs);

void rxRelayInit(void);
void rxRelaySetTransmitFn(rxRelayTransmitFnPtr transmitFn);

// Called from the RX path (rx.c) whenever a new frame has been received. The
// relay reads the unprocessed channel values straight from the receiver driver
// (no rcmap, no range calibration, no failsafe) and queues them to forward.
void rxRelayOnFrame(timeUs_t currentTimeUs);

// Scheduler hooks for TASK_RX_RELAY (mirrors rxUpdateCheck / taskUpdateRxMain).
bool rxRelayUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
void rxRelayUpdate(timeUs_t currentTimeUs);
