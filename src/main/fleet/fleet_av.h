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

// Fleet audio/video gating.
//
// On a shared FPV link only one drone may transmit. The rule is simple: a
// stream transmits only while this drone is the fleet leader. By default -- when
// no drone is the leader -- the transmitter is OFF on every drone; becoming the
// leader turns it on, and stepping down turns it back off. The actual
// transmitter (VTX, audio path) is abstracted behind a control hook so this
// module stays hardware agnostic: it decides *whether* to transmit, the hook
// decides *how*.
//
// Decision per stream: allowed = fleetIsLeader().
// The hook is only invoked when the allow/deny state changes, so it is safe to
// wire to a real VTX pit-mode toggle without spamming it every tick.

#pragma once

#include <stdbool.h>

#include "common/time.h"

// Enable/disable a transmitter. enable == true means "transmit". Returns true if
// the change took effect; returning false (e.g. a VTX that is not ready yet)
// leaves the state un-applied so it is retried on the next update.
typedef bool (*fleetAvControlFn)(bool enable);

void fleetAvInit(void);

// Install the hooks that actually start/stop each stream's transmitter.
void fleetAvSetVideoControlFn(fleetAvControlFn fn);
void fleetAvSetAudioControlFn(fleetAvControlFn fn);

// Re-evaluate and apply the gating; call from the fleet task.
void fleetAvUpdate(timeUs_t currentTimeUs);

// Current decisions (also useful for OSD/telemetry).
bool fleetAvVideoAllowed(void);
bool fleetAvAudioAllowed(void);
