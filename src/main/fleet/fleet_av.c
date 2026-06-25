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

#include "platform.h"

#include "common/utils.h"

#include "fleet/fleet_leader.h"
#include "fleet/fleet_av.h"

// Applied transmitter state: -1 = not yet applied, 0 = off, 1 = on. Starting at
// -1 forces the first update to drive the hook into a known state (off, since no
// drone is leader at boot).
static int8_t videoApplied;
static int8_t audioApplied;

static bool nullControl(bool enable)
{
    UNUSED(enable);
    return true; // no-op always "succeeds"
}
static fleetAvControlFn videoControlFn = nullControl;
static fleetAvControlFn audioControlFn = nullControl;

void fleetAvInit(void)
{
    videoApplied = -1;
    audioApplied = -1;
    videoControlFn = nullControl;
    audioControlFn = nullControl;
}

void fleetAvSetVideoControlFn(fleetAvControlFn fn)
{
    videoControlFn = fn ? fn : nullControl;
}

void fleetAvSetAudioControlFn(fleetAvControlFn fn)
{
    audioControlFn = fn ? fn : nullControl;
}

bool fleetAvVideoAllowed(void)
{
    return fleetIsLeader();
}

bool fleetAvAudioAllowed(void)
{
    return fleetIsLeader();
}

// Drive a stream's transmitter to the desired state, but only when it changes.
// If the hook reports it could not apply (e.g. VTX not ready), leave the state
// un-applied so the next update retries.
static void applyStream(bool allowed, int8_t *applied, fleetAvControlFn fn)
{
    const int8_t desired = allowed ? 1 : 0;
    if (desired != *applied && fn(allowed)) {
        *applied = desired;
    }
}

void fleetAvUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    applyStream(fleetAvVideoAllowed(), &videoApplied, videoControlFn);
    applyStream(fleetAvAudioAllowed(), &audioApplied, audioControlFn);
}
