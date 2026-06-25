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

#include <stdint.h>

#include <vector>

extern "C" {
    #include "fleet/fleet_id.h"
    #include "fleet/fleet_leader.h"
    #include "fleet/fleet_av.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    static timeUs_t testTimeUs = 0;
    timeUs_t micros(void) { return testTimeUs; }
    uint32_t millis(void) { return testTimeUs / 1000; }
}

static std::vector<bool> videoCalls;
static std::vector<bool> audioCalls;

static bool videoControl(bool enable) { videoCalls.push_back(enable); return true; }
static bool audioControl(bool enable) { audioCalls.push_back(enable); return true; }

static void driveIdResolved()
{
    for (int i = 0; i < 50 && !fleetIdIsResolved(); i++) {
        testTimeUs += 200000;
        fleetIdUpdate(testTimeUs);
    }
}

static void stepDownViaHigherTerm()
{
    fleetHeartbeatMessage_t hb;
    hb.type = FLEET_MSG_LEADER_HEARTBEAT;
    hb.leaderId = 5;
    hb.term = 99;
    hb.liveMask = 0xFF;
    fleetLeaderReceiveHeartbeat((const uint8_t *)&hb, sizeof(hb));
}

class FleetAvTest : public ::testing::Test {
protected:
    void SetUp() override {
        testTimeUs = 0;
        videoCalls.clear();
        audioCalls.clear();
        fleetIdInit();
        fleetIdSetUid(10);
        driveIdResolved();   // hold an ID so we are able to lead
        fleetLeaderInit();
        fleetAvInit();
        fleetAvSetVideoControlFn(videoControl);
        fleetAvSetAudioControlFn(audioControl);
    }
};

// Default state: not the leader -> transmitter is driven OFF on every drone.
TEST_F(FleetAvTest, OffByDefaultWhenNotLeader)
{
    ASSERT_FALSE(fleetIsLeader());
    EXPECT_FALSE(fleetAvVideoAllowed());
    EXPECT_FALSE(fleetAvAudioAllowed());

    fleetAvUpdate(testTimeUs);
    ASSERT_EQ(videoCalls.size(), (size_t)1);
    EXPECT_FALSE(videoCalls[0]);   // VTX off
    ASSERT_EQ(audioCalls.size(), (size_t)1);
    EXPECT_FALSE(audioCalls[0]);
}

// Becoming the leader turns the transmitter on.
TEST_F(FleetAvTest, BecomingLeaderTurnsOn)
{
    fleetAvUpdate(testTimeUs);     // start off as follower
    ASSERT_FALSE(videoCalls.back());

    fleetLeaderBecomeLeader(1);
    EXPECT_TRUE(fleetAvVideoAllowed());

    fleetAvUpdate(testTimeUs);
    EXPECT_TRUE(videoCalls.back()); // VTX on
    EXPECT_TRUE(audioCalls.back());
}

// The hook fires only on a change, not every tick.
TEST_F(FleetAvTest, AppliesOnlyOnChange)
{
    fleetAvUpdate(testTimeUs);     // -> off (call 1)
    fleetAvUpdate(testTimeUs);     // no change
    fleetAvUpdate(testTimeUs);
    EXPECT_EQ(videoCalls.size(), (size_t)1);

    fleetLeaderBecomeLeader(1);
    fleetAvUpdate(testTimeUs);     // -> on (call 2)
    ASSERT_EQ(videoCalls.size(), (size_t)2);
    EXPECT_FALSE(videoCalls[0]);
    EXPECT_TRUE(videoCalls[1]);
}

// Stepping down (a higher term appears) turns the transmitter back off.
TEST_F(FleetAvTest, SteppingDownTurnsOff)
{
    fleetLeaderBecomeLeader(1);
    fleetAvUpdate(testTimeUs);
    ASSERT_TRUE(videoCalls.back()); // on as leader

    stepDownViaHigherTerm();
    ASSERT_FALSE(fleetIsLeader());

    fleetAvUpdate(testTimeUs);
    EXPECT_FALSE(videoCalls.back()); // back off
}

// A hook that reports "not ready" is retried until it succeeds, then left alone.
TEST_F(FleetAvTest, RetriesUntilControlReady)
{
    static int calls;
    calls = 0;
    fleetAvSetVideoControlFn([](bool enable) -> bool {
        (void)enable;
        calls++;
        return calls >= 3; // first two attempts "not ready"
    });

    fleetLeaderBecomeLeader(1);     // want: on

    fleetAvUpdate(testTimeUs);      // attempt 1 -> not ready
    fleetAvUpdate(testTimeUs);      // attempt 2 -> not ready
    fleetAvUpdate(testTimeUs);      // attempt 3 -> applied
    fleetAvUpdate(testTimeUs);      // already applied -> no further call
    EXPECT_EQ(calls, 3);
}
