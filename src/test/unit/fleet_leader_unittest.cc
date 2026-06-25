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
#include <string.h>

#include <vector>

extern "C" {
    #include "fleet/fleet_id.h"
    #include "fleet/fleet_leader.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    static timeUs_t testTimeUs = 0;
    timeUs_t micros(void) { return testTimeUs; }
    uint32_t millis(void) { return testTimeUs / 1000; }
}

static std::vector<fleetHeartbeatMessage_t> sentHbs;

static void testSend(const uint8_t *data, uint8_t len)
{
    if (len >= sizeof(fleetHeartbeatMessage_t)) {
        fleetHeartbeatMessage_t m;
        memcpy(&m, data, sizeof(m));
        sentHbs.push_back(m);
    }
}

static void driveIdResolved()
{
    for (int i = 0; i < 50 && !fleetIdIsResolved(); i++) {
        testTimeUs += 200000;
        fleetIdUpdate(testTimeUs);
    }
}

static void deliverHeartbeat(uint8_t leaderId, uint32_t term, uint32_t liveMask)
{
    fleetHeartbeatMessage_t m;
    m.type = FLEET_MSG_LEADER_HEARTBEAT;
    m.leaderId = leaderId;
    m.term = term;
    m.liveMask = liveMask;
    fleetLeaderReceiveHeartbeat((const uint8_t *)&m, sizeof(m));
}

class FleetLeaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        testTimeUs = 0;
        sentHbs.clear();
        fleetIdInit();
        fleetIdSetUid(10);   // -> fleet ID 11
        driveIdResolved();   // settle the ID so fleetHasId() is true
        fleetLeaderInit();
        fleetLeaderSetSendFn(testSend);
    }
};

TEST_F(FleetLeaderTest, NoLeaderInitially)
{
    EXPECT_FALSE(fleetHasLeader());
    EXPECT_FALSE(fleetIsLeader());
    EXPECT_EQ(fleetLeaderId(), FLEET_ID_UNASSIGNED);
}

TEST_F(FleetLeaderTest, BecomingLeaderBroadcastsHeartbeat)
{
    fleetLeaderBecomeLeader(1);
    EXPECT_TRUE(fleetIsLeader());
    EXPECT_TRUE(fleetHasLeader());

    sentHbs.clear();
    fleetLeaderUpdate(testTimeUs);

    ASSERT_EQ(sentHbs.size(), (size_t)1);
    EXPECT_EQ(sentHbs[0].type, FLEET_MSG_LEADER_HEARTBEAT);
    EXPECT_EQ(sentHbs[0].leaderId, fleetIdGet());      // 11
    EXPECT_EQ(sentHbs[0].term, 1u);
    EXPECT_TRUE(sentHbs[0].liveMask & (1u << (11 - 1))); // membership includes self
}

TEST_F(FleetLeaderTest, AdoptsHigherTermLeader)
{
    deliverHeartbeat(5, 2, 0x00000ABCu);

    EXPECT_TRUE(fleetHasLeader());
    EXPECT_FALSE(fleetIsLeader());
    EXPECT_EQ(fleetLeaderId(), 5);
    EXPECT_EQ(fleetLeaderTerm(), 2u);
    EXPECT_EQ(fleetLeaderLiveMask(), 0x00000ABCu); // followers cache the leader's mask
}

TEST_F(FleetLeaderTest, IgnoresStaleLowerTerm)
{
    deliverHeartbeat(5, 5, 0xFFu);   // current leader, term 5
    ASSERT_EQ(fleetLeaderId(), 5);

    deliverHeartbeat(7, 3, 0x11u);   // an older term shows up

    EXPECT_EQ(fleetLeaderId(), 5);   // unchanged
    EXPECT_EQ(fleetLeaderTerm(), 5u);
}

TEST_F(FleetLeaderTest, LeaderTimesOutWhenSilent)
{
    deliverHeartbeat(5, 2, 0xFFu);
    EXPECT_TRUE(fleetHasLeader());

    testTimeUs += 400000; // > FLEET_LEADER_TIMEOUT_US (300ms)
    fleetLeaderUpdate(testTimeUs);

    EXPECT_FALSE(fleetHasLeader());
    EXPECT_EQ(fleetLeaderId(), FLEET_ID_UNASSIGNED);
}

TEST_F(FleetLeaderTest, StepsDownOnHigherTerm)
{
    fleetLeaderBecomeLeader(1);
    ASSERT_TRUE(fleetIsLeader());

    deliverHeartbeat(5, 2, 0x22u); // someone won a newer term

    EXPECT_FALSE(fleetIsLeader());
    EXPECT_EQ(fleetLeaderId(), 5);
    EXPECT_EQ(fleetLeaderTerm(), 2u);
}

TEST_F(FleetLeaderTest, IgnoresOwnHeartbeatEcho)
{
    fleetLeaderBecomeLeader(1);

    // The link echoes our own heartbeat back (same leaderId == our ID).
    deliverHeartbeat(fleetIdGet(), 9, 0xFFu);

    EXPECT_TRUE(fleetIsLeader());      // not fooled into stepping down
    EXPECT_EQ(fleetLeaderTerm(), 1u);  // term unchanged
}

TEST_F(FleetLeaderTest, IgnoresHeartbeatWithoutAssignedId)
{
    // Re-init the ID protocol so we are still probing (no assigned ID yet).
    fleetIdInit();
    fleetLeaderInit();
    fleetLeaderSetSendFn(testSend);
    ASSERT_FALSE(fleetHasId());

    deliverHeartbeat(5, 2, 0xFFu);

    EXPECT_FALSE(fleetHasLeader());
    EXPECT_EQ(fleetLeaderId(), FLEET_ID_UNASSIGNED);
}
