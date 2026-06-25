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
    #include "platform.h"
    #include "fleet/fleet_id.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    static timeUs_t testTimeUs = 0;
    timeUs_t micros(void) { return testTimeUs; }
    uint32_t millis(void) { return testTimeUs / 1000; }
}

// Records every claim the module broadcasts so tests can assert on link traffic.
static std::vector<fleetIdMessage_t> sentMsgs;

static void testSend(const uint8_t *data, uint8_t len)
{
    if (len >= sizeof(fleetIdMessage_t)) {
        fleetIdMessage_t m;
        memcpy(&m, data, sizeof(m));
        sentMsgs.push_back(m);
    }
}

// Build an inbound claim from another drone.
static fleetIdMessage_t makeClaim(uint8_t nodeId, fleetIdState_e state, uint32_t uid)
{
    fleetIdMessage_t m;
    m.type = FLEET_MSG_ID_CLAIM;
    m.nodeId = nodeId;
    m.state = (uint8_t)state;
    m.uid = uid;
    return m;
}

static void deliver(const fleetIdMessage_t &m)
{
    fleetIdReceive((const uint8_t *)&m, sizeof(m));
}

// Run the protocol forward, in probe-interval steps, until it settles or we give up.
static void driveUntilResolved(void)
{
    for (int i = 0; i < 50 && !fleetIdIsResolved(); i++) {
        testTimeUs += 200000; // FLEET_ID_PROBE_INTERVAL_US
        fleetIdUpdate(testTimeUs);
    }
}

class FleetIdTest : public ::testing::Test {
protected:
    void SetUp() override {
        testTimeUs = 0;
        sentMsgs.clear();
        fleetIdInit();
        fleetIdSetSendFn(testSend);
    }
};

// A lone drone should pick a UID-derived ID and settle on it uncontested.
TEST_F(FleetIdTest, ConvergesWhenAlone)
{
    fleetIdSetUid(5);
    EXPECT_EQ(fleetIdGet(), 6);          // (5 % 32) + 1
    EXPECT_FALSE(fleetIdIsResolved());

    driveUntilResolved();

    EXPECT_TRUE(fleetIdIsResolved());
    EXPECT_EQ(fleetIdGet(), 6);          // kept its ID, nobody contested
    EXPECT_GE(sentMsgs.size(), (size_t)3); // probed before claiming
}

// A claim for our ID from a lower-UID drone outranks us: we must yield and
// re-probe on a different, not-yet-seen ID.
TEST_F(FleetIdTest, YieldsToLowerUid)
{
    fleetIdSetUid(10);
    driveUntilResolved();
    ASSERT_TRUE(fleetIdIsResolved());
    ASSERT_EQ(fleetIdGet(), 11);         // (10 % 32) + 1

    deliver(makeClaim(11, FLEET_ID_STATE_CLAIMED, 3)); // lower UID wins

    EXPECT_FALSE(fleetIdIsResolved());
    EXPECT_EQ(fleetIdGet(), 12);         // 11 now observed taken -> next free ID
}

// A claim from a higher-UID drone does not outrank us: keep the ID and defend it.
TEST_F(FleetIdTest, DefendsAgainstHigherUid)
{
    fleetIdSetUid(10);
    driveUntilResolved();
    ASSERT_TRUE(fleetIdIsResolved());
    ASSERT_EQ(fleetIdGet(), 11);

    sentMsgs.clear();
    deliver(makeClaim(11, FLEET_ID_STATE_CLAIMED, 99)); // higher UID loses

    EXPECT_TRUE(fleetIdIsResolved());
    EXPECT_EQ(fleetIdGet(), 11);
    EXPECT_EQ(sentMsgs.size(), (size_t)1); // re-announced to defend
}

// A probing drone yields to an already-claimed one even if its UID is lower.
TEST_F(FleetIdTest, ProbingYieldsToClaimedRegardlessOfUid)
{
    fleetIdSetUid(2);
    EXPECT_EQ(fleetIdGet(), 3);          // probing, not yet resolved
    ASSERT_FALSE(fleetIdIsResolved());

    // Remote has a HIGHER uid but is already CLAIMED, so it still wins.
    deliver(makeClaim(3, FLEET_ID_STATE_CLAIMED, 500));

    EXPECT_FALSE(fleetIdIsResolved());
    EXPECT_NE(fleetIdGet(), 3);          // yielded to the settled holder
}

// The link echoing our own broadcast back must never be treated as a conflict.
TEST_F(FleetIdTest, IgnoresOwnEcho)
{
    fleetIdSetUid(10);
    driveUntilResolved();
    ASSERT_TRUE(fleetIdIsResolved());
    ASSERT_EQ(fleetIdGet(), 11);

    deliver(makeClaim(11, FLEET_ID_STATE_CLAIMED, 10)); // same UID == us

    EXPECT_TRUE(fleetIdIsResolved());
    EXPECT_EQ(fleetIdGet(), 11);
}

TEST_F(FleetIdTest, MemberCountTracksLiveClaims)
{
    fleetIdSetUid(10);
    driveUntilResolved();
    ASSERT_TRUE(fleetIdIsResolved());

    // Three other drones announce distinct IDs.
    deliver(makeClaim(20, FLEET_ID_STATE_CLAIMED, 101));
    deliver(makeClaim(21, FLEET_ID_STATE_CLAIMED, 102));
    deliver(makeClaim(22, FLEET_ID_STATE_CLAIMED, 103));

    EXPECT_EQ(fleetMemberCount(), 4); // three others + ourselves

    const uint32_t mask = fleetMemberMask();
    EXPECT_TRUE(mask & (1u << (11 - 1)));  // ourselves (id 11)
    EXPECT_TRUE(mask & (1u << (20 - 1)));
    EXPECT_TRUE(mask & (1u << (22 - 1)));
}

TEST_F(FleetIdTest, SilentMemberAgesOut)
{
    fleetIdSetUid(10);
    driveUntilResolved();
    ASSERT_TRUE(fleetIdIsResolved());

    deliver(makeClaim(20, FLEET_ID_STATE_CLAIMED, 101));
    EXPECT_EQ(fleetMemberCount(), 2);                   // the other drone + ourselves
    EXPECT_TRUE(fleetMemberMask() & (1u << (20 - 1)));

    // Hear nothing more from id 20; advance past the member timeout and tick.
    testTimeUs += 4000 * 1000; // 4s > FLEET_MEMBER_TIMEOUT_MS (3s)
    fleetIdUpdate(testTimeUs);

    EXPECT_FALSE(fleetMemberMask() & (1u << (20 - 1))); // aged out
    EXPECT_EQ(fleetMemberCount(), 1);                   // just ourselves now
}
