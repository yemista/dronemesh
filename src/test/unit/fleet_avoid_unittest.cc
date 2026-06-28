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

extern "C" {
    #include "build/debug.h"
    #include "common/axis.h"
    #include "fleet/fleet_id.h"
    #include "fleet/fleet_avoid.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    static timeUs_t testTimeUs = 0;
    timeUs_t micros(void) { return testTimeUs; }
    uint32_t millis(void) { return testTimeUs / 1000; }
}

static void driveIdResolved()
{
    for (int i = 0; i < 50 && !fleetIdIsResolved(); i++) {
        testTimeUs += 200000;
        fleetIdUpdate(testTimeUs);
    }
}

static void deliverAvoid(bool active, int16_t rollCd, int16_t pitchCd, uint8_t node = 7)
{
    fleetAvoidMessage_t m;
    memset(&m, 0, sizeof(m));
    m.type = FLEET_MSG_AVOID_COMMAND;
    m.flags = active ? FLEET_AVOID_FLAG_ACTIVE : 0;
    m.rollAngleCd = rollCd;
    m.pitchAngleCd = pitchCd;
    m.conflictNodeId = node;
    fleetAvoidReceive((const uint8_t *)&m, sizeof(m));
}

class FleetAvoidTest : public ::testing::Test {
protected:
    void SetUp() override {
        testTimeUs = 0;
        fleetIdInit();
        fleetIdSetUid(10);   // -> fleet ID 11
        driveIdResolved();   // settle the ID so fleetHasId() is true
        fleetAvoidInit();
    }
};

TEST_F(FleetAvoidTest, InactiveInitially)
{
    EXPECT_FALSE(fleetAvoidIsActive());
    EXPECT_FLOAT_EQ(fleetAvoidGetAngleCd(FD_ROLL), 0.0f);
    EXPECT_FLOAT_EQ(fleetAvoidGetAngleCd(FD_PITCH), 0.0f);
}

TEST_F(FleetAvoidTest, AppliesActiveCommand)
{
    deliverAvoid(true, 1500, -800, 7);

    EXPECT_TRUE(fleetAvoidIsActive());
    EXPECT_FLOAT_EQ(fleetAvoidGetAngleCd(FD_ROLL), 1500.0f);
    EXPECT_FLOAT_EQ(fleetAvoidGetAngleCd(FD_PITCH), -800.0f);
    EXPECT_EQ(fleetAvoidGetConflictNodeId(), 7);
}

TEST_F(FleetAvoidTest, ClearedFlagDisengages)
{
    deliverAvoid(true, 1500, 0);
    ASSERT_TRUE(fleetAvoidIsActive());

    deliverAvoid(false, 1500, 0);
    EXPECT_FALSE(fleetAvoidIsActive());
    EXPECT_FLOAT_EQ(fleetAvoidGetAngleCd(FD_ROLL), 0.0f);
}

TEST_F(FleetAvoidTest, StaleCommandTimesOut)
{
    deliverAvoid(true, 1200, 600);
    ASSERT_TRUE(fleetAvoidIsActive());

    // Within the freshness window: still active.
    testTimeUs += (FLEET_AVOID_TIMEOUT_MS - 1) * 1000;
    EXPECT_TRUE(fleetAvoidIsActive());

    // Past the deadline: avoidance fails safe back to the pilot.
    testTimeUs += 2 * 1000;
    EXPECT_FALSE(fleetAvoidIsActive());
    EXPECT_FLOAT_EQ(fleetAvoidGetAngleCd(FD_PITCH), 0.0f);
}

TEST_F(FleetAvoidTest, ClampsToMaxAngle)
{
    deliverAvoid(true, 5000, -5000);

    EXPECT_FLOAT_EQ(fleetAvoidGetAngleCd(FD_ROLL), (float)FLEET_AVOID_MAX_ANGLE_CD);
    EXPECT_FLOAT_EQ(fleetAvoidGetAngleCd(FD_PITCH), -(float)FLEET_AVOID_MAX_ANGLE_CD);
}

TEST_F(FleetAvoidTest, IgnoredWithoutFleetId)
{
    // Fresh node that has not resolved an ID yet must ignore avoid commands,
    // matching the fleet-wide "no mesh traffic until assigned an ID" rule.
    fleetIdInit();
    fleetAvoidInit();
    ASSERT_FALSE(fleetHasId());

    deliverAvoid(true, 1500, 0);
    EXPECT_FALSE(fleetAvoidIsActive());
}

TEST_F(FleetAvoidTest, RejectsShortPayload)
{
    deliverAvoid(true, 1500, 0);
    ASSERT_TRUE(fleetAvoidIsActive());

    fleetAvoidMessage_t m;
    memset(&m, 0, sizeof(m));
    m.type = FLEET_MSG_AVOID_COMMAND;
    m.flags = FLEET_AVOID_FLAG_ACTIVE;
    // Deliver a truncated payload; it must be ignored, leaving prior state intact.
    fleetAvoidReceive((const uint8_t *)&m, sizeof(m) - 1);
    EXPECT_TRUE(fleetAvoidIsActive());
}
