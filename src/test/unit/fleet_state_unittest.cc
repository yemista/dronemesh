#include <gtest/gtest.h>

#include "fleet/fleet_state.h"

TEST(FleetStateTest, DetectsApproachingCollision)
{
    fleetStateSample_t selfState = {
        .nodeId = 1,
        .valid = true,
        .posX = 0,
        .posY = 0,
        .posZ = 0,
        .velX = 100,
        .velY = 0,
        .velZ = 0,
        .yawDecideg = 0,
        .mode = FLEET_STATE_MODE_NORMAL,
    };

    fleetStateSample_t otherState = {
        .nodeId = 2,
        .valid = true,
        .posX = 200,
        .posY = 0,
        .posZ = 0,
        .velX = -100,
        .velY = 0,
        .velZ = 0,
        .yawDecideg = 0,
        .mode = FLEET_STATE_MODE_NORMAL,
    };

    float minDistanceCm = 0.0f;
    float timeToCollisionS = 0.0f;
    const bool collision = fleetStateEvaluateCollision(&selfState, &otherState, 200.0f, 3.0f,
                                                      &minDistanceCm, &timeToCollisionS);

    EXPECT_TRUE(collision);
    EXPECT_LT(minDistanceCm, 1.0f);
    EXPECT_GT(timeToCollisionS, 0.0f);
}

TEST(FleetStateTest, IgnoresNonIntersectingPaths)
{
    fleetStateSample_t selfState = {
        .nodeId = 1,
        .valid = true,
        .posX = 0,
        .posY = 0,
        .posZ = 0,
        .velX = 100,
        .velY = 0,
        .velZ = 0,
        .yawDecideg = 0,
        .mode = FLEET_STATE_MODE_NORMAL,
    };

    fleetStateSample_t otherState = {
        .nodeId = 2,
        .valid = true,
        .posX = 1000,
        .posY = 0,
        .posZ = 0,
        .velX = 100,
        .velY = 0,
        .velZ = 0,
        .yawDecideg = 0,
        .mode = FLEET_STATE_MODE_NORMAL,
    };

    float minDistanceCm = 0.0f;
    float timeToCollisionS = 0.0f;
    const bool collision = fleetStateEvaluateCollision(&selfState, &otherState, 200.0f, 3.0f,
                                                      &minDistanceCm, &timeToCollisionS);

    EXPECT_FALSE(collision);
    EXPECT_GT(minDistanceCm, 200.0f);
}
