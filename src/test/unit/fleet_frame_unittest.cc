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
    #include "fleet/fleet_frame.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Feed bytes through the parser; report whether a complete frame appeared and
// capture the last recovered payload.
static bool feedAll(fleetFrameParser_t *p, const uint8_t *bytes, int n,
                    uint8_t *outPayload, uint8_t *outLen)
{
    bool got = false;
    for (int i = 0; i < n; i++) {
        if (fleetFrameParseByte(p, bytes[i])) {
            got = true;
            *outLen = p->len;
            memcpy(outPayload, p->payload, p->len);
        }
    }
    return got;
}

TEST(FleetFrameTest, RoundTrip)
{
    const uint8_t payload[] = { 0x01, 0x0B, 0x01, 0xDE, 0xAD, 0xBE, 0xEF };
    uint8_t frame[FLEET_FRAME_MAX_SIZE];

    const uint8_t frameLen = fleetFrameEncode(payload, sizeof(payload), frame);
    EXPECT_EQ(frameLen, sizeof(payload) + FLEET_FRAME_OVERHEAD);

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);
    uint8_t out[FLEET_FRAME_MAX_PAYLOAD];
    uint8_t outLen = 0;

    EXPECT_TRUE(feedAll(&p, frame, frameLen, out, &outLen));
    EXPECT_EQ(outLen, sizeof(payload));
    EXPECT_EQ(0, memcmp(out, payload, sizeof(payload)));
}

TEST(FleetFrameTest, EncoderHeaderLayout)
{
    const uint8_t payload[] = { 0x10, 0x20, 0x30 };
    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(payload, sizeof(payload), frame);

    ASSERT_EQ(frameLen, sizeof(payload) + FLEET_FRAME_OVERHEAD);
    EXPECT_EQ(frame[0], FLEET_FRAME_SYNC);          // sync first
    EXPECT_EQ(frame[1], sizeof(payload));           // then length
    EXPECT_EQ(0, memcmp(&frame[2], payload, sizeof(payload))); // then payload
}

TEST(FleetFrameTest, BadCrcByteRejected)
{
    const uint8_t payload[] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(payload, sizeof(payload), frame);

    frame[frameLen - 1] ^= 0xFF; // corrupt only the trailing CRC byte

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);
    uint8_t out[FLEET_FRAME_MAX_PAYLOAD];
    uint8_t outLen = 0;

    EXPECT_FALSE(feedAll(&p, frame, frameLen, out, &outLen));
}

TEST(FleetFrameTest, BackToBackFramesBothParse)
{
    const uint8_t a[] = { 0xA1, 0xA2 };
    const uint8_t b[] = { 0xB1, 0xB2, 0xB3 };
    uint8_t stream[2 * FLEET_FRAME_MAX_SIZE];

    uint8_t n = fleetFrameEncode(a, sizeof(a), stream);
    n += fleetFrameEncode(b, sizeof(b), stream + n);

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);

    int completed = 0;
    uint8_t lastLen = 0;
    uint8_t lastPayload[FLEET_FRAME_MAX_PAYLOAD];
    for (int i = 0; i < n; i++) {
        if (fleetFrameParseByte(&p, stream[i])) {
            completed++;
            lastLen = p.len;
            memcpy(lastPayload, p.payload, p.len);
        }
    }

    EXPECT_EQ(completed, 2);                          // both frames recovered
    EXPECT_EQ(lastLen, sizeof(b));                    // last one is frame b
    EXPECT_EQ(0, memcmp(lastPayload, b, sizeof(b)));
}

TEST(FleetFrameTest, TruncatedFrameThenValidRecovers)
{
    const uint8_t payload[] = { 0x77, 0x88, 0x99 };
    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(payload, sizeof(payload), frame);

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);
    uint8_t out[FLEET_FRAME_MAX_PAYLOAD];
    uint8_t outLen = 0;

    // Feed all but the final CRC byte -> incomplete, no frame yet.
    EXPECT_FALSE(feedAll(&p, frame, frameLen - 1, out, &outLen));
    // A fresh, complete frame still parses (the parser is not wedged).
    EXPECT_TRUE(feedAll(&p, frame, frameLen, out, &outLen));
    EXPECT_EQ(outLen, sizeof(payload));
    EXPECT_EQ(0, memcmp(out, payload, sizeof(payload)));
}

TEST(FleetFrameTest, ResetClearsPartialFrame)
{
    const uint8_t payload[] = { 0x5A, 0x5B };
    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(payload, sizeof(payload), frame);

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);

    // Feed the first two bytes (sync + len), then reset mid-frame.
    fleetFrameParseByte(&p, frame[0]);
    fleetFrameParseByte(&p, frame[1]);
    fleetFrameParserReset(&p);

    // Remaining bytes of the old frame must not be mistaken for a frame...
    uint8_t out[FLEET_FRAME_MAX_PAYLOAD];
    uint8_t outLen = 0;
    EXPECT_FALSE(feedAll(&p, frame + 2, frameLen - 2, out, &outLen));
    // ...but a full fresh frame parses fine.
    EXPECT_TRUE(feedAll(&p, frame, frameLen, out, &outLen));
    EXPECT_EQ(outLen, sizeof(payload));
}

TEST(FleetFrameTest, EmptyPayloadRoundTrips)
{
    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(NULL, 0, frame);
    EXPECT_EQ(frameLen, FLEET_FRAME_OVERHEAD);

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);
    uint8_t out[FLEET_FRAME_MAX_PAYLOAD];
    uint8_t outLen = 99;

    EXPECT_TRUE(feedAll(&p, frame, frameLen, out, &outLen));
    EXPECT_EQ(outLen, 0);
}

TEST(FleetFrameTest, MaxPayloadRoundTrips)
{
    uint8_t payload[FLEET_FRAME_MAX_PAYLOAD];
    for (int i = 0; i < FLEET_FRAME_MAX_PAYLOAD; i++) {
        payload[i] = (uint8_t)(i * 7 + 1);
    }
    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(payload, FLEET_FRAME_MAX_PAYLOAD, frame);
    EXPECT_EQ(frameLen, FLEET_FRAME_MAX_SIZE);

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);
    uint8_t out[FLEET_FRAME_MAX_PAYLOAD];
    uint8_t outLen = 0;

    EXPECT_TRUE(feedAll(&p, frame, frameLen, out, &outLen));
    EXPECT_EQ(outLen, FLEET_FRAME_MAX_PAYLOAD);
    EXPECT_EQ(0, memcmp(out, payload, FLEET_FRAME_MAX_PAYLOAD));
}

TEST(FleetFrameTest, OversizedPayloadRejectedByEncoder)
{
    uint8_t frame[FLEET_FRAME_MAX_SIZE + 4];
    EXPECT_EQ(fleetFrameEncode(frame, FLEET_FRAME_MAX_PAYLOAD + 1, frame), 0);
}

TEST(FleetFrameTest, CorruptPayloadFailsCrc)
{
    const uint8_t payload[] = { 0xAA, 0xBB, 0xCC };
    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(payload, sizeof(payload), frame);

    frame[3] ^= 0xFF; // flip a payload byte

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);
    uint8_t out[FLEET_FRAME_MAX_PAYLOAD];
    uint8_t outLen = 0;

    EXPECT_FALSE(feedAll(&p, frame, frameLen, out, &outLen));
}

TEST(FleetFrameTest, ResyncsAfterGarbageThenParsesNextFrame)
{
    const uint8_t payload[] = { 0x42, 0x43 };
    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(payload, sizeof(payload), frame);

    // Junk (including a stray sync and an over-long length) before a good frame.
    const uint8_t garbage[] = { 0x00, 0x11, FLEET_FRAME_SYNC, 200, 0x55, 0x66 };

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);
    uint8_t out[FLEET_FRAME_MAX_PAYLOAD];
    uint8_t outLen = 0;

    EXPECT_FALSE(feedAll(&p, garbage, sizeof(garbage), out, &outLen));
    EXPECT_TRUE(feedAll(&p, frame, frameLen, out, &outLen));
    EXPECT_EQ(outLen, sizeof(payload));
    EXPECT_EQ(0, memcmp(out, payload, sizeof(payload)));
}

TEST(FleetFrameTest, SyncByteInPayloadIsData)
{
    // Payload deliberately contains the sync byte; the length field means it is
    // consumed as data, not treated as a new frame start.
    const uint8_t payload[] = { FLEET_FRAME_SYNC, 0x01, FLEET_FRAME_SYNC };
    uint8_t frame[FLEET_FRAME_MAX_SIZE];
    const uint8_t frameLen = fleetFrameEncode(payload, sizeof(payload), frame);

    fleetFrameParser_t p;
    fleetFrameParserReset(&p);
    uint8_t out[FLEET_FRAME_MAX_PAYLOAD];
    uint8_t outLen = 0;

    EXPECT_TRUE(feedAll(&p, frame, frameLen, out, &outLen));
    EXPECT_EQ(outLen, sizeof(payload));
    EXPECT_EQ(0, memcmp(out, payload, sizeof(payload)));
}
