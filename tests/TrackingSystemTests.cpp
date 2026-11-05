
#include <gtest/gtest.h>
#include <vector>

#include "RadarTestHelpers.h"

class TrackingSystemTests : public ::testing::Test
{
protected:
    entt::registry registry;
    entt::entity observer = entt::null;

    void SetUp() override
    {
        registry.ctx().emplace<RadarDetectionData>();

        observer = MakeObserver(registry);
    }

    RadarDetectionData& Data()
    {
        return registry.ctx().get<RadarDetectionData>();
    }

    std::vector<RadarTrack>& Tracks()
    {
        return Data().activeTracks[observer];
    }

    void SetPings(std::vector<RadarTrack> pings)
    {
        Data().framePings[observer] = std::move(pings);
    }
};

// ============================================================
// TRACK CREATION
// ============================================================

TEST_F(TrackingSystemTests, FirstPingCreatesTrackWithoutVelocity)
{
    const MathUtils::Vec2 position{10.0f, 20.0f};

    SetPings({MakePing(position, 1000.0f)});

    Systems::TrackingSystem(registry, 0.5f);

    ASSERT_EQ(Tracks().size(), 1);

    const RadarTrack& track = Tracks().front();

    EXPECT_FLOAT_EQ(track.pos.x, position.x);
    EXPECT_FLOAT_EQ(track.pos.y, position.y);

    EXPECT_FLOAT_EQ(track.latestPing.x, position.x);
    EXPECT_FLOAT_EQ(track.latestPing.y, position.y);

    EXPECT_FALSE(track.hasVelocity);
    EXPECT_FALSE(track.isMissing);

    EXPECT_FLOAT_EQ(track.ageSec, 0.0f);
    EXPECT_FLOAT_EQ(track.speedKnots, 0.0f);

    EXPECT_EQ(track.globalTrackId, NO_GLOBAL_TRACK);
    EXPECT_EQ(track.classification, ThreatClass::UNKNOWN);
}

// ============================================================
// HIGH-SPEED CORRELATION / THE MAIN BUG TEST
// ============================================================

TEST_F(TrackingSystemTests, FastMovingMissileKeepsSameLocalTrack)
{
    constexpr float deltaTime = 0.5f;
    constexpr float missileSpeedKnots = 2000.0f;

    const MathUtils::Vec2 velocity =
        VelocityFromKnots(missileSpeedKnots);

    const MathUtils::Vec2 startingPosition{20.0f, 20.0f};

    const MathUtils::Vec2 step =
        MathUtils::Scale(velocity, deltaTime);

    // First radar detection: creates the track.
    SetPings({MakePing(startingPosition, 1000.0f)});

    Systems::TrackingSystem(registry, deltaTime);

    ASSERT_EQ(Tracks().size(), 1);
    EXPECT_FALSE(Tracks()[0].hasVelocity);

    // Continue detecting the same missile at constant velocity.
    for (int scan = 1; scan <= 10; ++scan)
    {
        const MathUtils::Vec2 position{
            startingPosition.x + step.x * scan,
            startingPosition.y + step.y * scan
        };

        SetPings({MakePing(position, 1000.0f)});

        Systems::TrackingSystem(registry, deltaTime);

        // This is the most important assertion in this test.
        // It must never create another local track for this target.
        ASSERT_EQ(
            Tracks().size(),
            1
        ) << "A duplicate local track was created on scan " << scan;

        const RadarTrack& track = Tracks().front();

        EXPECT_TRUE(track.hasVelocity);
        EXPECT_FALSE(track.isMissing);

        EXPECT_NEAR(track.latestPing.x, position.x, 0.0001f);
        EXPECT_NEAR(track.latestPing.y, position.y, 0.0001f);

        EXPECT_NEAR(track.pos.x, position.x, 0.0001f);
        EXPECT_NEAR(track.pos.y, position.y, 0.0001f);

        EXPECT_NEAR(
            track.speedKnots,
            missileSpeedKnots,
            0.1f
        );

        EXPECT_NEAR(track.ageSec, 0.0f, 0.0001f);
    }
}

// ============================================================
// VELOCITY AND POSITION UPDATES
// ============================================================

TEST_F(TrackingSystemTests, MatchedPingUpdatesEveryImportantTrackField)
{
    constexpr float deltaTime = 0.5f;
    constexpr float speedKnots = 1000.0f;

    const MathUtils::Vec2 velocity =
        VelocityFromKnots(speedKnots);

    const MathUtils::Vec2 firstPosition{10.0f, 10.0f};

    SetPings({MakePing(firstPosition, 500.0f)});

    Systems::TrackingSystem(registry, deltaTime);

    ASSERT_EQ(Tracks().size(), 1);

    const MathUtils::Vec2 secondPosition =
        MathUtils::Add(
            firstPosition,
            MathUtils::Scale(velocity, deltaTime));

    SetPings({MakePing(secondPosition, 1200.0f)});

    Systems::TrackingSystem(registry, deltaTime);

    ASSERT_EQ(Tracks().size(), 1);

    const RadarTrack& track = Tracks().front();

    EXPECT_TRUE(track.hasVelocity);
    EXPECT_FALSE(track.isMissing);

    EXPECT_NEAR(track.latestPing.x, secondPosition.x, 0.0001f);
    EXPECT_NEAR(track.latestPing.y, secondPosition.y, 0.0001f);

    EXPECT_NEAR(track.pos.x, secondPosition.x, 0.0001f);
    EXPECT_NEAR(track.pos.y, secondPosition.y, 0.0001f);

    EXPECT_NEAR(track.vel.x, velocity.x, 0.0001f);
    EXPECT_NEAR(track.vel.y, velocity.y, 0.0001f);

    EXPECT_NEAR(track.speedKnots, speedKnots, 0.1f);

    EXPECT_FLOAT_EQ(track.altitude, 1200.0f);
    EXPECT_NEAR(track.ageSec, 0.0f, 0.0001f);
}

TEST_F(TrackingSystemTests, VelocityIsCalculatedFromLastActualPing)
{
    constexpr float deltaTime = 0.5f;
    constexpr float speedKnots = 1800.0f;

    const MathUtils::Vec2 velocity =
        VelocityFromKnots(speedKnots);

    const MathUtils::Vec2 p0{10.0f, 10.0f};

    const MathUtils::Vec2 step =
        MathUtils::Scale(velocity, deltaTime);

    const MathUtils::Vec2 p1 =
        MathUtils::Add(p0, step);

    const MathUtils::Vec2 p2 =
        MathUtils::Add(p1, step);

    // First actual ping.
    SetPings({MakePing(p0)});
    Systems::TrackingSystem(registry, deltaTime);

    // Second actual ping calculates initial velocity.
    SetPings({MakePing(p1)});
    Systems::TrackingSystem(registry, deltaTime);

    ASSERT_EQ(Tracks().size(), 1);

    // Third actual ping should calculate velocity using p1,
    // which was the previous actual radar ping.
    SetPings({MakePing(p2)});
    Systems::TrackingSystem(registry, deltaTime);

    ASSERT_EQ(Tracks().size(), 1);

    const RadarTrack& track = Tracks().front();

    EXPECT_NEAR(track.latestPing.x, p2.x, 0.0001f);
    EXPECT_NEAR(track.latestPing.y, p2.y, 0.0001f);

    EXPECT_NEAR(track.vel.x, velocity.x, 0.0001f);
    EXPECT_NEAR(track.vel.y, velocity.y, 0.0001f);
}

// ============================================================
// PREDICTION
// ============================================================

TEST_F(TrackingSystemTests, TrackPositionIsExtrapolatedWhileWaitingForPing)
{
    const MathUtils::Vec2 latestPing{10.0f, 20.0f};
    const MathUtils::Vec2 velocity =
        VelocityFromKnots(1000.0f);

    RadarTrack track = MakeRadarTrack(
        latestPing,
        ThreatClass::MISSILE_INBOUND,
        true,
        velocity,
        0.25f);

    Tracks().push_back(track);

    // No radar scan for this observer this frame.
    // The track should still age and be predicted.
    Systems::TrackingSystem(registry, 0.25f);

    ASSERT_EQ(Tracks().size(), 1);

    const RadarTrack& updated = Tracks().front();

    const MathUtils::Vec2 expected =
        MathUtils::ExtrapolatePosition(
            latestPing,
            velocity,
            0.5f);

    EXPECT_NEAR(updated.ageSec, 0.5f, 0.0001f);

    EXPECT_NEAR(updated.pos.x, expected.x, 0.0001f);
    EXPECT_NEAR(updated.pos.y, expected.y, 0.0001f);

    // Prediction must not overwrite the last real radar observation.
    EXPECT_NEAR(updated.latestPing.x, latestPing.x, 0.0001f);
    EXPECT_NEAR(updated.latestPing.y, latestPing.y, 0.0001f);
}

// ============================================================
// MISSING TRACK BEHAVIOR
// ============================================================

TEST_F(TrackingSystemTests, NoRadarScanDoesNotMarkTrackMissing)
{
    RadarTrack track = MakeRadarTrack(
        {10.0f, 10.0f},
        ThreatClass::MISSILE_INBOUND,
        true,
        VelocityFromKnots(1000.0f));

    Tracks().push_back(track);

    // No entry for this observer means no scan was performed.
    Systems::TrackingSystem(registry, 0.1f);

    ASSERT_EQ(Tracks().size(), 1);
    EXPECT_FALSE(Tracks().front().isMissing);
}

TEST_F(TrackingSystemTests, EmptyRadarScanMarksTrackMissing)
{
    RadarTrack track = MakeRadarTrack(
        {10.0f, 10.0f},
        ThreatClass::MISSILE_INBOUND,
        true,
        VelocityFromKnots(1000.0f));

    Tracks().push_back(track);

    // An existing entry with zero pings means a scan happened,
    // but this observer detected nothing.
    SetPings({});

    Systems::TrackingSystem(registry, 0.1f);

    ASSERT_EQ(Tracks().size(), 1);
    EXPECT_TRUE(Tracks().front().isMissing);
}

// ============================================================
// REJECTED PING / NEW TRACK CREATION
// ============================================================

TEST_F(TrackingSystemTests, RejectedPingCreatesNewTrackAndMarksOldTrackMissing)
{
    RadarTrack oldTrack = MakeRadarTrack(
        {10.0f, 10.0f},
        ThreatClass::MISSILE_INBOUND,
        true,
        VelocityFromKnots(1000.0f));

    oldTrack.latestPing = {10.0f, 10.0f};

    Tracks().push_back(oldTrack);

    // This ping is deliberately nowhere near the predicted location.
    SetPings({MakePing({100.0f, 100.0f})});

    Systems::TrackingSystem(registry, 0.5f);

    ASSERT_EQ(Tracks().size(), 2);

    EXPECT_TRUE(Tracks()[0].isMissing);

    EXPECT_FALSE(Tracks()[1].hasVelocity);
    EXPECT_FALSE(Tracks()[1].isMissing);

    EXPECT_NEAR(Tracks()[1].pos.x, 100.0f, 0.0001f);
    EXPECT_NEAR(Tracks()[1].pos.y, 100.0f, 0.0001f);
}

// ============================================================
// TRACK EXPIRATION
// ============================================================

TEST_F(TrackingSystemTests, TrackIsDeletedAfterTimeout)
{
    RadarTrack track = MakeRadarTrack(
        {10.0f, 10.0f},
        ThreatClass::MISSILE_INBOUND,
        true,
        VelocityFromKnots(1000.0f),
        TRACK_DELETION_TIMEOUT_SEC - 0.1f);

    Tracks().push_back(track);

    Systems::TrackingSystem(registry, 0.2f);

    EXPECT_TRUE(Tracks().empty());
}
