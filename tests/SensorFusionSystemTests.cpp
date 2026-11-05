
#include <gtest/gtest.h>

#include "RadarTestHelpers.h"

class SensorFusionSystemTests : public ::testing::Test
{
protected:
    entt::registry registry;
    entt::entity observer1 = entt::null;
    entt::entity observer2 = entt::null;

    void SetUp() override
    {
        InitializeRadarContexts(registry);

        observer1 = registry.create();
        observer2 = registry.create();
    }

    RadarDetectionData& Data()
    {
        return registry.ctx().get<RadarDetectionData>();
    }

    MasterTrackTable& Master()
    {
        return registry.ctx().get<MasterTrackTable>();
    }

    RadarTrack MakeKnownLocalTrack(
        MathUtils::Vec2 position = {10.0f, 10.0f},
        MathUtils::Vec2 velocity = {0.2f, 0.0f})
    {
        RadarTrack track = MakeRadarTrack(
            position,
            ThreatClass::MISSILE_INBOUND,
            true,
            velocity);

        track.latestPing = position;
        track.globalTrackId = NO_GLOBAL_TRACK;

        return track;
    }

    MasterTrackTable::GlobalTrack MakeGlobalTrack(
        uint32_t id,
        MathUtils::Vec2 position = {10.0f, 10.0f},
        MathUtils::Vec2 velocity = {0.2f, 0.0f})
    {
        MasterTrackTable::GlobalTrack global{};

        global.track = MakeKnownLocalTrack(position, velocity);
        global.track.globalTrackId = id;
        global.confidence = 1.0f;

        return global;
    }
};

// ============================================================
// GLOBAL TRACK CREATION
// ============================================================

TEST_F(SensorFusionSystemTests, CreatesGlobalTrackForKnownLocalTrack)
{
    Data().activeTracks[observer1].push_back(
        MakeKnownLocalTrack());

    Systems::SensorFusionSystem(registry, 0.0f);

    ASSERT_EQ(Master().activeGlobalTracks.size(), 1);

    const uint32_t id =
        Data().activeTracks[observer1].front().globalTrackId;

    EXPECT_NE(id, NO_GLOBAL_TRACK);
    EXPECT_TRUE(Master().activeGlobalTracks.contains(id));

    EXPECT_EQ(id, 1000u);
}

TEST_F(SensorFusionSystemTests, DoesNotFuseUnknownLocalTrack)
{
    RadarTrack track = MakeRadarTrack(
        {10.0f, 10.0f},
        ThreatClass::UNKNOWN,
        true,
        VelocityFromKnots(1000.0f));

    Data().activeTracks[observer1].push_back(track);

    Systems::SensorFusionSystem(registry, 0.0f);

    EXPECT_TRUE(Master().activeGlobalTracks.empty());
    EXPECT_EQ(
        Data().activeTracks[observer1].front().globalTrackId,
        NO_GLOBAL_TRACK);
}

TEST_F(SensorFusionSystemTests, DoesNotFuseMissingLocalTrack)
{
    RadarTrack track = MakeKnownLocalTrack();
    track.isMissing = true;

    Data().activeTracks[observer1].push_back(track);

    Systems::SensorFusionSystem(registry, 0.0f);

    EXPECT_TRUE(Master().activeGlobalTracks.empty());
}

// ============================================================
// CORRELATION TO EXISTING GLOBAL TRACK
// ============================================================

TEST_F(SensorFusionSystemTests, LocalTrackCorrelatesToExistingGlobalTrack)
{
    constexpr uint32_t existingId = 1000;

    Master().activeGlobalTracks.emplace(
        existingId,
        MakeGlobalTrack(existingId));

    Master().nextGlobalTrackId = existingId + 1;

    Data().activeTracks[observer1].push_back(
        MakeKnownLocalTrack());

    Systems::SensorFusionSystem(registry, 0.0f);

    ASSERT_EQ(Master().activeGlobalTracks.size(), 1);

    EXPECT_EQ(
        Data().activeTracks[observer1].front().globalTrackId,
        existingId);

    EXPECT_TRUE(
        Master().activeGlobalTracks.contains(existingId));
}

// ============================================================
// MULTI-OBSERVER FUSION
// ============================================================

TEST_F(SensorFusionSystemTests, TwoObserversReportingSameTargetShareOneGlobalID)
{
    const MathUtils::Vec2 position{10.0f, 10.0f};
    const MathUtils::Vec2 velocity =
        VelocityFromKnots(1000.0f);

    Data().activeTracks[observer1].push_back(
        MakeKnownLocalTrack(position, velocity));

    Data().activeTracks[observer2].push_back(
        MakeKnownLocalTrack(position, velocity));

    Systems::SensorFusionSystem(registry, 0.0f);

    ASSERT_EQ(Master().activeGlobalTracks.size(), 1);

    const uint32_t id1 =
        Data().activeTracks[observer1].front().globalTrackId;

    const uint32_t id2 =
        Data().activeTracks[observer2].front().globalTrackId;

    EXPECT_NE(id1, NO_GLOBAL_TRACK);
    EXPECT_EQ(id1, id2);
}

TEST_F(SensorFusionSystemTests, TwoDifferentTargetsDoNotShareGlobalID)
{
    auto& tracks = Data().activeTracks[observer1];

    tracks.push_back(
        MakeKnownLocalTrack(
            {10.0f, 10.0f},
            VelocityFromKnots(1000.0f)));

    tracks.push_back(
        MakeKnownLocalTrack(
            {100.0f, 100.0f},
            VelocityFromKnots(1000.0f)));

    Systems::SensorFusionSystem(registry, 0.0f);

    ASSERT_EQ(Master().activeGlobalTracks.size(), 2);

    EXPECT_NE(tracks[0].globalTrackId, NO_GLOBAL_TRACK);
    EXPECT_NE(tracks[1].globalTrackId, NO_GLOBAL_TRACK);

    EXPECT_NE(
        tracks[0].globalTrackId,
        tracks[1].globalTrackId);
}

// ============================================================
// ID PERSISTENCE / CLASSIFICATION CONFLICT
// ============================================================

TEST_F(SensorFusionSystemTests, ClassificationConflictCreatesNewGlobalTrack)
{
    constexpr uint32_t oldId = 1000;

    Master().activeGlobalTracks.emplace(
        oldId,
        MakeGlobalTrack(oldId));

    Master().nextGlobalTrackId = oldId + 1;

    RadarTrack local = MakeKnownLocalTrack();

    // The local track already points to the old global ID,
    // but now its confirmed class conflicts with the global class.
    local.globalTrackId = oldId;
    local.classification = ThreatClass::SURFACE_SHIP;

    Data().activeTracks[observer1].push_back(local);

    Systems::SensorFusionSystem(registry, 0.0f);

    ASSERT_EQ(Master().activeGlobalTracks.size(), 2);

    const uint32_t newId =
        Data().activeTracks[observer1].front().globalTrackId;

    EXPECT_NE(newId, oldId);
    EXPECT_TRUE(Master().activeGlobalTracks.contains(oldId));
    EXPECT_TRUE(Master().activeGlobalTracks.contains(newId));
}

// ============================================================
// GLOBAL TRACK AGING / EXPIRATION
// ============================================================

TEST_F(SensorFusionSystemTests, GlobalTrackPositionIsPredicted)
{
    constexpr uint32_t id = 1000;

    const MathUtils::Vec2 latestPing{10.0f, 10.0f};
    const MathUtils::Vec2 velocity =
        VelocityFromKnots(1000.0f);

    Master().activeGlobalTracks.emplace(
        id,
        MakeGlobalTrack(id, latestPing, velocity));

    Systems::SensorFusionSystem(registry, 0.5f);

    const auto& global = Master().activeGlobalTracks.at(id).track;

    const MathUtils::Vec2 expected =
        MathUtils::ExtrapolatePosition(
            latestPing,
            velocity,
            0.5f);

    EXPECT_NEAR(global.pos.x, expected.x, 0.0001f);
    EXPECT_NEAR(global.pos.y, expected.y, 0.0001f);

    EXPECT_NEAR(global.ageSec, 0.5f, 0.0001f);
    EXPECT_TRUE(global.isMissing);
}

TEST_F(SensorFusionSystemTests, GlobalTrackExpiresAfterTimeout)
{
    constexpr uint32_t id = 1000;

    Master().activeGlobalTracks.emplace(
        id,
        MakeGlobalTrack(id));

    Systems::SensorFusionSystem(
        registry,
        TRACK_DELETION_TIMEOUT_SEC + 0.1f);

    EXPECT_FALSE(Master().activeGlobalTracks.contains(id));
}
