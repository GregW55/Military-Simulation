
#include <gtest/gtest.h>

#include "RadarTestHelpers.h"
#include "ecs/Systems.h"

class SensorFusionSystemTests : public ::testing::Test
{
protected:
    entt::registry registry;
    entt::entity observer1 = entt::null;
    entt::entity observer2 = entt::null;
    static constexpr MathUtils::Vec2 DefaultPos = {10.0f, 10.0f};


    void SetUp() override
    {
        registry.ctx().emplace<RadarDetectionData>();
        registry.ctx().emplace<MasterTrackTable>();

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

    MasterTrackTable::GlobalTrack MakeGlobalTrack(
        uint32_t id,
        ThreatClass classification = ThreatClass::UNKNOWN,
        MathUtils::Vec2 position = DefaultPos,
        MathUtils::Vec2 velocity = {0.2f, 0.0f})
    {
        MasterTrackTable::GlobalTrack global{};

        global.track = radar_test::MakeLocalTrack(position, classification, velocity);
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
        radar_test::MakeLocalTrack(DefaultPos,
            ThreatClass::MISSILE_INBOUND, {0.2f, 0.0f}));

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
    RadarTrack track = radar_test::MakeLocalTrack(
        DefaultPos,
        ThreatClass::UNKNOWN,
        radar_test::VelocityFromKnots(1000.0f),
        true);

    Data().activeTracks[observer1].push_back(track);

    Systems::SensorFusionSystem(registry, 0.0f);

    EXPECT_TRUE(Master().activeGlobalTracks.empty());
    EXPECT_EQ(
        Data().activeTracks[observer1].front().globalTrackId,
        NO_GLOBAL_TRACK);
}

TEST_F(SensorFusionSystemTests, DoesNotFuseMissingLocalTrack)
{
    RadarTrack track = radar_test::MakeLocalTrack(DefaultPos,
        ThreatClass::UNKNOWN, {0.2f, 0.0f});
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
    const ThreatClass classification = ThreatClass::MISSILE_INBOUND;

    Master().activeGlobalTracks.emplace(
        existingId,
        MakeGlobalTrack(existingId));

    Master().nextGlobalTrackId = existingId + 1;

    Data().activeTracks[observer1].push_back(
        radar_test::MakeLocalTrack(DefaultPos, classification, {0.2f, 0.0f}));

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
    const MathUtils::Vec2 velocity =
            radar_test::VelocityFromKnots(1000.0f);
    const ThreatClass classification = ThreatClass::MISSILE_INBOUND;

    Data().activeTracks[observer1].push_back(
        radar_test::MakeLocalTrack(DefaultPos, classification, velocity));

    Data().activeTracks[observer2].push_back(
        radar_test::MakeLocalTrack(DefaultPos, classification, velocity));

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
    const ThreatClass classification = ThreatClass::MISSILE_INBOUND;

    tracks.push_back(
        radar_test::MakeLocalTrack(
            DefaultPos,
            classification,
            radar_test::VelocityFromKnots(1000.0f)));

    tracks.push_back(
        radar_test::MakeLocalTrack(
            {100.0f, 100.0f},
            classification,
            radar_test::VelocityFromKnots(1000.0f)));

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

TEST_F(SensorFusionSystemTests, ClassificationConflictCreatesNewGlobalTrack) {
    constexpr uint32_t oldId = 1000;
    const ThreatClass classification = ThreatClass::MISSILE_INBOUND;

    Master().activeGlobalTracks.emplace(
        oldId,
        MakeGlobalTrack(oldId, classification));

    Master().nextGlobalTrackId = oldId + 1;

    RadarTrack local = radar_test::MakeLocalTrack(DefaultPos, classification,{0.2f, 0.0f});

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

    const MathUtils::Vec2 velocity =
            radar_test::VelocityFromKnots(1000.0f);

    auto globalTrack = MakeGlobalTrack(
        id, ThreatClass::UNKNOWN, DefaultPos, velocity);

    // A live global track owns a Kalman state. Give this fixture the same
    // constant-velocity state so the test exercises the EKF prediction path.
    globalTrack.track.kalman.initialized = true;
    globalTrack.track.kalman.x = {
        DefaultPos.x, DefaultPos.y, globalTrack.track.altitude / METERS_PER_NM,
        velocity.x, velocity.y, 0.0f
    };
    for (int axis = 0; axis < 6; ++axis)
        globalTrack.track.kalman.P[axis * 6 + axis] = 0.01f;

    Master().activeGlobalTracks.emplace(id, globalTrack);

    Systems::SensorFusionSystem(registry, 0.5f);

    const auto& global = Master().activeGlobalTracks.at(id).track;

    const MathUtils::Vec2 expected =
        MathUtils::ExtrapolatePosition(
            DefaultPos,
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
