#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "RadarTestHelpers.h"
#include "ecs/Systems.h"

class SensorFusionSystemTests : public ::testing::Test
{
protected:
    entt::registry registry;

    void SetUp() override
    {
        registry.ctx().emplace<RadarDetectionData>();
        registry.ctx().emplace<MasterTrackTable>();
    }

    entt::entity CreateObserver(bool hostile = false)
    {
        const entt::entity observer = registry.create();
        registry.emplace<IFF>(observer, IFF{hostile});
        return observer;
    }

    RadarDetectionData& Data()
    {
        return registry.ctx().get<RadarDetectionData>();
    }

    MasterTrackTable& Master()
    {
        return registry.ctx().get<MasterTrackTable>();
    }

    RadarTrack MakeTrack(
        MathUtils::Vec2 position = {10.0f, 10.0f},
        float altitudeMeters = 1000.0f,
        float timestampSec = 1.0f,
        ThreatClass classification = ThreatClass::UNKNOWN)
    {
        return radar_test::MakeInitializedTrack(
            position,
            altitudeMeters,
            {0.1f, 0.0f},
            0.0f,
            timestampSec,
            classification,
            true
        );
    }

    void Submit(entt::entity observer, std::vector<RadarTrack> tracks)
    {
        radar_test::SubmitTracks(Data(), observer, std::move(tracks));
    }
};

TEST_F(SensorFusionSystemTests, CreatesGlobalTrackForConfirmedLocalTrack)
{
    const entt::entity observer = CreateObserver(false);
    Submit(observer, {MakeTrack()});

    Systems::SensorFusionSystem(registry, 0.0f);

    auto& blueSide = Master().GetSide(false);
    ASSERT_EQ(blueSide.activeGlobalTracks.size(), 1u);

    const auto& localTrack = Data().activeTracks.at(observer).front();
    EXPECT_EQ(localTrack.globalTrackId, 1001u);

    const auto& globalTrack = blueSide.activeGlobalTracks.at(1001u).track;
    EXPECT_EQ(globalTrack.globalTrackId, 1001u);
    EXPECT_EQ(globalTrack.lifecycle, TrackLifecycle::CONFIRMED);
    EXPECT_NEAR(globalTrack.pos.x, 10.0f, 1.0e-5f);
    EXPECT_NEAR(globalTrack.altitude, 1000.0f, 1.0e-3f);
}

TEST_F(SensorFusionSystemTests, DoesNotPublishTentativeOrMissingLocalTracks)
{
    const entt::entity observer = CreateObserver(false);

    RadarTrack tentative = MakeTrack({10.0f, 10.0f});
    tentative.lifecycle = TrackLifecycle::TENTATIVE;

    RadarTrack missing = MakeTrack({20.0f, 20.0f});
    missing.isMissing = true;

    Submit(observer, {tentative, missing});
    Systems::SensorFusionSystem(registry, 0.0f);

    EXPECT_TRUE(Master().GetSide(false).activeGlobalTracks.empty());
}

TEST_F(SensorFusionSystemTests, TwoObserversCorrelatingSameTargetShareOneGlobalId)
{
    const entt::entity observer1 = CreateObserver(false);
    const entt::entity observer2 = CreateObserver(false);

    RadarTrack track1 = MakeTrack({10.0f, 10.0f});
    RadarTrack track2 = MakeTrack({10.01f, 10.0f});

    Submit(observer1, {track1});
    Submit(observer2, {track2});

    Systems::SensorFusionSystem(registry, 0.0f);

    auto& blueSide = Master().GetSide(false);
    ASSERT_EQ(blueSide.activeGlobalTracks.size(), 1u);

    const uint32_t id1 = Data().activeTracks.at(observer1).front().globalTrackId;
    const uint32_t id2 = Data().activeTracks.at(observer2).front().globalTrackId;

    EXPECT_NE(id1, NO_GLOBAL_TRACK);
    EXPECT_EQ(id1, id2);
    EXPECT_EQ(id1, 1001u);

    const auto& fused = blueSide.activeGlobalTracks.at(id1).track;
    EXPECT_TRUE(std::isfinite(fused.pos.x));
    EXPECT_TRUE(std::isfinite(fused.altitude));
}

TEST_F(SensorFusionSystemTests, BlueAndRedSidesMaintainIndependentGlobalIdSpaces)
{
    const entt::entity blueObserver = CreateObserver(false);
    const entt::entity redObserver = CreateObserver(true);

    Submit(blueObserver, {MakeTrack({10.0f, 10.0f})});
    Submit(redObserver, {MakeTrack({10.0f, 10.0f})});

    Systems::SensorFusionSystem(registry, 0.0f);

    ASSERT_EQ(Master().GetSide(false).activeGlobalTracks.size(), 1u);
    ASSERT_EQ(Master().GetSide(true).activeGlobalTracks.size(), 1u);

    EXPECT_EQ(Data().activeTracks.at(blueObserver).front().globalTrackId, 1001u);
    EXPECT_EQ(Data().activeTracks.at(redObserver).front().globalTrackId, 1001u);
}

TEST_F(SensorFusionSystemTests, OneObserverCannotAssignTwoLocalTracksToSameGlobalTrack)
{
    const entt::entity observer = CreateObserver(false);

    // Both local contacts are close enough to correlate with one another,
    // but one observer must not claim a single global ID twice.
    Submit(observer, {
        MakeTrack({10.0f, 10.0f}),
        MakeTrack({10.001f, 10.0f})
    });

    Systems::SensorFusionSystem(registry, 0.0f);

    ASSERT_EQ(Master().GetSide(false).activeGlobalTracks.size(), 2u);

    const auto& tracks = Data().activeTracks.at(observer);
    ASSERT_EQ(tracks.size(), 2u);
    EXPECT_NE(tracks[0].globalTrackId, NO_GLOBAL_TRACK);
    EXPECT_NE(tracks[1].globalTrackId, NO_GLOBAL_TRACK);
    EXPECT_NE(tracks[0].globalTrackId, tracks[1].globalTrackId);
}

TEST_F(SensorFusionSystemTests, DoesNotFuseTheSameLocalMeasurementMoreThanOnce)
{
    const entt::entity observer = CreateObserver(false);
    Submit(observer, {MakeTrack()});

    Systems::SensorFusionSystem(registry, 0.0f);

    auto& side = Master().GetSide(false);
    ASSERT_EQ(side.activeGlobalTracks.size(), 1u);

    const auto before = side.activeGlobalTracks.at(1001u).track.kalman.x;

    // No new local measurement timestamp. Fusion should only coast the
    // global track rather than process the same local measurement again.
    Systems::SensorFusionSystem(registry, 0.0f);

    const auto& global = side.activeGlobalTracks.at(1001u).track;
    EXPECT_TRUE(global.isMissing);

    for (size_t i = 0; i < before.size(); ++i)
        EXPECT_NEAR(global.kalman.x[i], before[i], 1.0e-6f);
}

TEST_F(SensorFusionSystemTests, FusesANewerMeasurementFromAnAlreadyAssociatedObserver)
{
    const entt::entity observer = CreateObserver(false);
    RadarTrack initial = MakeTrack({10.0f, 10.0f}, 1000.0f, 1.0f);
    Submit(observer, {initial});

    Systems::SensorFusionSystem(registry, 0.0f);

    auto& tracks = Data().activeTracks.at(observer);
    ASSERT_EQ(tracks.front().globalTrackId, 1001u);

    // New local estimate at t=2. The global filter is predicted by 1 sec
    // in SensorFusionSystem so both estimates have matching timestamps.
    RadarTrack updated = MakeTrack({10.1f, 10.0f}, 1200.0f, 2.0f);
    updated.globalTrackId = 1001u;
    tracks.front() = updated;

    Systems::SensorFusionSystem(registry, 1.0f);

    const auto& global = Master().GetSide(false).activeGlobalTracks.at(1001u).track;
    EXPECT_FALSE(global.isMissing);
    EXPECT_NEAR(global.ageSec, 0.0f, 1.0e-6f);
    EXPECT_FLOAT_EQ(global.lastMeasurementTimeSec, 2.0f);
    EXPECT_GT(global.pos.x, 10.0f);
}


TEST_F(SensorFusionSystemTests, AdoptsKnownLocalClassificationIntoUnknownGlobalTrack)
{
    const entt::entity observer = CreateObserver(false);
    RadarTrack initial = MakeTrack({10.0f, 10.0f}, 1000.0f, 1.0f,
                                   ThreatClass::UNKNOWN);
    Submit(observer, {initial});

    Systems::SensorFusionSystem(registry, 0.0f);

    auto& local = Data().activeTracks.at(observer).front();
    ASSERT_EQ(local.globalTrackId, 1001u);

    RadarTrack updated = MakeTrack({10.1f, 10.0f}, 1000.0f, 2.0f,
                                   ThreatClass::MISSILE_INBOUND);
    updated.globalTrackId = 1001u;
    local = updated;

    Systems::SensorFusionSystem(registry, 1.0f);

    EXPECT_EQ(
        Master().GetSide(false).activeGlobalTracks.at(1001u).track.classification,
        ThreatClass::MISSILE_INBOUND
    );
}

TEST_F(SensorFusionSystemTests, DoesNotMergeConfirmedConflictingClassifications)
{
    const entt::entity observer1 = CreateObserver(false);
    const entt::entity observer2 = CreateObserver(false);

    Submit(observer1, {
        MakeTrack({10.0f, 10.0f}, 1000.0f, 1.0f,
                  ThreatClass::SURFACE_SHIP)
    });
    Submit(observer2, {
        MakeTrack({10.0f, 10.0f}, 1000.0f, 1.0f,
                  ThreatClass::MISSILE_INBOUND)
    });

    Systems::SensorFusionSystem(registry, 0.0f);

    ASSERT_EQ(Master().GetSide(false).activeGlobalTracks.size(), 2u);

    const uint32_t id1 = Data().activeTracks.at(observer1).front().globalTrackId;
    const uint32_t id2 = Data().activeTracks.at(observer2).front().globalTrackId;
    EXPECT_NE(id1, NO_GLOBAL_TRACK);
    EXPECT_NE(id2, NO_GLOBAL_TRACK);
    EXPECT_NE(id1, id2);
}

TEST_F(SensorFusionSystemTests, RemovesGlobalTrackAfterDeletionTimeout)
{
    const entt::entity observer = CreateObserver(false);
    Submit(observer, {MakeTrack()});

    Systems::SensorFusionSystem(registry, 0.0f);
    ASSERT_EQ(Master().GetSide(false).activeGlobalTracks.size(), 1u);

    Systems::SensorFusionSystem(registry, TRACK_DELETION_TIMEOUT_SEC + 0.1f);

    EXPECT_TRUE(Master().GetSide(false).activeGlobalTracks.empty());
}
