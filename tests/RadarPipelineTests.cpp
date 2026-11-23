#include <gtest/gtest.h>

#include <random>

#include "core/MapProjection.h"
#include "ecs/Systems.h"
#include "ecs/components.h"

class RadarPipelineTests : public ::testing::Test
{
protected:
    entt::registry registry;
    entt::entity target = entt::null;
    entt::entity observer1 = entt::null;
    entt::entity observer2 = entt::null;

    static constexpr float kAltitudeMeters = 10000.0f;
    static constexpr float kScanRateSec = 0.5f;

    void SetUp() override
    {
        registry.ctx().emplace<MapProjection>();
        registry.ctx().emplace<RadarDetectionData>();
        registry.ctx().emplace<MasterTrackTable>();
        registry.ctx().emplace<std::mt19937>(12345);
        registry.ctx().emplace<float>(0.0f);

        observer1 = CreateObserver({0.0f, 0.0f});
        observer2 = CreateObserver({0.0f, 1.0f});
        target = CreateTarget({10.0f, 0.0f});
    }

    entt::entity CreateObserver(MathUtils::Vec2 position)
    {
        const entt::entity entity = registry.create();
        registry.emplace<Transform2D>(
            entity, Transform2D{position, kAltitudeMeters, 0.0f});
        registry.emplace<Kinematics>(entity);

        auto& radar = registry.emplace<RadarEmitter>(entity);
        radar.maxRangeNM = 100.0f;
        radar.maxRangeNmSq = 10000.0f;
        radar.scanRateSec = kScanRateSec;
        radar.timeSinceLastScan = kScanRateSec;
        radar.rangeErrorNM = 0.01f;
        radar.angleErrorRadians = 0.001f;
        radar.velocityErrorKnots = 5.0f;

        registry.emplace<RadarSignature>(entity, RadarSignature{1.0f, 1.0f});
        registry.emplace<IFF>(entity, IFF{false});
        return entity;
    }

    entt::entity CreateTarget(MathUtils::Vec2 position)
    {
        const entt::entity entity = registry.create();
        registry.emplace<Transform2D>(
            entity, Transform2D{position, kAltitudeMeters, 0.0f});
        registry.emplace<Kinematics>(entity);
        registry.emplace<RadarSignature>(entity, RadarSignature{1.0f, 1.0f});
        registry.emplace<IFF>(entity, IFF{true});
        return entity;
    }

    void RunScan(float timestampSec, float deltaTime)
    {
        registry.ctx().get<float>() = timestampSec;
        registry.get<RadarEmitter>(observer1).timeSinceLastScan = kScanRateSec;
        registry.get<RadarEmitter>(observer2).timeSinceLastScan = kScanRateSec;

        Systems::SensorSystem(registry, 0.0f);
        Systems::TrackingSystem(registry, deltaTime);
        Systems::SensorFusionSystem(registry, deltaTime);
    }
};

TEST_F(RadarPipelineTests, TwoObserversBuildOneConfirmedFusedGlobalTrack)
{
    for (int scan = 0; scan < 3; ++scan)
    {
        const float time = static_cast<float>(scan) * kScanRateSec;
        RunScan(time, scan == 0 ? 0.0f : kScanRateSec);
    }

    const auto& data = registry.ctx().get<RadarDetectionData>();
    ASSERT_TRUE(data.activeTracks.contains(observer1));
    ASSERT_TRUE(data.activeTracks.contains(observer2));
    ASSERT_EQ(data.activeTracks.at(observer1).size(), 1u);
    ASSERT_EQ(data.activeTracks.at(observer2).size(), 1u);

    const RadarTrack& track1 = data.activeTracks.at(observer1).front();
    const RadarTrack& track2 = data.activeTracks.at(observer2).front();

    EXPECT_EQ(track1.lifecycle, TrackLifecycle::CONFIRMED);
    EXPECT_EQ(track2.lifecycle, TrackLifecycle::CONFIRMED);
    EXPECT_TRUE(track1.kalman.initialized);
    EXPECT_TRUE(track2.kalman.initialized);

    EXPECT_NE(track1.globalTrackId, NO_GLOBAL_TRACK);
    EXPECT_EQ(track1.globalTrackId, track2.globalTrackId);

    const auto& blueGlobalTracks =
        registry.ctx().get<MasterTrackTable>().GetSide(false).activeGlobalTracks;

    ASSERT_EQ(blueGlobalTracks.size(), 1u);
    const RadarTrack& fused = blueGlobalTracks.begin()->second.track;

    EXPECT_EQ(fused.globalTrackId, track1.globalTrackId);
    EXPECT_NEAR(fused.pos.x, 10.0f, 0.02f);
    EXPECT_NEAR(fused.pos.y, 0.0f, 0.02f);
    EXPECT_NEAR(fused.altitude, kAltitudeMeters, 1.0f);
}

TEST_F(RadarPipelineTests, StationaryTargetDoesNotInheritClimbingObserversVerticalVelocity)
{
    // This is the regression scenario from the rendered target estimate:
    // the target is level, while the radar observer is climbing.
    registry.get<Kinematics>(observer1).verticalVelocityNMPerSec = 0.01f;

    registry.ctx().get<float>() = 0.0f;
    registry.get<RadarEmitter>(observer1).timeSinceLastScan = kScanRateSec;
    registry.get<RadarEmitter>(observer2).timeSinceLastScan = 0.0f;

    Systems::SensorSystem(registry, 0.0f);
    Systems::TrackingSystem(registry, 0.0f);

    const auto& localTracks =
        registry.ctx().get<RadarDetectionData>().activeTracks.at(observer1);

    ASSERT_EQ(localTracks.size(), 1u);

    EXPECT_NEAR(
        localTracks.front().verticalVelocityNMPerSec,
        0.0f,
        1.0e-6f
    );

    EXPECT_NEAR(
        localTracks.front().altitude,
        kAltitudeMeters,
        1.0e-3f
    );
}
