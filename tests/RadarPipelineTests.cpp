#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ecs/Systems.h"
#include "core/Constants.h"
#include "core/MapProjection.h"
#include "utils/KalmanFilter3D.h"

class RadarPipelineTests : public ::testing::Test
{
protected:
    entt::registry registry;
    entt::entity observer = entt::null;
    entt::entity target = entt::null;

    static constexpr float kStep = 0.5f;
    static constexpr float kTargetSpeedKnots = 1000.0f;

    void SetUp() override
    {
        registry.ctx().emplace<MapProjection>();
        registry.ctx().emplace<RadarDetectionData>();
        registry.ctx().emplace<MasterTrackTable>();
        registry.ctx().emplace<std::mt19937>(123456);
        registry.ctx().emplace<float>(0.0f);

        observer = registry.create();
        registry.emplace<Transform2D>(
            observer, Transform2D{{0.0f, 0.0f}, 5000.0f, 0.0f});
        registry.emplace<Kinematics>(observer);
        registry.emplace<IFF>(observer, IFF{false});

        RadarEmitter emitter{};
        emitter.maxRangeNM = 100.0f;
        emitter.maxRangeNmSq = 10000.0f;
        emitter.scanRateSec = kStep;
        emitter.timeSinceLastScan = 0.0f;
        emitter.rangeErrorNM = 0.0f;
        emitter.angleErrorRadians = 0.0f;
        emitter.velocityErrorKnots = 0.0f;
        registry.emplace<RadarEmitter>(observer, emitter);

        target = registry.create();
        registry.emplace<Transform2D>(
            target, Transform2D{{10.0f, 0.0f}, 5000.0f, 0.0f});

        Kinematics targetKinematics{};
        targetKinematics.velocity = {
            -kTargetSpeedKnots / NM_PER_SEC_TO_KNOTS, 0.0f};
        registry.emplace<Kinematics>(target, targetKinematics);
        registry.emplace<RadarSignature>(target, RadarSignature{1.0f, 1.0f});
        registry.emplace<IFF>(target, IFF{true});
    }

    RadarDetectionData& Data()
    {
        return registry.ctx().get<RadarDetectionData>();
    }

    MasterTrackTable& Master()
    {
        return registry.ctx().get<MasterTrackTable>();
    }

    void Step(float dt = kStep)
    {
        Systems::SensorSystem(registry, dt);
        Systems::TrackingSystem(registry, dt);
        Systems::SensorFusionSystem(registry, dt);

        auto& time = registry.ctx().get<float>();
        time += dt;

        auto& transform = registry.get<Transform2D>(target);
        const auto& velocity = registry.get<Kinematics>(target).velocity;
        transform.pos.x += velocity.x * dt;
        transform.pos.y += velocity.y * dt;
    }
};

TEST_F(RadarPipelineTests, SensorTrackingAndFusionProduceOneConsistentGlobalTrack)
{
    // Continue long enough for the local track to acquire velocity and pass
    // the existing classification-confirmation interval.
    for (int i = 0; i < 8; ++i)
        Step();

    ASSERT_TRUE(Data().activeTracks.contains(observer));
    ASSERT_EQ(Data().activeTracks.at(observer).size(), 1u);

    const RadarTrack& local = Data().activeTracks.at(observer).front();
    ASSERT_EQ(local.classification, ThreatClass::MISSILE_INBOUND);
    ASSERT_NE(local.globalTrackId, NO_GLOBAL_TRACK);

    ASSERT_EQ(Master().activeGlobalTracks.size(), 1u);
    ASSERT_TRUE(Master().activeGlobalTracks.contains(local.globalTrackId));

    const RadarTrack& global =
        Master().activeGlobalTracks.at(local.globalTrackId).track;

    EXPECT_TRUE(local.kalman.initialized);
    EXPECT_TRUE(global.kalman.initialized);
    EXPECT_EQ(local.kalman.measurementCount, 8u);
    EXPECT_EQ(global.kalman.measurementCount, local.kalman.measurementCount);
    EXPECT_FALSE(local.isMissing);
    EXPECT_FALSE(global.isMissing);

    EXPECT_NEAR(global.pos.x, local.pos.x, 1.0e-4f);
    EXPECT_NEAR(global.pos.y, local.pos.y, 1.0e-4f);
    EXPECT_NEAR(global.vel.x, local.vel.x, 1.0e-4f);
    EXPECT_NEAR(global.vel.y, local.vel.y, 1.0e-4f);

    // The target is moving toward the observer, so the fused estimate should
    // have a negative X velocity and a plausible speed.
    EXPECT_LT(global.vel.x, 0.0f);
    EXPECT_NEAR(global.speedKnots, kTargetSpeedKnots, 80.0f);
}

TEST_F(RadarPipelineTests, MultipleFramesDoNotCreateDuplicateLocalOrGlobalTracks)
{
    for (int i = 0; i < 20; ++i)
        Step();

    ASSERT_TRUE(Data().activeTracks.contains(observer));
    EXPECT_EQ(Data().activeTracks.at(observer).size(), 1u);
    EXPECT_EQ(Master().activeGlobalTracks.size(), 1u);
}

TEST_F(RadarPipelineTests, NoDetectionLeavesGlobalTrackPredictedAndMarkedMissing)
{
    for (int i = 0; i < 8; ++i)
        Step();

    ASSERT_EQ(Master().activeGlobalTracks.size(), 1u);
    const uint32_t id = Master().activeGlobalTracks.begin()->first;
    const float oldX = Master().activeGlobalTracks.at(id).track.pos.x;

    // Stop the observer from scanning while allowing the fusion system to age
    // and predict the existing global track.
    registry.get<RadarEmitter>(observer).timeSinceLastScan = 0.0f;
    registry.get<RadarEmitter>(observer).scanRateSec = 1000.0f;

    Systems::SensorSystem(registry, kStep);
    // Force a completed scan with no detections; SensorSystem clears the
    // previous frame's measurements when the scan interval is not reached.
    Data().frameMeasurements[observer].clear();
    Systems::TrackingSystem(registry, kStep);
    Systems::SensorFusionSystem(registry, kStep);

    const RadarTrack& global = Master().activeGlobalTracks.at(id).track;
    EXPECT_TRUE(global.isMissing);
    EXPECT_GT(global.ageSec, 0.0f);
    EXPECT_NE(global.pos.x, oldX);
}
