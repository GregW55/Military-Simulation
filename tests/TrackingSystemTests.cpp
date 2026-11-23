#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "RadarTestHelpers.h"
#include "ecs/Systems.h"
#include "utils/KalmanFilter3D.h"

class TrackingSystemTests : public ::testing::Test
{
protected:
    entt::registry registry;
    entt::entity observer = entt::null;

    Transform2D observerTransform{{0.0f, 0.0f}, 5000.0f, 0.0f};
    Kinematics observerKinematics{};
    RadarEmitter radar{};

    void SetUp() override
    {
        registry.ctx().emplace<RadarDetectionData>();
        observer = registry.create();
        registry.emplace<Transform2D>(observer, observerTransform);
        registry.emplace<Kinematics>(observer, observerKinematics);
        registry.emplace<RadarEmitter>(observer, radar);
    }

    RadarDetectionData& Data()
    {
        return registry.ctx().get<RadarDetectionData>();
    }

    RadarMeasurement MeasurementAt(
        float x,
        float y = 0.0f,
        float altitudeMeters = 5000.0f,
        float radialVelocityKnots = -1000.0f,
        float timestampSec = 0.0f) const
    {
        const float dx = x - observerTransform.pos.x;
        const float dy = y - observerTransform.pos.y;
        const float dz =
            (altitudeMeters - observerTransform.altitude) / METERS_PER_NM;

        const float horizontalRange = std::sqrt(dx * dx + dy * dy);
        const float slantRange = std::sqrt(
            horizontalRange * horizontalRange + dz * dz);

        return radar_test::MakeMeasurement(
            slantRange,
            std::atan2(dy, dx),
            std::atan2(dz, horizontalRange),
            radialVelocityKnots,
            timestampSec
        );
    }

    void Submit(std::vector<RadarMeasurement> measurements)
    {
        radar_test::SubmitMeasurements(Data(), observer, std::move(measurements));
    }

    const std::vector<RadarTrack>& Tracks() const
    {
        return registry.ctx().get<RadarDetectionData>().activeTracks.at(observer);
    }

    void RunScan(std::vector<RadarMeasurement> measurements, float deltaTime)
    {
        Submit(std::move(measurements));
        Systems::TrackingSystem(registry, deltaTime);
    }
};

TEST_F(TrackingSystemTests, FirstMeasurementCreatesOneInitializedTentativeTrack)
{
    RunScan({MeasurementAt(10.0f)}, 0.0f);

    ASSERT_EQ(Tracks().size(), 1u);
    const RadarTrack& track = Tracks().front();

    EXPECT_TRUE(track.kalman.initialized);
    EXPECT_EQ(track.kalman.measurementCount, 1u);
    EXPECT_FALSE(track.hasVelocity);
    EXPECT_EQ(track.lifecycle, TrackLifecycle::TENTATIVE);
    EXPECT_NEAR(track.pos.x, 10.0f, 1.0e-4f);
    EXPECT_NEAR(track.pos.y, 0.0f, 1.0e-4f);
    EXPECT_NEAR(track.ageSec, 0.0f, 1.0e-5f);
    EXPECT_FLOAT_EQ(track.lastMeasurementTimeSec, 0.0f);
    EXPECT_FALSE(track.isMissing);
}

TEST_F(TrackingSystemTests, SecondMeasurementCorrectsExistingTrack)
{
    RunScan({MeasurementAt(10.0f, 0.0f, 5000.0f, -1000.0f, 0.0f)}, 0.0f);

    RunScan({MeasurementAt(9.86f, 0.0f, 5000.0f, -1000.0f, 0.5f)}, 0.5f);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_EQ(Tracks().front().kalman.measurementCount, 2u);
    EXPECT_TRUE(Tracks().front().hasVelocity);
    EXPECT_FALSE(Tracks().front().isMissing);
    EXPECT_NEAR(Tracks().front().ageSec, 0.0f, 1.0e-5f);
    EXPECT_FLOAT_EQ(Tracks().front().lastMeasurementTimeSec, 0.5f);
}

TEST_F(TrackingSystemTests, ConstantVelocityTargetRemainsOneTrackAcrossManyScans)
{
    constexpr float dt = 0.5f;
    constexpr float vx = -1000.0f / 3600.0f;
    float targetX = 10.0f;

    for (int frame = 0; frame < 12; ++frame)
    {
        const float timestamp = static_cast<float>(frame) * dt;
        RunScan(
            {MeasurementAt(targetX, 0.0f, 5000.0f, -1000.0f, timestamp)},
            frame == 0 ? 0.0f : dt
        );
        targetX += vx * dt;
    }

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_EQ(Tracks().front().kalman.measurementCount, 12u);
    EXPECT_TRUE(Tracks().front().hasVelocity);
    EXPECT_NEAR(Tracks().front().pos.x, targetX - vx * dt, 0.08f);
    EXPECT_NEAR(Tracks().front().vel.x, vx, 0.04f);
}

TEST_F(TrackingSystemTests, TwoSeparatedTargetsCreateTwoIndependentTracks)
{
    RunScan({
        MeasurementAt(10.0f, 0.0f, 5000.0f, -1000.0f),
        MeasurementAt(30.0f, 0.0f, 5000.0f, -1000.0f)
    }, 0.0f);

    ASSERT_EQ(Tracks().size(), 2u);
    EXPECT_NEAR(Tracks()[0].pos.x, 10.0f, 1.0e-4f);
    EXPECT_NEAR(Tracks()[1].pos.x, 30.0f, 1.0e-4f);
}

TEST_F(TrackingSystemTests, TwoTargetsCrossWithoutExchangingTheirVelocityTracks)
{
    constexpr float dt = 0.5f;
    constexpr float targetSpeedNmPerSec = 0.2f;
    constexpr float targetSpeedKnots = targetSpeedNmPerSec * 3600.0f;

    for (int frame = 0; frame <= 6; ++frame)
    {
        const float time = static_cast<float>(frame) * dt;
        const float targetA = 9.5f + targetSpeedNmPerSec * time;
        const float targetB = 10.5f - targetSpeedNmPerSec * time;

        auto a = MeasurementAt(
            targetA, 0.0f, 5000.0f, targetSpeedKnots, time);
        auto b = MeasurementAt(
            targetB, 0.0f, 5000.0f, -targetSpeedKnots, time);

        // Reverse input order on alternate scans. Track identity must not
        // depend on the order in which the sensor returns measurements.
        if (frame % 2 == 0)
            RunScan({a, b}, frame == 0 ? 0.0f : dt);
        else
            RunScan({b, a}, dt);
    }

    ASSERT_EQ(Tracks().size(), 2u);

    const auto positiveVelocityTrack = std::find_if(
        Tracks().begin(), Tracks().end(),
        [](const RadarTrack& track) { return track.vel.x > 0.0f; });

    const auto negativeVelocityTrack = std::find_if(
        Tracks().begin(), Tracks().end(),
        [](const RadarTrack& track) { return track.vel.x < 0.0f; });

    ASSERT_NE(positiveVelocityTrack, Tracks().end());
    ASSERT_NE(negativeVelocityTrack, Tracks().end());

    EXPECT_GT(positiveVelocityTrack->pos.x, 10.0f);
    EXPECT_LT(negativeVelocityTrack->pos.x, 10.0f);
    EXPECT_EQ(positiveVelocityTrack->kalman.measurementCount, 7u);
    EXPECT_EQ(negativeVelocityTrack->kalman.measurementCount, 7u);
}

TEST_F(TrackingSystemTests, UnmatchedTrackIsMarkedMissingAndPredicted)
{
    RunScan({MeasurementAt(10.0f)}, 0.0f);
    ASSERT_EQ(Tracks().size(), 1u);

    const float oldX = Tracks().front().pos.x;

    // An entry with an empty vector means a completed scan with no detections.
    RunScan({}, 0.5f);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_TRUE(Tracks().front().isMissing);
    EXPECT_EQ(Tracks().front().lifecycle, TrackLifecycle::TENTATIVE);
    EXPECT_NEAR(Tracks().front().ageSec, 0.5f, 1.0e-5f);
    EXPECT_NE(Tracks().front().pos.x, oldX);
    EXPECT_EQ(Tracks().front().kalman.measurementCount, 1u);
}

TEST_F(TrackingSystemTests, TrackConfirmsAfterThreeHitsWithinFiveScans)
{
    constexpr float dt = 0.5f;
    constexpr float vx = -1000.0f / 3600.0f;

    for (int frame = 0; frame < 3; ++frame)
    {
        const float time = static_cast<float>(frame) * dt;
        RunScan({MeasurementAt(10.0f + vx * time, 0.0f, 5000.0f,
                               -1000.0f, time)},
                frame == 0 ? 0.0f : dt);
    }

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_EQ(Tracks().front().lifecycle, TrackLifecycle::CONFIRMED);
    EXPECT_EQ(Tracks().front().scanHistoryCount, 3u);
}

TEST_F(TrackingSystemTests, ConfirmedTrackCoastsOnMissAndReturnsToConfirmedOnHit)
{
    constexpr float dt = 0.5f;
    constexpr float vx = -1000.0f / 3600.0f;

    for (int frame = 0; frame < 3; ++frame)
    {
        const float time = static_cast<float>(frame) * dt;
        RunScan({MeasurementAt(10.0f + vx * time, 0.0f, 5000.0f,
                               -1000.0f, time)},
                frame == 0 ? 0.0f : dt);
    }

    ASSERT_EQ(Tracks().front().lifecycle, TrackLifecycle::CONFIRMED);

    RunScan({}, dt);
    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_TRUE(Tracks().front().isMissing);
    EXPECT_EQ(Tracks().front().lifecycle, TrackLifecycle::COASTING);

    const float reacquireTime = 2.0f;
    const float predictedX = 10.0f + vx * reacquireTime;
    RunScan({MeasurementAt(predictedX, 0.0f, 5000.0f, -1000.0f,
                           reacquireTime)}, dt);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_FALSE(Tracks().front().isMissing);
    EXPECT_EQ(Tracks().front().lifecycle, TrackLifecycle::CONFIRMED);
}

TEST_F(TrackingSystemTests, ReturningMeasurementReacquiresTentativeTrackAtPredictedPosition)
{
    constexpr float dt = 0.5f;
    constexpr float targetVelocity = -1000.0f / 3600.0f;

    // First detection at t=0.
    RunScan({
        MeasurementAt(10.0f, 0.0f, 5000.0f, -1000.0f, 0.0f)
    }, 0.0f);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_EQ(Tracks().front().lifecycle, TrackLifecycle::TENTATIVE);

    // Target is missed at t=0.5.
    RunScan({}, dt);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_TRUE(Tracks().front().isMissing);

    // Target returns at t=1.0, at the position predicted
    // by its initial velocity estimate.
    constexpr float reacquireTime = 1.0f;

    const float predictedX =
        10.0f + targetVelocity * reacquireTime;

    RunScan({
        MeasurementAt(
            predictedX,
            0.0f,
            5000.0f,
            -1000.0f,
            reacquireTime
        )
    }, dt);

    ASSERT_EQ(Tracks().size(), 1u);

    EXPECT_FALSE(Tracks().front().isMissing);
    EXPECT_NEAR(Tracks().front().ageSec, 0.0f, 1.0e-5f);
    EXPECT_EQ(Tracks().front().kalman.measurementCount, 2u);
}

TEST_F(TrackingSystemTests, FirstMeasurementDoesNotCopyUnobservableObserverVerticalVelocity)
{
    // A horizontal line of sight has no vertical component. The radar's
    // first return cannot infer target vertical velocity from this geometry.
    registry.get<Kinematics>(observer).verticalVelocityNMPerSec = 0.01f;

    RunScan({MeasurementAt(10.0f, 0.0f, 5000.0f, 0.0f, 0.0f)}, 0.0f);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_NEAR(Tracks().front().kalman.x[5], 0.0f, 1.0e-6f);
}

TEST_F(TrackingSystemTests, FirstMeasurementCompensatesObserverVelocityAlongLineOfSight)
{
    registry.get<Kinematics>(observer).velocity = {0.1f, 0.0f};

    // A stationary target appears to move toward the observer at 0.1 NM/sec.
    RunScan({MeasurementAt(10.0f, 0.0f, 5000.0f, -360.0f, 0.0f)}, 0.0f);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_NEAR(Tracks().front().kalman.x[3], 0.0f, 1.0e-5f);
}

TEST_F(TrackingSystemTests, RepeatedElevationMeasurementsEstimateTargetVerticalVelocity)
{
    constexpr float dt = 0.5f;
    constexpr float trueVerticalVelocityNMPerSec = 0.01f;
    constexpr int scanCount = 41;

    for (int frame = 0; frame < scanCount; ++frame)
    {
        const float time = static_cast<float>(frame) * dt;
        const float verticalDisplacementNM =
            trueVerticalVelocityNMPerSec * time;
        const float altitudeMeters =
            observerTransform.altitude +
            verticalDisplacementNM * METERS_PER_NM;

        const float slantRangeNM = std::sqrt(
            10.0f * 10.0f +
            verticalDisplacementNM * verticalDisplacementNM);
        const float elevationRadians =
            std::atan2(verticalDisplacementNM, 10.0f);
        const float radialVelocityKnots =
            trueVerticalVelocityNMPerSec *
            std::sin(elevationRadians) *
            NM_PER_SEC_TO_KNOTS;

        const RadarMeasurement measurement = radar_test::MakeMeasurement(
            slantRangeNM,
            0.0f,
            elevationRadians,
            radialVelocityKnots,
            time
        );

        RunScan({measurement}, frame == 0 ? 0.0f : dt);
    }

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_NEAR(
        Tracks().front().altitude,
        observerTransform.altitude +
            trueVerticalVelocityNMPerSec * (scanCount - 1) * dt * METERS_PER_NM,
        20.0f
    );
    EXPECT_NEAR(
        Tracks().front().verticalVelocityNMPerSec,
        trueVerticalVelocityNMPerSec,
        0.01f
    );
}

TEST_F(TrackingSystemTests, TrackExpiresAfterDeletionTimeout)
{
    RunScan({MeasurementAt(10.0f)}, 0.0f);
    RunScan({}, TRACK_DELETION_TIMEOUT_SEC + 0.1f);
    EXPECT_TRUE(Tracks().empty());
}

TEST_F(TrackingSystemTests, InvalidObserverIsRemovedFromTrackStorage)
{
    RunScan({MeasurementAt(10.0f)}, 0.0f);
    ASSERT_TRUE(Data().activeTracks.contains(observer));

    registry.destroy(observer);
    Data().frameMeasurements.clear();
    Systems::TrackingSystem(registry, 0.1f);

    EXPECT_FALSE(Data().activeTracks.contains(observer));
}
