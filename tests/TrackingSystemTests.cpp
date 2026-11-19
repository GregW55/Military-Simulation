#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>
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
        float timestamp = 0.0f) const
    {
        const float dx = x - observerTransform.pos.x;
        const float dy = y - observerTransform.pos.y;
        const float dz = (altitudeMeters - observerTransform.altitude) / METERS_PER_NM;
        const float horizontal = std::sqrt(dx * dx + dy * dy);
        return {
            std::sqrt(horizontal * horizontal + dz * dz),
            std::atan2(dy, dx),
            std::atan2(dz, horizontal),
            radialVelocityKnots,
            timestamp
        };
    }

    void Submit(std::initializer_list<RadarMeasurement> measurements)
    {
        Data().frameMeasurements.insert_or_assign(
            observer, std::vector<RadarMeasurement>(measurements));
    }

    const std::vector<RadarTrack>& Tracks() const
    {
        return registry.ctx().get<RadarDetectionData>().activeTracks.at(observer);
    }
};

TEST_F(TrackingSystemTests, FirstMeasurementCreatesOneInitializedTrack)
{
    Submit({MeasurementAt(10.0f)});
    Systems::TrackingSystem(registry, 0.0f);

    ASSERT_EQ(Tracks().size(), 1u);
    const RadarTrack& track = Tracks().front();

    EXPECT_TRUE(track.kalman.initialized);
    EXPECT_EQ(track.kalman.measurementCount, 1u);
    EXPECT_FALSE(track.hasVelocity);
    EXPECT_NEAR(track.pos.x, 10.0f, 1.0e-4f);
    EXPECT_NEAR(track.pos.y, 0.0f, 1.0e-4f);
    EXPECT_NEAR(track.ageSec, 0.0f, 1.0e-5f);
    EXPECT_FALSE(track.isMissing);
}

TEST_F(TrackingSystemTests, SecondMeasurementCorrectsExistingTrack)
{
    Submit({MeasurementAt(10.0f, 0.0f, 5000.0f, -1000.0f, 0.0f)});
    Systems::TrackingSystem(registry, 0.0f);

    Submit({MeasurementAt(9.86f, 0.0f, 5000.0f, -1000.0f, 0.5f)});
    Systems::TrackingSystem(registry, 0.5f);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_EQ(Tracks().front().kalman.measurementCount, 2u);
    EXPECT_TRUE(Tracks().front().hasVelocity);
    EXPECT_FALSE(Tracks().front().isMissing);
    EXPECT_NEAR(Tracks().front().ageSec, 0.0f, 1.0e-5f);
}

TEST_F(TrackingSystemTests, ConstantVelocityTargetRemainsOneTrackAcrossManyScans)
{
    constexpr float dt = 0.5f;
    constexpr float vx = -1000.0f / 3600.0f;
    float targetX = 10.0f;

    for (int frame = 0; frame < 12; ++frame)
    {
        Submit({MeasurementAt(targetX, 0.0f, 5000.0f, -1000.0f, frame * dt)});
        Systems::TrackingSystem(registry, frame == 0 ? 0.0f : dt);
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
    Submit({
        MeasurementAt(10.0f, 0.0f, 5000.0f, -1000.0f),
        MeasurementAt(30.0f, 0.0f, 5000.0f, -1000.0f)
    });
    Systems::TrackingSystem(registry, 0.0f);

    ASSERT_EQ(Tracks().size(), 2u);
    EXPECT_NEAR(Tracks()[0].pos.x, 10.0f, 1.0e-4f);
    EXPECT_NEAR(Tracks()[1].pos.x, 30.0f, 1.0e-4f);
}

TEST_F(TrackingSystemTests, UnmatchedTrackIsMarkedMissingAndPredicted)
{
    Submit({MeasurementAt(10.0f)});
    Systems::TrackingSystem(registry, 0.0f);
    ASSERT_EQ(Tracks().size(), 1u);

    const float oldX = Tracks().front().pos.x;
    // A map entry with an empty vector represents a completed scan with no detections.
    Submit({});
    Systems::TrackingSystem(registry, 0.5f);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_TRUE(Tracks().front().isMissing);
    EXPECT_NEAR(Tracks().front().ageSec, 0.5f, 1.0e-5f);
    EXPECT_NE(Tracks().front().pos.x, oldX);
}

TEST_F(TrackingSystemTests, ReturningMeasurementReacquiresExistingTrack)
{
    Submit({MeasurementAt(10.0f)});
    Systems::TrackingSystem(registry, 0.0f);

    Submit({});
    Systems::TrackingSystem(registry, 0.5f);
    ASSERT_TRUE(Tracks().front().isMissing);

    Submit({MeasurementAt(10.0f, 0.0f, 5000.0f, -1000.0f, 1.0f)});
    Systems::TrackingSystem(registry, 0.5f);

    ASSERT_EQ(Tracks().size(), 1u);
    EXPECT_FALSE(Tracks().front().isMissing);
    EXPECT_NEAR(Tracks().front().ageSec, 0.0f, 1.0e-5f);
    EXPECT_EQ(Tracks().front().kalman.measurementCount, 2u);
}

TEST_F(TrackingSystemTests, TrackExpiresAfterDeletionTimeout)
{
    Submit({MeasurementAt(10.0f)});
    Systems::TrackingSystem(registry, 0.0f);

    Submit({});
    Systems::TrackingSystem(registry, TRACK_DELETION_TIMEOUT_SEC + 0.1f);

    EXPECT_TRUE(Tracks().empty());
}

TEST_F(TrackingSystemTests, InvalidObserverIsRemovedFromTrackStorage)
{
    Submit({MeasurementAt(10.0f)});
    Systems::TrackingSystem(registry, 0.0f);
    ASSERT_TRUE(Data().activeTracks.contains(observer));

    registry.destroy(observer);
    Data().frameMeasurements.clear();
    Systems::TrackingSystem(registry, 0.1f);

    EXPECT_FALSE(Data().activeTracks.contains(observer));
}
