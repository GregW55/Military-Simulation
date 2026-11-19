#include <gtest/gtest.h>

#include <cmath>

#include "core/Constants.h"
#include "ecs/components.h"
#include "utils/KalmanFilter3D.h"

class KalmanFilter3DTests : public ::testing::Test
{
protected:
    Transform2D observer{{0.0f, 0.0f}, 1000.0f, 0.0f};
    Kinematics observerKinematics{};
    RadarEmitter radar{};

    void SetUp() override
    {
        radar.rangeErrorNM = 0.01f;
        radar.angleErrorRadians = 0.001f;
        radar.velocityErrorKnots = 5.0f;
    }

    RadarMeasurement Measurement(
        float range,
        float azimuth = 0.0f,
        float elevation = 0.0f,
        float radialVelocity = 0.0f,
        float timestamp = 0.0f) const
    {
        return {range, azimuth, elevation, radialVelocity, timestamp};
    }
};

TEST_F(KalmanFilter3DTests, InitializeConvertsPolarMeasurementToWorldPosition)
{
    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state, Measurement(10.0f, MATH_PI / 2.0f), observer,
        observerKinematics, radar);

    ASSERT_TRUE(state.initialized);
    EXPECT_EQ(state.measurementCount, 1u);
    EXPECT_NEAR(state.x[0], 0.0f, 1.0e-4f);
    EXPECT_NEAR(state.x[1], 10.0f, 1.0e-4f);
    EXPECT_NEAR(state.x[2], observer.altitude / METERS_PER_NM, 1.0e-4f);
}

TEST_F(KalmanFilter3DTests, InitializeIncludesObserverPositionAndVelocity)
{
    observer.pos = {5.0f, -2.0f};
    observerKinematics.velocity = {0.1f, 0.2f};

    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state, Measurement(10.0f, 0.0f, 0.0f, 360.0f),
        observer, observerKinematics, radar);

    EXPECT_NEAR(state.x[0], 15.0f, 1.0e-4f);
    EXPECT_NEAR(state.x[1], -2.0f, 1.0e-4f);
    EXPECT_NEAR(state.x[3], 0.2f, 1.0e-4f);
    EXPECT_NEAR(state.x[4], 0.2f, 1.0e-4f);
}

TEST_F(KalmanFilter3DTests, PredictAppliesConstantVelocityMotionInAllThreeAxes)
{
    KalmanState3D state{};
    state.initialized = true;
    state.timestampSec = 2.0f;
    state.x = {1.0f, 2.0f, 3.0f, 0.1f, -0.2f, 0.3f};

    KalmanFilter3D::Predict(state, 2.0f);

    EXPECT_NEAR(state.x[0], 1.2f, 1.0e-5f);
    EXPECT_NEAR(state.x[1], 1.6f, 1.0e-5f);
    EXPECT_NEAR(state.x[2], 3.6f, 1.0e-5f);
    EXPECT_NEAR(state.timestampSec, 4.0f, 1.0e-5f);
}

TEST_F(KalmanFilter3DTests, PredictIncreasesPositionAndVelocityUncertainty)
{
    KalmanState3D state{};
    state.initialized = true;
    for (int i = 0; i < 6; ++i)
        state.P[i * 6 + i] = 1.0f;

    const float oldPositionVariance = state.P[0];
    const float oldVelocityVariance = state.P[21];

    KalmanFilter3D::Predict(state, 1.0f);

    EXPECT_GT(state.P[0], oldPositionVariance);
    EXPECT_GT(state.P[21], oldVelocityVariance);
}

TEST_F(KalmanFilter3DTests, CorrectPullsEstimateTowardRadarMeasurement)
{
    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state, Measurement(10.0f), observer, observerKinematics, radar);

    KalmanFilter3D::Predict(state, 0.5f);
    const float beforeError = std::abs(state.x[0] - 9.9f);

    const bool corrected = KalmanFilter3D::Correct(
        state, Measurement(9.9f, 0.0f, 0.0f, 0.0f, 0.5f),
        observer, observerKinematics, radar);

    ASSERT_TRUE(corrected);
    EXPECT_LT(std::abs(state.x[0] - 9.9f), beforeError);
    EXPECT_EQ(state.measurementCount, 2u);
}

TEST_F(KalmanFilter3DTests, CorrectRejectsUninitializedState)
{
    KalmanState3D state{};
    EXPECT_FALSE(KalmanFilter3D::Correct(
        state, Measurement(10.0f), observer, observerKinematics, radar));
}

TEST_F(KalmanFilter3DTests, CorrectRejectsMeasurementAtObserverPosition)
{
    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state, Measurement(0.0f), observer, observerKinematics, radar);

    EXPECT_FALSE(KalmanFilter3D::Correct(
        state, Measurement(0.0f), observer, observerKinematics, radar));
}

TEST_F(KalmanFilter3DTests, CopyToTrackTransfersPositionVelocityAndAltitude)
{
    RadarTrack track{};
    track.kalman.initialized = true;
    track.kalman.measurementCount = 2;
    track.kalman.x = {1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f};

    KalmanFilter3D::CopyToTrack(track);

    EXPECT_NEAR(track.pos.x, 1.0f, 1.0e-6f);
    EXPECT_NEAR(track.pos.y, 2.0f, 1.0e-6f);
    EXPECT_NEAR(track.vel.x, 0.1f, 1.0e-6f);
    EXPECT_NEAR(track.vel.y, 0.2f, 1.0e-6f);
    EXPECT_NEAR(track.altitude, 3.0f * METERS_PER_NM, 1.0e-3f);
    EXPECT_TRUE(track.hasVelocity);
}

TEST_F(KalmanFilter3DTests, CovarianceRemainsSymmetricAfterPredictionAndCorrection)
{
    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state, Measurement(10.0f, 0.2f, 0.01f, -500.0f),
        observer, observerKinematics, radar);

    KalmanFilter3D::Predict(state, 0.25f);
    ASSERT_TRUE(KalmanFilter3D::Correct(
        state, Measurement(9.95f, 0.2f, 0.01f, -500.0f, 0.25f),
        observer, observerKinematics, radar));

    for (int row = 0; row < 6; ++row)
        for (int col = 0; col < 6; ++col)
            EXPECT_NEAR(
                state.P[row * 6 + col],
                state.P[col * 6 + row],
                1.0e-4f) << "row=" << row << " col=" << col;
}
