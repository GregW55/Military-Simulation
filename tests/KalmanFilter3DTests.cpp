#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "RadarTestHelpers.h"
#include "utils/KalmanFilter3D.h"

class KalmanFilter3DTests : public ::testing::Test
{
protected:
    Transform2D observer{};
    Kinematics observerKinematics{};

    RadarMeasurement Measurement(
        float rangeNM = 10.0f,
        float azimuthRadians = 0.0f,
        float elevationRadians = 0.0f,
        float radialVelocityKnots = 0.0f,
        float timestampSec = 0.0f) const
    {
        return radar_test::MakeMeasurement(
            rangeNM,
            azimuthRadians,
            elevationRadians,
            radialVelocityKnots,
            timestampSec
        );
    }
};

TEST_F(KalmanFilter3DTests, InitializesWorldPositionFromRangeAzimuthAndElevation)
{
    observer.pos = {2.0f, 3.0f};
    observer.altitude = 10000.0f;

    const float elevation = std::atan2(1.0f, 10.0f);
    const float slantRange = std::sqrt(101.0f);

    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state,
        Measurement(slantRange, 0.0f, elevation),
        observer,
        observerKinematics
    );

    ASSERT_TRUE(state.initialized);
    EXPECT_NEAR(state.x[0], 12.0f, 1.0e-5f);
    EXPECT_NEAR(state.x[1], 3.0f, 1.0e-5f);
    EXPECT_NEAR(state.x[2] * METERS_PER_NM, 11852.0f, 1.0e-3f);
    EXPECT_EQ(state.measurementCount, 1u);
}

TEST_F(KalmanFilter3DTests, DoesNotCopyUnobservableObserverVerticalVelocityIntoTargetState)
{
    // Horizontal LOS: vertical observer velocity is tangential to the
    // measurement direction and is not observable in this single return.
    observer.altitude = 10000.0f;
    observerKinematics.verticalVelocityNMPerSec = 0.01f;

    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state,
        Measurement(10.0f, 0.0f, 0.0f, 0.0f),
        observer,
        observerKinematics
    );

    ASSERT_TRUE(state.initialized);
    EXPECT_NEAR(state.x[5], 0.0f, 1.0e-6f);

    KalmanFilter3D::Predict(state, 1.0f);
    EXPECT_NEAR(state.x[2] * METERS_PER_NM, 10000.0f, 1.0e-3f);
}

TEST_F(KalmanFilter3DTests, CompensatesObserverVelocityAlongLineOfSight)
{
    observerKinematics.velocity = {0.1f, 0.0f};

    // Target is stationary; observer moves away from it along +X.
    // The observer-relative Doppler measurement is -0.1 NM/sec.
    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state,
        Measurement(10.0f, 0.0f, 0.0f, -0.1f * NM_PER_SEC_TO_KNOTS),
        observer,
        observerKinematics
    );

    ASSERT_TRUE(state.initialized);
    EXPECT_NEAR(state.x[3], 0.0f, 1.0e-5f);
}

TEST_F(KalmanFilter3DTests, PredictAdvancesPositionUsingEstimatedVelocity)
{
    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state,
        Measurement(10.0f, 0.0f, 0.0f, 360.0f),
        observer,
        observerKinematics
    );

    ASSERT_TRUE(state.initialized);
    ASSERT_NEAR(state.x[3], 0.1f, 1.0e-5f);

    KalmanFilter3D::Predict(state, 2.0f);

    EXPECT_NEAR(state.x[0], 10.2f, 1.0e-5f);
    EXPECT_FLOAT_EQ(state.timestampSec, 2.0f);
}

TEST_F(KalmanFilter3DTests, CalculateInnovationRejectsMeasurementAtWrongTimestamp)
{
    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state,
        Measurement(10.0f, 0.0f, 0.0f, 0.0f, 0.0f),
        observer,
        observerKinematics
    );

    KalmanFilter3D::Predict(state, 1.0f);

    RadarInnovation innovation{};
    const bool valid = KalmanFilter3D::CalculateInnovation(
        state,
        Measurement(10.0f, 0.0f, 0.0f, 0.0f, 0.5f),
        observer,
        observerKinematics,
        innovation
    );

    EXPECT_FALSE(valid);
}

TEST_F(KalmanFilter3DTests, CorrectReducesRangeInnovation)
{
    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state,
        Measurement(10.0f, 0.0f, 0.0f, 0.0f, 0.0f),
        observer,
        observerKinematics
    );

    KalmanFilter3D::Predict(state, 1.0f);

    const RadarMeasurement updatedMeasurement =
        Measurement(9.9f, 0.0f, 0.0f, 0.0f, 1.0f);

    RadarInnovation before{};
    ASSERT_TRUE(KalmanFilter3D::CalculateInnovation(
        state,
        updatedMeasurement,
        observer,
        observerKinematics,
        before
    ));

    ASSERT_TRUE(KalmanFilter3D::Correct(
        state,
        updatedMeasurement,
        observer,
        observerKinematics
    ));

    RadarInnovation after{};
    ASSERT_TRUE(KalmanFilter3D::CalculateInnovation(
        state,
        updatedMeasurement,
        observer,
        observerKinematics,
        after
    ));

    EXPECT_LT(std::abs(after.residual[0]), std::abs(before.residual[0]));
    EXPECT_EQ(state.measurementCount, 2u);
}

TEST_F(KalmanFilter3DTests, AzimuthInnovationUsesShortestAngularDifferenceAcrossPi)
{
    constexpr float offset = 0.01f;

    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state,
        Measurement(10.0f, MATH_PI - offset, 0.0f, 0.0f, 0.0f),
        observer,
        observerKinematics
    );

    KalmanFilter3D::Predict(state, 1.0f);

    RadarInnovation innovation{};
    ASSERT_TRUE(KalmanFilter3D::CalculateInnovation(
        state,
        Measurement(10.0f, -MATH_PI + offset, 0.0f, 0.0f, 1.0f),
        observer,
        observerKinematics,
        innovation
    ));

    EXPECT_NEAR(innovation.residual[1], 2.0f * offset, 1.0e-4f);
}

TEST_F(KalmanFilter3DTests, PredictionKeepsCovarianceFiniteAndSymmetric)
{
    KalmanState3D state{};
    KalmanFilter3D::Initialize(
        state,
        Measurement(10.0f, 0.2f, 0.05f, 100.0f),
        observer,
        observerKinematics
    );

    KalmanFilter3D::Predict(state, 0.25f);
    KalmanFilter3D::Predict(state, 0.25f);

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            const float a = state.P[row * 6 + col];
            const float b = state.P[col * 6 + row];
            EXPECT_TRUE(std::isfinite(a));
            EXPECT_NEAR(a, b, 1.0e-5f);
        }

        EXPECT_GT(state.P[row * 6 + row], 0.0f);
    }
}

TEST_F(
    KalmanFilter3DTests,
    OffAxisZeroDopplerUncertaintyProducesValidInnovation)
{
    // Observer is offset from the target's horizontal axis.
    observer.pos = {0.0f, 1.0f};
    observer.altitude = 10000.0f;

    const float range = std::sqrt(101.0f);
    const float azimuth = std::atan2(-1.0f, 10.0f);

    // Exact measurements, but zero reported uncertainties.
    // The EKF must apply its uncertainty floors.
    const auto makeMeasurement = [&](float timestamp)
    {
        return radar_test::MakeMeasurement(
            range,
            azimuth,
            0.0f,
            0.0f,
            timestamp,
            0.0f,
            0.0f,
            0.0f,
            0.0f
        );
    };

    KalmanState3D state{};

    KalmanFilter3D::Initialize(
        state,
        makeMeasurement(0.0f),
        observer,
        observerKinematics
    );

    ASSERT_TRUE(state.initialized);

    KalmanFilter3D::Predict(state, 0.5f);

    RadarInnovation innovation{};

    EXPECT_TRUE(KalmanFilter3D::CalculateInnovation(
        state,
        makeMeasurement(0.5f),
        observer,
        observerKinematics,
        innovation
    ));

    EXPECT_TRUE(KalmanFilter3D::Correct(
        state,
        makeMeasurement(0.5f),
        observer,
        observerKinematics
    ));

    EXPECT_EQ(state.measurementCount, 2u);
}