#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "RadarTestHelpers.h"
#include "utils/TrackFusion.h"

TEST(TrackFusionTests, MatchingInitializedTracksHaveZeroDistance)
{
    const RadarTrack a = radar_test::MakeInitializedTrack(
        {10.0f, 5.0f}, 1000.0f, {0.1f, 0.0f}, 0.0f, 1.0f);
    const RadarTrack b = a;

    float score = -1.0f;
    ASSERT_TRUE(TrackFusion::CalculateTrackDistance(a, b, score));
    EXPECT_NEAR(score, 0.0f, 1.0e-6f);
}

TEST(TrackFusionTests, RejectsUninitializedTracks)
{
    RadarTrack a{};
    RadarTrack b{};
    float score = 0.0f;

    EXPECT_FALSE(TrackFusion::CalculateTrackDistance(a, b, score));
}

TEST(TrackFusionTests, RejectsTrackEstimatesAtDifferentTimestamps)
{
    const RadarTrack a = radar_test::MakeInitializedTrack(
        {10.0f, 5.0f}, 1000.0f, {0.1f, 0.0f}, 0.0f, 1.0f);
    const RadarTrack b = radar_test::MakeInitializedTrack(
        {10.0f, 5.0f}, 1000.0f, {0.1f, 0.0f}, 0.0f, 1.1f);

    float score = 0.0f;
    EXPECT_FALSE(TrackFusion::CalculateTrackDistance(a, b, score));
}

TEST(TrackFusionTests, ComputesFiniteDistanceForNearbyTracks)
{
    const RadarTrack a = radar_test::MakeInitializedTrack(
        {10.0f, 5.0f}, 1000.0f, {0.1f, 0.0f}, 0.0f, 1.0f);
    const RadarTrack b = radar_test::MakeInitializedTrack(
        {10.05f, 5.0f}, 1010.0f, {0.101f, 0.0f}, 0.001f, 1.0f);

    float score = 0.0f;
    ASSERT_TRUE(TrackFusion::CalculateTrackDistance(a, b, score));
    EXPECT_TRUE(std::isfinite(score));
    EXPECT_GE(score, 0.0f);
}

TEST(TrackFusionTests, UsesPositionOnlyWhenEitherTrackLacksVelocity)
{
    const RadarTrack positionOnly = radar_test::MakeInitializedTrack(
        {10.0f, 5.0f}, 1000.0f, {0.0f, 0.0f}, 0.0f, 1.0f,
        ThreatClass::UNKNOWN, false);
    const RadarTrack moving = radar_test::MakeInitializedTrack(
        {10.0f, 5.0f}, 1000.0f, {1.0f, 0.0f}, 0.1f, 1.0f,
        ThreatClass::UNKNOWN, true);

    float score = -1.0f;
    ASSERT_TRUE(TrackFusion::CalculateTrackDistance(positionOnly, moving, score));
    EXPECT_NEAR(score, 0.0f, 1.0e-6f);
}

TEST(TrackFusionTests, CovarianceIntersectionProducesValidFusedState)
{
    const RadarTrack a = radar_test::MakeInitializedTrack(
        {10.0f, 5.0f}, 1000.0f, {0.1f, 0.02f}, 0.001f, 1.0f);
    const RadarTrack b = radar_test::MakeInitializedTrack(
        {10.1f, 5.05f}, 1010.0f, {0.11f, 0.025f}, 0.002f, 1.0f);

    KalmanState3D fused{};
    ASSERT_TRUE(TrackFusion::CovarianceIntersection(
        a.kalman,
        b.kalman,
        fused
    ));

    EXPECT_TRUE(fused.initialized);
    EXPECT_FLOAT_EQ(fused.timestampSec, 1.0f);

    // The fused state should be finite and remain between the inputs
    // for each component in this equal-covariance test case.
    for (size_t i = 0; i < fused.x.size(); ++i)
    {
        EXPECT_TRUE(std::isfinite(fused.x[i]));
        const float low = std::min(a.kalman.x[i], b.kalman.x[i]);
        const float high = std::max(a.kalman.x[i], b.kalman.x[i]);
        EXPECT_GE(fused.x[i], low - 1.0e-5f);
        EXPECT_LE(fused.x[i], high + 1.0e-5f);
    }

    for (int row = 0; row < 6; ++row)
    {
        EXPECT_GT(fused.P[row * 6 + row], 0.0f);
        for (int col = 0; col < 6; ++col)
        {
            EXPECT_TRUE(std::isfinite(fused.P[row * 6 + col]));
            EXPECT_NEAR(
                fused.P[row * 6 + col],
                fused.P[col * 6 + row],
                1.0e-5f
            );
        }
    }
}

TEST(TrackFusionTests, CovarianceIntersectionRejectsTimestampMismatch)
{
    const RadarTrack a = radar_test::MakeInitializedTrack(
        {10.0f, 5.0f}, 1000.0f, {0.1f, 0.0f}, 0.0f, 1.0f);
    const RadarTrack b = radar_test::MakeInitializedTrack(
        {10.0f, 5.0f}, 1000.0f, {0.1f, 0.0f}, 0.0f, 2.0f);

    KalmanState3D fused{};
    EXPECT_FALSE(TrackFusion::CovarianceIntersection(
        a.kalman,
        b.kalman,
        fused
    ));
}
