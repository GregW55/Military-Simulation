
#include <gtest/gtest.h>

#include "RadarTestHelpers.h"
#include "utils/TrackCorrelation.h"

using namespace TrackCorrelation;

// ============================================================
// GATE TESTS
// ============================================================

TEST(TrackGateTests, NoVelocityGateUsesMaximumSpeedAndAge)
{
    RadarTrack track =
            radar_test::MakeLocalTrack({}, ThreatClass::UNKNOWN, {}, false);

    track.ageSec = 0.5f;

    const float expected =
        TRACK_POSITION_TOLERANCE_NM
        + TRACK_MAX_SPEED_NM_S * track.ageSec;

    EXPECT_NEAR(GetTrackGateNM(track), expected, 0.000001f);
}

TEST(TrackGateTests, KnownVelocityWithoutInitializedKalmanUsesFallbackGate)
{
    RadarTrack track =
        radar_test::MakeLocalTrack(
            {},
            ThreatClass::MISSILE_INBOUND,
            {0.2f, 0.0f},
            true);

    track.ageSec = 0.5f;

    const float expected =
        TRACK_POSITION_TOLERANCE_NM
        + TRACK_MAX_SPEED_NM_S * track.ageSec;

    EXPECT_NEAR(GetTrackGateNM(track), expected, 0.000001f);
}

TEST(TrackGateTests, GateGrowsAsTrackGetsOlder)
{
    RadarTrack track =
        radar_test::MakeLocalTrack({}, ThreatClass::MISSILE_INBOUND,
            {0.2f, 0.0f}, true);

    track.ageSec = 0.25f;
    const float gate1 = GetTrackGateNM(track);

    track.ageSec = 0.5f;
    const float gate2 = GetTrackGateNM(track);

    track.ageSec = 1.0f;
    const float gate3 = GetTrackGateNM(track);

    EXPECT_LT(gate1, gate2);
    EXPECT_LT(gate2, gate3);
}

TEST(TrackGateTests, ExactGateBoundaryIsAccepted)
{
    constexpr float gate = 0.25f;

    EXPECT_TRUE(IsWithinGate(gate * gate, gate));
}

TEST(TrackGateTests, PositionOutsideGateIsRejected)
{
    constexpr float gate = 0.25f;

    const float distance = gate + 0.001f;

    EXPECT_FALSE(IsWithinGate(distance * distance, gate));
}

TEST(TrackGateTests, InitializedTrackGateUsesLargestPositionCovarianceEigenvalue)
{
    RadarTrack track = radar_test::MakeLocalTrack(
        {}, ThreatClass::MISSILE_INBOUND, {}, false);

    track.kalman.initialized = true;
    track.kalman.P.fill(0.0f);
    track.kalman.P[0] = 0.04f;
    track.kalman.P[7] = 0.01f;
    track.kalman.P[1] = 0.0f;
    track.kalman.P[6] = 0.0f;

    const float expected =
        TRACK_POSITION_TOLERANCE_NM + 3.034f * 0.2f;

    EXPECT_NEAR(GetTrackGateNM(track), expected, 1.0e-5f);
}

TEST(TrackGateTests, LargerPositionUncertaintyProducesLargerAssociationGate)
{
    RadarTrack narrow = radar_test::MakeLocalTrack(
        {}, ThreatClass::MISSILE_INBOUND, {}, false);
    RadarTrack broad = narrow;

    narrow.kalman.initialized = true;
    broad.kalman.initialized = true;
    narrow.kalman.P.fill(0.0f);
    broad.kalman.P.fill(0.0f);

    narrow.kalman.P[0] = narrow.kalman.P[7] = 0.01f;
    broad.kalman.P[0] = broad.kalman.P[7] = 0.25f;

    EXPECT_LT(GetTrackGateNM(narrow), GetTrackGateNM(broad));
}

// ============================================================
// PING COMPATIBILITY TESTS
// ============================================================

TEST(PingCompatibilityTests, AcceptsPingAtZeroAgeWhenAltitudeIsValid)
{
    RadarTrack track =
        radar_test::MakeLocalTrack({}, ThreatClass::MISSILE_INBOUND,
            {0.0f, 0.0f}, false);

    track.ageSec = 0.0f;

    RadarTrack ping = radar_test::MakePing({0.0f, 0.0f});

    EXPECT_TRUE(IsCompatible(
        ping, track, SourceKind::PING));
}

TEST(PingCompatibilityTests, AcceptsConstantVelocityMissile)
{
    constexpr float age = 0.5f;
    constexpr float speedKnots = 1500.0f;

    const MathUtils::Vec2 velocity =
            radar_test::VelocityFromKnots(speedKnots);

    RadarTrack track = radar_test::MakeLocalTrack(
        {0.0f, 0.0f},
        ThreatClass::MISSILE_INBOUND,
        velocity,
        true,
        NO_GLOBAL_TRACK,
        age);

    track.latestPing = {0.0f, 0.0f};
    track.speedKnots = speedKnots;

    const MathUtils::Vec2 newPosition =
        MathUtils::Scale(velocity, age);

    RadarTrack ping = radar_test::MakePing(newPosition);

    EXPECT_TRUE(IsCompatible(
        ping, track, SourceKind::PING));
}

TEST(PingCompatibilityTests, DoesNotInferSpeedFromSinglePing)
{
    constexpr float age = 1.0f;

    RadarTrack track = radar_test::MakeLocalTrack(
        {0.0f, 0.0f},
        ThreatClass::SURFACE_SHIP,
        {0.0f, 0.0f},
        false);

    track.ageSec = age;
    track.latestPing = {0.0f, 0.0f};

    const float impossibleSpeed =
        MAX_SHIP_SPEED_KTS + 100.0f;

    const MathUtils::Vec2 velocity =
            radar_test::VelocityFromKnots(impossibleSpeed);

    RadarTrack ping = radar_test::MakePing(
        MathUtils::Scale(velocity, age));

    EXPECT_TRUE(IsCompatible(
        ping, track, SourceKind::PING));
}

TEST(PingCompatibilityTests, RejectsAltitudeAboveClassLimit)
{
    RadarTrack track = radar_test::MakeLocalTrack(
        {},
        ThreatClass::SURFACE_SHIP,
        {0.0f, 0.0f},
        false);

    track.ageSec = 1.0f;

    RadarTrack ping = radar_test::MakePing(
        {0.0f, 0.0f},
        MAX_SHIP_ALTITUDE_M + 100.0f);

    EXPECT_FALSE(IsCompatible(
        ping, track, SourceKind::PING));
}

TEST(PingCompatibilityTests, DoesNotInferAccelerationFromSinglePing)
{
    constexpr float age = 1.0f;

    const float startingSpeed = MAX_SHIP_SPEED_KTS * 0.5f;

    RadarTrack track = radar_test::MakeLocalTrack(
        {},
        ThreatClass::SURFACE_SHIP,
        radar_test::VelocityFromKnots(startingSpeed),
        true,
        NO_GLOBAL_TRACK,
        age);

    track.latestPing = {0.0f, 0.0f};
    track.speedKnots = startingSpeed;

    const float impossibleSpeed =
        startingSpeed
        + MAX_SHIP_ACCEL_KNOTS_PER_SEC * age
        + 0.5f;

    ASSERT_LT(impossibleSpeed, MAX_SHIP_SPEED_KTS);

    RadarTrack ping = radar_test::MakePing(
        MathUtils::Scale(
            radar_test::VelocityFromKnots(impossibleSpeed), age));

    EXPECT_TRUE(IsCompatible(
        ping, track, SourceKind::PING));
}

TEST(PingCompatibilityTests, DoesNotInferHeadingChangeFromSinglePing)
{
    constexpr float age = 1.0f;
    constexpr float speed = MAX_SHIP_SPEED_KTS * 0.5f;

    ASSERT_GT(speed, MIN_SPEED_FOR_HEADING_KNOTS);

    RadarTrack track = radar_test::MakeLocalTrack(
        {},
        ThreatClass::SURFACE_SHIP,
        radar_test::VelocityFromKnots(speed, 0.0f),
        true,
        NO_GLOBAL_TRACK,
        age);

    track.latestPing = {0.0f, 0.0f};
    track.speedKnots = speed;

    RadarTrack ping = radar_test::MakePing(
        MathUtils::Scale(
            radar_test::VelocityFromKnots(speed, 90.0f), age));

    EXPECT_TRUE(IsCompatible(
        ping, track, SourceKind::PING));
}

// ============================================================
// TRACK-TO-TRACK COMPATIBILITY TESTS
// ============================================================

TEST(TrackCompatibilityTests, RejectsCandidateWithoutVelocity)
{
    RadarTrack candidate = radar_test::MakeLocalTrack(
        {},
        ThreatClass::MISSILE_INBOUND,
        {0.0f, 0.0f},
        false);

    RadarTrack track = radar_test::MakeLocalTrack(
        {},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(1000.0f),
        true);

    EXPECT_FALSE(IsCompatible(
        candidate, track, SourceKind::TRACK));
}

TEST(TrackCompatibilityTests, AcceptsMatchingVelocityAndClassification)
{
    const MathUtils::Vec2 velocity =
        radar_test::VelocityFromKnots(1000.0f);

    RadarTrack candidate = radar_test::MakeLocalTrack(
        {10.0f, 10.0f},
        ThreatClass::MISSILE_INBOUND,
        velocity,
        true);

    RadarTrack track = radar_test::MakeLocalTrack(
        {10.0f, 10.0f},
        ThreatClass::MISSILE_INBOUND,
        velocity,
        true);

    EXPECT_TRUE(IsCompatible(
        candidate, track, SourceKind::TRACK));
}

TEST(TrackCompatibilityTests, RejectsConflictingKnownClassifications)
{
    RadarTrack candidate = radar_test::MakeLocalTrack(
        {},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(1000.0f),
        true);

    RadarTrack track = radar_test::MakeLocalTrack(
        {},
        ThreatClass::SURFACE_SHIP,
        radar_test::VelocityFromKnots(1000.0f),
        true);

    EXPECT_FALSE(IsCompatible(
        candidate, track, SourceKind::TRACK));
}

TEST(TrackCompatibilityTests, RejectsExcessiveVelocityDifference)
{
    RadarTrack candidate = radar_test::MakeLocalTrack(
        {},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(1500.0f),
        true);

    RadarTrack track = radar_test::MakeLocalTrack(
        {},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(500.0f),
        true);

    EXPECT_FALSE(IsCompatible(
        candidate, track, SourceKind::TRACK));
}

TEST(TrackCompatibilityTests, RejectsExcessiveHeadingDifference)
{
    RadarTrack candidate = radar_test::MakeLocalTrack(
        {},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(100.0f, 60.0f),
        true);

    RadarTrack track = radar_test::MakeLocalTrack(
        {},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(100.0f, 0.0f),
        true);

    EXPECT_FALSE(IsCompatible(
        candidate, track, SourceKind::TRACK));
}

// ============================================================
// CANDIDATE GENERATION TESTS
// ============================================================

TEST(CandidateGenerationTests, ReturnsCandidateInsideGate)
{
    RadarTrack source = radar_test::MakeLocalTrack(
        {10.0f, 10.0f},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(1000.0f),
        true);

    RadarTrack target = source;

    std::vector<RadarTrack> sources{source};
    std::vector<RadarTrack> targets{target};

    const auto candidates = GenerateCandidates(
        sources, targets, SourceKind::TRACK);

    ASSERT_EQ(candidates.size(), 1);
    EXPECT_EQ(candidates[0].sourceIndex, 0);
    EXPECT_EQ(candidates[0].targetIndex, 0);
}

TEST(CandidateGenerationTests, RejectsCandidateOutsideGate)
{
    RadarTrack source = radar_test::MakeLocalTrack(
        {100.0f, 100.0f},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(1000.0f),
        true);

    RadarTrack target = radar_test::MakeLocalTrack(
        {0.0f, 0.0f},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(1000.0f),
        true);

    std::vector<RadarTrack> sources{source};
    std::vector<RadarTrack> targets{target};

    const auto candidates = GenerateCandidates(
        sources, targets, SourceKind::TRACK);

    EXPECT_TRUE(candidates.empty());
}

TEST(CandidateGenerationTests, PreservesSourceAndTargetIndices)
{
    std::vector<RadarTrack> sources;
    std::vector<RadarTrack> targets;

    sources.push_back(radar_test::MakeLocalTrack(
        {10.0f, 10.0f},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(1000.0f),
        true));

    sources.push_back(radar_test::MakeLocalTrack(
        {20.0f, 20.0f},
        ThreatClass::MISSILE_INBOUND,
        radar_test::VelocityFromKnots(1000.0f),
        true));

    targets.push_back(sources[0]);
    targets.push_back(sources[1]);

    const auto candidates = GenerateCandidates(
        sources, targets, SourceKind::TRACK);

    ASSERT_EQ(candidates.size(), 2);

    EXPECT_EQ(candidates[0].sourceIndex, 0);
    EXPECT_EQ(candidates[0].targetIndex, 0);

    EXPECT_EQ(candidates[1].sourceIndex, 1);
    EXPECT_EQ(candidates[1].targetIndex, 1);
}
