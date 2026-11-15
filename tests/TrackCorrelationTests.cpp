
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

TEST(TrackGateTests, VelocityGateUsesVelocityErrorAndAcceleration)
{
    RadarTrack track =
        radar_test::MakeLocalTrack({}, ThreatClass::MISSILE_INBOUND,
{0.2f, 0.0f}, true);

    track.ageSec = 0.5f;

    const float t = track.ageSec;

    const float velocityError =
        MAX_PLAUSIBLE_VEL_CHANGE_KNOTS * KNOTS_TO_NM_PER_SEC;

    const float expected =
        TRACK_POSITION_TOLERANCE_NM
        + velocityError * t
        + 0.5f * TRACK_MAX_ACCEL_NM_S2 * t * t;

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

// ============================================================
// PING COMPATIBILITY TESTS
// ============================================================

TEST(PingCompatibilityTests, RejectsTrackWithZeroAge)
{
    RadarTrack track =
        radar_test::MakeLocalTrack({}, ThreatClass::MISSILE_INBOUND,
            {0.0f, 0.0f}, false);

    track.ageSec = 0.0f;

    RadarTrack ping = radar_test::MakePing({0.0f, 0.0f});

    EXPECT_FALSE(IsCompatible(
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

TEST(PingCompatibilityTests, RejectsSpeedAboveShipLimit)
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

    EXPECT_FALSE(IsCompatible(
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

TEST(PingCompatibilityTests, RejectsShipAccelerationAboveLimit)
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

    EXPECT_FALSE(IsCompatible(
        ping, track, SourceKind::PING));
}

TEST(PingCompatibilityTests, RejectsShipHeadingChangeAboveLimit)
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

    EXPECT_FALSE(IsCompatible(
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
