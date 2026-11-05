#pragma once

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "core/Constants.h"
#include "ecs/components.h"
#include "utils/MathUtils.h"

namespace radar_test {

inline constexpr float kEpsilon = 1.0e-3f;
inline constexpr float kObserverAltitudeM = 30.0f;

inline RadarTrack MakePing(
    MathUtils::Vec2 position,
    float altitude = kObserverAltitudeM)
{
    RadarTrack ping{};
    ping.pos = position;
    ping.altitude = altitude;
    return ping;
}

inline RadarTrack MakeCorrelationTrack(
    MathUtils::Vec2 latestPing,
    MathUtils::Vec2 velocity = {0.0f, 0.0f},
    float ageSec = 1.0f,
    ThreatClass classification = ThreatClass::UNKNOWN,
    bool hasVelocity = false)
{
    RadarTrack track{};
    track.latestPing = latestPing;
    track.pos = latestPing;
    track.vel = velocity;
    track.ageSec = ageSec;
    track.classification = classification;
    track.hasVelocity = hasVelocity;
    track.speedKnots = MathUtils::NmPerSecToKnots(MathUtils::Length(velocity));
    track.isMissing = false;
    return track;
}

inline RadarTrack MakeLocalTrack(
    MathUtils::Vec2 position,
    ThreatClass classification = ThreatClass::UNKNOWN,
    MathUtils::Vec2 velocity = {0.0f, 0.0f},
    bool hasVelocity = true,
    uint32_t globalId = NO_GLOBAL_TRACK,
    float ageSec = 0.0f,
    bool missing = false,
    float altitude = kObserverAltitudeM)
{
    RadarTrack track{};
    track.globalTrackId = globalId;
    track.pos = position;
    track.latestPing = position;
    track.vel = velocity;
    track.altitude = altitude;
    track.ageSec = ageSec;
    track.isMissing = missing;
    track.classification = classification;
    track.hasVelocity = hasVelocity;
    track.speedKnots = MathUtils::NmPerSecToKnots(MathUtils::Length(velocity));
    return track;
}

inline MathUtils::Vec2 VelocityFromKnots(
    float speedKnots,
    float directionDegrees = 0.0f)
{;
    const float radians = directionDegrees * DEG_TO_RAD;
    const float speedNmPerSec = speedKnots * KNOTS_TO_NM_PER_SEC;

    return {
        std::cos(radians) * speedNmPerSec,
        std::sin(radians) * speedNmPerSec
    };
}

inline void SubmitPings(
    RadarDetectionData& detectionData,
    entt::entity observer,
    std::vector<RadarTrack> pings)
{
    detectionData.framePings.clear();
    detectionData.framePings.emplace(observer, std::move(pings));
}

} // namespace radar_test
