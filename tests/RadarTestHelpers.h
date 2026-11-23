#pragma once

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "core/Constants.h"
#include "ecs/components.h"
#include "utils/MathUtils.h"

namespace radar_test {

inline constexpr float kDefaultRangeSigmaNM = 0.01f;
inline constexpr float kDefaultAngleSigmaRadians = 0.001f;
inline constexpr float kDefaultRadialVelocitySigmaKnots = 5.0f;

// Creates a lightweight track estimate for tests that do not need
// an initialized Kalman filter (for example, simple UI/data tests).
inline RadarTrack MakeLocalTrack(
    MathUtils::Vec2 position = {10.0f, 10.0f},
    ThreatClass classification = ThreatClass::UNKNOWN,
    MathUtils::Vec2 velocity = {0.0f, 0.0f},
    bool hasVelocity = true,
    uint32_t globalId = NO_GLOBAL_TRACK,
    float ageSec = 0.0f,
    bool missing = false,
    float altitudeMeters = 30.0f)
{
    RadarTrack track{};
    track.globalTrackId = globalId;
    track.pos = position;
    track.latestPing = position;
    track.vel = velocity;
    track.altitude = altitudeMeters;
    track.ageSec = ageSec;
    track.isMissing = missing;
    track.classification = classification;
    track.hasVelocity = hasVelocity;
    track.speedKnots = MathUtils::NmPerSecToKnots(MathUtils::Length(velocity));
    return track;
}

// Creates a fully initialized 6-state local/global estimate suitable for
// TrackFusion and SensorFusionSystem tests. Position is NM, altitude is m,
// velocity is NM/sec, and timestamp is simulation seconds.
inline RadarTrack MakeInitializedTrack(
    MathUtils::Vec2 position,
    float altitudeMeters,
    MathUtils::Vec2 velocity,
    float verticalVelocityNMPerSec,
    float timestampSec,
    ThreatClass classification = ThreatClass::UNKNOWN,
    bool hasVelocity = true,
    float positionVariance = 0.01f,
    float velocityVariance = 0.0025f)
{
    RadarTrack track{};

    track.globalTrackId = NO_GLOBAL_TRACK;
    track.latestPing = position;
    track.pos = position;
    track.vel = velocity;
    track.altitude = altitudeMeters;
    track.verticalVelocityNMPerSec = verticalVelocityNMPerSec;
    track.ageSec = 0.0f;
    track.lastMeasurementTimeSec = timestampSec;
    track.hasVelocity = hasVelocity;
    track.isMissing = false;
    track.lifecycle = TrackLifecycle::CONFIRMED;
    track.classification = classification;
    track.speedKnots = MathUtils::NmPerSecToKnots(MathUtils::Length(velocity));

    track.kalman.x = {
        position.x,
        position.y,
        altitudeMeters / METERS_PER_NM,
        velocity.x,
        velocity.y,
        verticalVelocityNMPerSec
    };

    track.kalman.P.fill(0.0f);
    for (int axis = 0; axis < 3; ++axis)
    {
        track.kalman.P[axis * 6 + axis] = positionVariance;
        track.kalman.P[(axis + 3) * 6 + (axis + 3)] = velocityVariance;
    }

    track.kalman.timestampSec = timestampSec;
    track.kalman.measurementCount = hasVelocity ? 2u : 1u;
    track.kalman.processNoiseAccelPSD = KALMAN_PSD_UNKNOWN;
    track.kalman.initialized = true;

    return track;
}

// Constructs the current RadarMeasurement layout explicitly. This avoids
// fragile aggregate initialization when uncertainty fields are added.
inline RadarMeasurement MakeMeasurement(
    float rangeNM,
    float azimuthRadians,
    float elevationRadians = 0.0f,
    float radialVelocityKnots = 0.0f,
    float timestampSec = 0.0f,
    float rangeSigmaNM = kDefaultRangeSigmaNM,
    float azimuthSigmaRadians = kDefaultAngleSigmaRadians,
    float elevationSigmaRadians = kDefaultAngleSigmaRadians,
    float radialVelocitySigmaKnots = kDefaultRadialVelocitySigmaKnots)
{
    RadarMeasurement measurement{};
    measurement.rangeNM = rangeNM;
    measurement.azimuthRadians = azimuthRadians;
    measurement.elevationRadians = elevationRadians;
    measurement.radialVelocityKnots = radialVelocityKnots;
    measurement.rangeSigmaNM = rangeSigmaNM;
    measurement.azimuthSigmaRadians = azimuthSigmaRadians;
    measurement.elevationSigmaRadians = elevationSigmaRadians;
    measurement.radialVelocitySigmaKnots = radialVelocitySigmaKnots;
    measurement.timestampSec = timestampSec;
    return measurement;
}

inline MathUtils::Vec2 VelocityFromKnots(
    float speedKnots,
    float directionDegrees = 0.0f)
{
    const float radians = directionDegrees * DEG_TO_RAD;
    const float speedNmPerSec = speedKnots * KNOTS_TO_NM_PER_SEC;
    return {
        std::cos(radians) * speedNmPerSec,
        std::sin(radians) * speedNmPerSec
    };
}

inline void SubmitTracks(
    RadarDetectionData& detectionData,
    entt::entity observer,
    std::vector<RadarTrack> tracks)
{
    detectionData.activeTracks.insert_or_assign(
        observer,
        std::move(tracks)
    );
}

inline void SubmitMeasurements(
    RadarDetectionData& detectionData,
    entt::entity observer,
    std::vector<RadarMeasurement> measurements)
{
    detectionData.frameMeasurements.insert_or_assign(
        observer,
        std::move(measurements)
    );
}

} // namespace radar_test
