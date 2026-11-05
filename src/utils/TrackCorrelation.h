#pragma once

#include <iostream>

#include "../ecs/components.h"
#include "../core/Constants.h"
#include "MathUtils.h"

namespace TrackCorrelation {
    enum class SourceKind {
        PING,
        TRACK
    };

    struct Candidate {
        float distSq;
        std::size_t sourceIndex;
        std::size_t targetIndex;
    };

    struct Limits { float maxTurnRateDegPerSec, maxAccelerationKnotsPerSec, maxSpeedKnots, maxAltitudeM; };

    inline const RadarTrack& GetTrack(const RadarTrack& track) { return track; }
    inline const RadarTrack& GetTrack(const RadarTrack* track) { return *track; }
    inline const RadarTrack& GetTrack(const MasterTrackTable::GlobalTrack* globalTrack) { return globalTrack->track; }

    inline Limits GetLimits(ThreatClass c)
    {
        if (c == ThreatClass::SURFACE_SHIP)
            return { MAX_SHIP_TURN_RATE_DEG_PER_SEC, MAX_SHIP_ACCEL_KNOTS_PER_SEC,
                     MAX_SHIP_SPEED_KTS, MAX_SHIP_ALTITUDE_M };
        // MISSILE and UNKNOWN: the loose set
        return { MAX_MISSILE_TURN_RATE_DEG_PER_SEC, MAX_MISSILE_ACCEL_KNOTS_PER_SEC,
                 MAX_PLAUSIBLE_SPEED_KNOTS, MAX_MISSILE_ALTITUDE_M };
    }


    inline float GetTrackGateNM(const RadarTrack& track)
    {
        if (!track.hasVelocity)
        {
            return TRACK_POSITION_TOLERANCE_NM + TRACK_MAX_SPEED_NM_S * track.ageSec;
        }

        const float t = track.ageSec;

        const float velocityErrorNMPerSec =
            MAX_PLAUSIBLE_VEL_CHANGE_KNOTS * KNOTS_TO_NM_PER_SEC;

        return TRACK_POSITION_TOLERANCE_NM
            + velocityErrorNMPerSec * t
            + 0.5f * TRACK_MAX_ACCEL_NM_S2 * t * t;
    }

    inline bool IsWithinGate(float distSq, float gateNM)
    {
        return distSq <= gateNM * gateNM;
    }

    inline bool HasCompatiableVelocity(
        MathUtils::Vec2 velocityA,
        MathUtils::Vec2 velocityB)
    {
        return MathUtils::VelocityDifferenceKnots(velocityA, velocityB) <= MAX_PLAUSIBLE_VEL_CHANGE_KNOTS;
    }

    inline bool IsCompatible(
        const RadarTrack& candidate,        //    Ping         | local track
        const RadarTrack& track,            //    local track  | global track
        const SourceKind sourceKind)        //    Is source a ping, or a track
    {
        // Reject conflicting known classifications immediately
        if (sourceKind == SourceKind::TRACK &&
            candidate.classification != ThreatClass::UNKNOWN &&
            track.classification != ThreatClass::UNKNOWN &&
            candidate.classification != track.classification)
        {
            return false;
        }

        if (sourceKind == SourceKind::PING)
        {
            if (track.ageSec <= TRACK_MIN_ACTIONABLE_AGE_SEC) return false;

            const MathUtils::Vec2 impliedVelocity =
                    MathUtils::VelocityBetweenPositions(track.latestPing, candidate.pos, track.ageSec);

            const float impliedSpeedKnots = MathUtils::NmPerSecToKnots(MathUtils::Length(impliedVelocity));

            const Limits lim = GetLimits(track.classification);

            if (impliedSpeedKnots > lim.maxSpeedKnots) return false;
            if (candidate.altitude > lim.maxAltitudeM) return false;

            if (!track.hasVelocity) return true;

            const float speedDelta = std::abs(impliedSpeedKnots - track.speedKnots);

            if (speedDelta > lim.maxAccelerationKnotsPerSec * track.ageSec) return false;

            if (track.speedKnots > MIN_SPEED_FOR_HEADING_KNOTS &&
                impliedSpeedKnots > MIN_SPEED_FOR_HEADING_KNOTS)
            {
                const float trackAngle = MathUtils::GetVectorDirectionDegrees(track.vel);

                const float candidateAngle = MathUtils::GetVectorDirectionDegrees(impliedVelocity);

                const float angleDiff = std::abs(MathUtils::GetShortestAngleDiff(trackAngle, candidateAngle));

                if (angleDiff > lim.maxTurnRateDegPerSec * track.ageSec) return false;
            }
            return true;
        }

        if (sourceKind == SourceKind::TRACK)
        {
            if (!candidate.hasVelocity) return false;

            if (!HasCompatiableVelocity(track.vel, candidate.vel)) return false;

            const float candidateSpeed  = MathUtils::NmPerSecToKnots(MathUtils::Length(candidate.vel));
            const float trackSpeed = MathUtils::NmPerSecToKnots(MathUtils::Length(track.vel));

            if (candidateSpeed > MIN_SPEED_FOR_HEADING_KNOTS && trackSpeed > MIN_SPEED_FOR_HEADING_KNOTS)
            {
                const float candidateAngle = MathUtils::GetVectorDirectionDegrees(candidate.vel);
                const float trackAngle = MathUtils::GetVectorDirectionDegrees(track.vel);

                const float angleDiff = std::abs(
                    MathUtils::GetShortestAngleDiff(trackAngle, candidateAngle));

                if (angleDiff > TRACK_MAX_HEADING_DIFF_DEG) return false;
            }
            return true;
        }
        return false;
    }

    template <typename Sources, typename Targets>
    std::vector<Candidate> GenerateCandidates(
        const Sources& sources,
        const Targets& targets,
        const SourceKind sourceKind)
    {
        std::vector<Candidate> candidates;
        candidates.reserve(sources.size());

        for (std::size_t t = 0; t < targets.size(); ++t) {
            const auto& target = GetTrack(targets[t]);
            const float gateNM = GetTrackGateNM(target);                                    // Once per target

            for (std::size_t s = 0; s < sources.size(); ++s) {
                const auto& source = GetTrack(sources[s]);

                const float distSq = MathUtils::GetDistanceSq(source.pos, target.pos);
                if (!IsWithinGate(distSq, gateNM))
                {
                    if (sourceKind == SourceKind::PING)
                    {
                        std::fprintf(stderr,
                            "[GATE REJECT] "
                            "s=%zu t=%zu "
                            "dist=%.6f gate=%.6f age=%.3f "
                            "source=(%.6f, %.6f) "
                            "target.pos=(%.6f, %.6f) "
                            "target.latestPing=(%.6f, %.6f) "
                            "target.vel=(%.6f, %.6f) "
                            "hasVelocity=%d class=%d\n",

                            s, t,
                            std::sqrt(distSq),
                            gateNM,
                            target.ageSec,

                            source.pos.x,
                            source.pos.y,

                            target.pos.x,
                            target.pos.y,

                            target.latestPing.x,
                            target.latestPing.y,

                            target.vel.x,
                            target.vel.y,

                            static_cast<int>(target.hasVelocity),
                            static_cast<int>(target.classification));
                    }

                    continue;
                }
                if (!IsCompatible(source, target, sourceKind)) {
                    if (target.classification == source.classification) {
                        if (sourceKind == SourceKind::PING)
                            std::fprintf(stderr, "[COMPAT REJECT] dist=%.4f age=%.3f trackKts=%.0f class=%d\n",
                                std::sqrt(distSq), target.ageSec, target.speedKnots, (int)target.classification);
                    }
                    continue;
                }

                candidates.push_back({distSq, s, t});
            }
        }
        return candidates;
    }
}