#pragma once
#include <cmath>

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
        float score;                // squared Mahalanobis distance, lower = better
        std::size_t sourceIndex;
        std::size_t targetIndex;
    };

    constexpr float GATE_CHI2_2DOF = 13.8f;
    constexpr float TOLERANCE_VAR =  TRACK_POSITION_TOLERANCE_NM * TRACK_POSITION_TOLERANCE_NM;

    struct PosCov { float xx, xy, yy; };

    inline PosCov GetPositionCov(const RadarTrack& t)
    {
        if (t.kalman.initialized)
            return { t.kalman.P[0], 0.5f * (t.kalman.P[1] + t.kalman.P[6]), t.kalman.P[7] };
        const float s = TRACK_MAX_SPEED_NM_S * t.ageSec / 3.0f;             // Crude fallback
        return { s * s, 0.0f, s * s };
    }

    inline PosCov GetSourceCov(const RadarTrack& s, SourceKind kind) {
        if (kind == SourceKind::PING) {
            const float v = s.measurementSigmaNM * s.measurementSigmaNM;
            return { v, 0.0, v };
        }
        return GetPositionCov(s);
    }

    inline float Mahalanobis2(float dx, float dy, const PosCov& c)
    {
        const float det = c.xx * c.yy - c.xy * c.xy;
        return (c.yy * dx * dx - 2.0f * c.xy * dx * dy + c.xx * dy * dy) / det;
    }

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

    inline bool HasCompatiableVelocity(
        MathUtils::Vec2 velocityA,
        MathUtils::Vec2 velocityB)
    {
        return MathUtils::VelocityDifferenceKnots(velocityA, velocityB) <= MAX_PLAUSIBLE_VEL_CHANGE_KNOTS;
    }

    // 1-sigma uncertainty of a track's heading, in degrees, from its velocity covariance
    inline float HeadingSigmaDeg(const RadarTrack& t)
    {
        const float speedNmS = MathUtils::Length(t.vel);
        if (speedNmS < 1e-6f || !t.kalman.initialized) return 180.0f;

        // P[21] = var(vx), P[28] = var(vy)
        const float velSigma = std::sqrt(std::max(0.0f, 0.5f * (t.kalman.P[21] + t.kalman.P[28])));
        return std::atan2(velSigma, speedNmS) * RAD_TO_DEG;
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
            const Limits lim = GetLimits(track.classification);

            // The candidate is a position observation, not a velocity estimate.
            // Velocity compatibility is handled by the predicted EKF state
            // and the positional association gate.
            if (candidate.altitude > lim.maxAltitudeM + 3.0f * candidate.measurementSigmaNM * METERS_PER_NM)
                return false;

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
                const float angleDiff = std::abs(MathUtils::GetShortestAngleDiff(trackAngle, candidateAngle));

                // The two headings differ by noise too: allow 3 sigma of their combined uncertainty
                const float sa = HeadingSigmaDeg(candidate);
                const float sb = HeadingSigmaDeg(track);
                const float allowedDiff = std::max(TRACK_MAX_HEADING_DIFF_DEG, 3.0f * std::sqrt(sa * sa + sb * sb));

                if (allowedDiff < 180.0f && angleDiff > allowedDiff) return false;
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
            const PosCov tc = GetPositionCov(target);

            for (std::size_t s = 0; s < sources.size(); ++s) {
                const auto& source = GetTrack(sources[s]);
                const PosCov sc = GetSourceCov(source, sourceKind);

                // Offset variance = both sides' uncertainty, plus a small floor so det never hits zero
                const PosCov c { tc.xx + sc.xx + TOLERANCE_VAR,
                                 tc.xy + sc.xy,
                                 tc.yy + sc.yy + TOLERANCE_VAR };

                const float score = Mahalanobis2(source.pos.x - target.pos.x,
                                                 source.pos.y - target.pos.y, c);
                if (score > GATE_CHI2_2DOF) continue;
                if (!IsCompatible(source, target, sourceKind)) continue;

                candidates.push_back({score, s, t});
            }
        }
        return candidates;
    }
}