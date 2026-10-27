#pragma once

#include <tuple>

#include "../ecs/components.h"
#include "../core/Constants.h"
#include "MathUtils.h"

namespace TrackCorrelation {
    enum class SourceKind {
        PING,
        TRACK,
    };

    struct Candidate {
        float distSq;
        size_t sourceIndex;
        size_t targetIndex;
    };

    struct MatchPair {
        size_t ping;
        size_t track;
    };

    struct AssignmentResult {
        std::vector<MatchPair> matchedPairs;
        std::vector<char> pingUsed;
        std::vector<char> trackUsed;
        size_t originalTrackCount;
    };

    inline const RadarTrack& GetTrack(const RadarTrack& track) {
        return track;
    }

    inline const RadarTrack& GetTrack(const RadarTrack* track) {
        return *track;
    }

    inline const RadarTrack& GetTrack(const MasterTrackTable::GlobalTrack* globalTrack) {
        return globalTrack->track;
    }

    inline bool HasCompatiableVelocity(
        MathUtils::Vec2 velocityA,
        MathUtils::Vec2 velocityB)
    {
        return MathUtils::VelocityDifferenceKnots(velocityA, velocityB) <= MAX_PLAUSIBLE_VEL_CHANGE_KNOTS;
    }

    inline float GetTrackGateNM(const RadarTrack& track)
    {
        if (!track.hasVelocity)
        {
            return TRACK_POSITION_TOLERANCE_NM + TRACK_MAX_SPEED_NM_S * track.ageSec;
        }

        return TRACK_POSITION_TOLERANCE_NM + 0.5f * TRACK_MAX_ACCEL_NM_S2 * (track.ageSec * track.ageSec);
    }

    inline bool IsWithinGate(
        float distSq,
        float gateNM)
    {
        return distSq <= gateNM * gateNM;
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

        const auto [maxTurn, maxAccel, maxSpeed, maxAlt] = [&] {
            switch (track.classification) {
                case ThreatClass::SURFACE_SHIP:
                    return std::tuple{MAX_SHIP_TURN_RATE_DEG_PER_SEC, MAX_SHIP_ACCEL_KNOTS_PER_SEC,
                                      MAX_SHIP_SPEED_KTS, MAX_SHIP_ALTITUDE_M};
                default: // missile limits are the loosest, so unknowns get them too
                case ThreatClass::MISSILE_INBOUND:
                    return std::tuple{MAX_MISSILE_TURN_RATE_DEG_PER_SEC, MAX_MISSILE_ACCEL_KNOTS_PER_SEC,
                                      MAX_PLAUSIBLE_SPEED_KNOTS, MAX_MISSILE_ALTITUDE_M};
            }
        }();

        if (sourceKind == SourceKind::PING)
        {
            if (track.ageSec <= TRACK_MIN_ACTIONABLE_AGE_SEC) return false;

            const MathUtils::Vec2 impliedVelocity =
                    MathUtils::VelocityBetweenPositions(
                        track.latestPing,
                        candidate.pos,
                        track.ageSec);

            const float impliedSpeedKnots = MathUtils::NmPerSecToKnots(MathUtils::Length(impliedVelocity));

            if (impliedSpeedKnots > maxSpeed) return false;
            if (candidate.altitude > maxAlt) return false;

            if (!track.hasVelocity) return true;

            const float speedDelta = std::abs(impliedSpeedKnots - track.speedKnots);

            if (speedDelta > maxAccel * track.ageSec) return false;

            if (track.speedKnots > 0.0f && impliedSpeedKnots > 0.0f) {
                const float trackAngle = MathUtils::GetVectorDirectionDegrees(track.vel);

                const float candidateAngle = MathUtils::GetVectorDirectionDegrees(impliedVelocity);

                const float angleDiff = std::abs(MathUtils::GetShortestAngleDiff(trackAngle, candidateAngle));

                if (angleDiff > maxTurn * track.ageSec) return false;
            }

            return true;
        }

        if (sourceKind == SourceKind::TRACK)
        {
            if (!track.hasVelocity || !candidate.hasVelocity) return true;

            if (!HasCompatiableVelocity(track.vel, candidate.vel)) return false;

            if (MathUtils::LengthSq(candidate.vel) > 0.0f &&
                MathUtils::LengthSq(track.vel) > 0.0f)
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

    template <
        typename Sources,
        typename Targets
    >
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
                if (!IsWithinGate(distSq, gateNM)) continue;                                // Cheap check first
                if (!IsCompatible(source, target, sourceKind)) continue;

                candidates.push_back({distSq, s, t});
            }
        }

        return candidates;
    }
}