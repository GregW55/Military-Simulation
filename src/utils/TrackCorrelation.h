#pragma once

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
        const RadarTrack& source,       //    Ping         | local track
        const RadarTrack& target,       //    local track  | global track
        const SourceKind sourceKind)    //    Is source a ping or track
    {
        if (sourceKind == SourceKind::PING) {
            if (target.ageSec <= 0.0001f) return true;

            const MathUtils::Vec2 impliedVel =
                MathUtils::VelocityBetweenPositions(
                    target.latestPing,
                    source.pos,
                    target.ageSec);

            const float impliedSpeedKnots = MathUtils::NmPerSecToKnots(MathUtils::Length(impliedVel));

            if (impliedSpeedKnots > MAX_PLAUSIBLE_SPEED_KNOTS) return false;

            if (target.hasVelocity) {
                const float speedDelta = std::abs(impliedSpeedKnots - target.speedKnots);

                if (speedDelta > 1000.0f) return false;

                if (target.speedKnots > 0.0f && impliedSpeedKnots > 0.0f) {
                    const float targetAngle = MathUtils::GetVectorDirectionDegrees(target.vel);

                    const float sourceAngle = MathUtils::GetVectorDirectionDegrees(impliedVel);

                    const float angleDiff = std::abs(MathUtils::GetShortestAngleDiff(targetAngle, sourceAngle));

                    if (angleDiff > 30.0f) return false;
                }
            }
            // The pair passed the checks
            // Not all checks need to pass. if (!target.hasVelocity) then only max plausible speed needs to be checked
            return true;
        }

        if (sourceKind == SourceKind::TRACK) {
            if (source.hasVelocity && target.hasVelocity)
            {
                if (!HasCompatiableVelocity(source.vel, target.vel)) return false;

                if (MathUtils::LengthSq(source.vel) > 0.0f &&
                    MathUtils::LengthSq(target.vel) > 0.0f)
                {
                    const float sourceAngle = MathUtils::GetVectorDirectionDegrees(source.vel);

                    const float targetAngle = MathUtils::GetVectorDirectionDegrees(target.vel);

                    const float angleDiff = std::abs(
                        MathUtils::GetShortestAngleDiff(targetAngle, sourceAngle));

                    if (angleDiff > 30.0f) return false;
                }
            }
            // If velocity information is unavailable, allow the distance gate in GenerateCandidates to decide
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

        for (std::size_t s = 0; s < sources.size(); ++s) {
            for (std::size_t t = 0; t < targets.size(); ++t) {
                const auto& source = GetTrack(sources[s]);
                const auto& target = GetTrack(targets[t]);

                // Reject pairs that fail the system-specific checks
                if (!IsCompatible(source, target, sourceKind)) continue;

                // Apply the distance gate
                const float distSq = MathUtils::GetDistanceSq(source.pos, target.pos);

                const float gateNM = GetTrackGateNM(target);

                if (!IsWithinGate(distSq, gateNM)) continue;

                candidates.push_back({distSq, s, t});
            }
        }

        return candidates;
    }
}