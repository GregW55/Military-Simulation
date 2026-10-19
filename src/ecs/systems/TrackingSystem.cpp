#include "../Systems.h"
#include "../components.h"
#include "../../core/Constants.h"
#include "../../utils/ThreatAnalysis.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
    // ============================================================
    // Track maintenance
    // ============================================================

    void UpdateTrackAges(entt::registry& registry, RadarDetectionData& detectionData, float deltaTime)
    {
        for (auto it = detectionData.activeTracks.begin();
             it != detectionData.activeTracks.end();)
        {
            if (!registry.valid(it->first))
            {
                it = detectionData.activeTracks.erase(it);
                continue;
            }

            for (auto& track : it->second)
            {
                track.ageSec += deltaTime;
            }

            std::erase_if(it->second, [](const RadarTrack& track)
                {
                    return track.ageSec > TRACK_DELETION_TIMEOUT_SEC;
                });

            ++it;
        }
    }

    // ============================================================
    // Existing track update
    // ============================================================

    void UpdateExistingTrack(
        RadarTrack& track,
        const RadarTrack& ping,
        const MathUtils::Vec2& observerPos)
    {
        if (track.ageSec > 0.0001f)
        {
            const float timeElapsed = track.ageSec;

            MathUtils::Vec2 calculatedVel = MathUtils::VelocityBetweenPositions(track.pos, ping.pos, timeElapsed);
            float derivedSpeedNmPerSec = MathUtils::Length(calculatedVel);
            float derivedSpeedKnots = MathUtils::NmPerSecToKnots(derivedSpeedNmPerSec);

            // Reject impossible jumps
            if (derivedSpeedKnots > MAX_PLAUSIBLE_SPEED_KNOTS) return;

            track.vel = calculatedVel;
            track.speedKnots = derivedSpeedKnots;
            track.hasVelocity = true;
            track.isMissing = false;
            track.pos = ping.pos;
            track.altitude = ping.altitude;
            track.ageSec = 0.0f;

            float distance = MathUtils::GetDistance(observerPos, track.pos);
            if (distance > 0.001f) {
                // Update closing speed.
                float closingSpeedNmPerSec =
                    MathUtils::GetClosingSpeedNmPerSec(track.pos, track.vel, observerPos);

                track.closingSpeedKnots = MathUtils::NmPerSecToKnots(closingSpeedNmPerSec);

                // Update time to impact.
                track.timeToImpactSec = ThreatAnalysis::EstimateTimeToImpact(distance, closingSpeedNmPerSec);;

            }
            else track.timeToImpactSec = 0.0f;

            // Update threat classification.
            ThreatClass newClassification = ThreatAnalysis::Classify(track.speedKnots, track.altitude);

            if (newClassification == track.classification) track.consistentObservationSec += timeElapsed;
            else  track.consistentObservationSec = 0.0f;

            track.classification = newClassification;

            // Update threat score.
            track.threatScore = ThreatAnalysis::ComputeThreatScore(track);
        }
    }


    // ============================================================
    // New track creation
    // ============================================================

    RadarTrack CreateNewTrack(
        const RadarTrack& ping)
    {
        RadarTrack track;

        track.pos = ping.pos;
        track.vel = {0.0f, 0.0f};
        track.altitude = ping.altitude;
        track.ageSec = 0.0f;
        track.hasVelocity = false;
        track.isMissing = false;

        track.speedKnots = 0.0f;
        track.closingSpeedKnots = 0.0f;
        track.timeToImpactSec = -1.0f;

        track.classification = ThreatClass::UNKNOWN;

        track.threatScore = 0.0f;

        track.consistentObservationSec = 0.0f;

        return track;
    }

    float TrackGateNM(const RadarTrack& track) {
        if (!track.hasVelocity) {
            return TRACK_POSITION_TOLERANCE_NM + TRACK_MAX_SPEED_NM_S * track.ageSec;
        }
        return TRACK_POSITION_TOLERANCE_NM + 0.5f * TRACK_MAX_ACCEL_NM_S2 * (track.ageSec * track.ageSec);
    }

    // ============================================
    // === Correlate all pings for one observer ===
    // ============================================

    void CorrelatePings(entt::registry& registry, entt::entity observer, const std::vector<RadarTrack>& pings)
    {
        auto& tracks = registry.ctx().get<RadarDetectionData>().activeTracks[observer];
        const MathUtils::Vec2 observerPos = registry.get<Transform2D>(observer).pos;

        struct Candidate { float distSq; size_t ping; size_t track; };
        std::vector <Candidate> candidates;

        // Every ping/track pair that falls inside that track's gate
        for (size_t p = 0; p < pings.size(); ++p) {
            for (size_t t = 0; t < tracks.size(); ++t) {
                const RadarTrack& track = tracks[t];

                MathUtils::Vec2 predictedPos = track.hasVelocity ?
                    MathUtils::ExtrapolatePosition(track.pos, track.vel, track.ageSec)
                    : track.pos;

                const float distSq = MathUtils::GetDistanceSq(pings[p].pos, predictedPos);
                const float gate = TrackGateNM(track);
                if (distSq <= gate * gate) candidates.push_back({distSq, p, t});
            }
        }

        // Closest pairs win; each ping and each track can be used once
        std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.distSq < b.distSq; });

        std::vector<char> pingUsed(pings.size(), 0);
        std::vector<char> trackUsed(tracks.size(), 0);

        for (const Candidate& c : candidates) {
            if (pingUsed[c.ping] || trackUsed[c.track]) continue;
            pingUsed[c.ping] = 1;
            trackUsed[c.track] = 1;
            tracks[c.track].isMissing = false;
            UpdateExistingTrack(tracks[c.track], pings[c.ping], observerPos);
        }

        // Save the original track count before adding new ones
        const size_t originalTrackCount = tracks.size();

        // Pings nobody claimed become new tracks
        for (auto [p, ping] : pings | std::views::enumerate) {
            if (!pingUsed[p]) {
                tracks.push_back(CreateNewTrack(pings[p]));
            }
        }

        // Handle missing tracks only for the original pre-existing tracks
        for (auto [t, track] : tracks | std::views::take(originalTrackCount) | std::views::enumerate) {
            if (!trackUsed[t]) {
                tracks[t].isMissing = true;
            }
        }
    }
}


// ================================================================
// RADAR SYSTEM
// ================================================================

void Systems::TrackingSystem(entt::registry& registry, float deltaTime)
{
    auto& detectionData = registry.ctx().get<RadarDetectionData>();

    // First age and remove old tracks.
    UpdateTrackAges(registry, detectionData, deltaTime);

    for (const auto& [observer, pings] : detectionData.framePings) {
        if (registry.valid(observer)) {
            CorrelatePings(registry, observer, pings);
        }
    }
}