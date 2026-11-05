#include "../../../utils/TrackCorrelation.h"
#include "../../Systems.h"
#include "../../components.h"
#include "../../../core/Constants.h"
#include "../../../utils/ThreatAnalysis.h"

#include <algorithm>
#include <vector>

namespace {

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

    // =======================================
    // === Track aging and timeout removal ===
    // =======================================
    void UpdateTrackAges(entt::registry& registry, RadarDetectionData& detectionData, float deltaTime)
    {
        for (auto it = detectionData.activeTracks.begin(); it != detectionData.activeTracks.end();){
            if (!registry.valid(it->first)) {
                it = detectionData.activeTracks.erase(it);
                continue;
            }

            for (auto& track : it->second) {
                track.ageSec += deltaTime;
            }

            std::erase_if(it->second, [](const RadarTrack& track) {
                return track.ageSec > TRACK_DELETION_TIMEOUT_SEC;
            });

            ++it;
        }
    }

    // =======================
    // === Prediction Step ===
    // =======================
    void UpdateTrackPredictions(RadarDetectionData& detectionData)
    {
        for (auto& [observer, tracks] : detectionData.activeTracks) {
            for (auto& track : tracks) {
                if (track.hasVelocity) {
                    track.pos = MathUtils::ExtrapolatePosition(track.latestPing, track.vel, track.ageSec);
                }
            }
        }
    }

    // =============================
    // === Existing track update ===
    // =============================
    void UpdateExistingTrack(
        RadarTrack& track,
        const RadarTrack& ping,
        const MathUtils::Vec2& observerPos)
    {
        const float timeElapsed = track.ageSec;

        // Calculate velocity using the anchor from the last actual ping
        const MathUtils::Vec2 calculatedVel = MathUtils::VelocityBetweenPositions(track.latestPing, ping.pos, timeElapsed);
        const float derivedSpeedNmPerSec = MathUtils::Length(calculatedVel);
        const float derivedSpeedKnots = MathUtils::NmPerSecToKnots(derivedSpeedNmPerSec);

        float closingSpeedKnots = 0.0f;
        float timeToImpact = 0.0f;

        const float distance = MathUtils::GetDistance(observerPos, track.pos);
        if (distance > 0.001f) {
            const float closingSpeedNmPerSec =
                MathUtils::GetClosingSpeedNmPerSec(track.pos, track.vel, observerPos);

            closingSpeedKnots = MathUtils::NmPerSecToKnots(closingSpeedNmPerSec);
            timeToImpact = ThreatAnalysis::EstimateTimeToImpact(distance, closingSpeedNmPerSec);
        }
        else {
            timeToImpact = 0.0f;
            closingSpeedKnots = 0.0f;
        }

        track.latestPing = ping.pos;
        track.pos = ping.pos;
        track.vel = calculatedVel;
        track.altitude = ping.altitude;
        track.ageSec = 0.0f;
        track.hasVelocity = true;
        track.isMissing = false;
        track.speedKnots = derivedSpeedKnots;
        track.closingSpeedKnots = closingSpeedKnots;
        track.timeToImpactSec = timeToImpact;
        track.threatScore = ThreatAnalysis::ComputeThreatScore(track);

        if (track.classification == ThreatClass::UNKNOWN) {         // Only classify track once
            const ThreatClass observed = ThreatAnalysis::Classify(track.speedKnots, track.altitude);

            if (observed == track.classification) {
                track.pendingSec = 0.0f;
                track.consistentObservationSec += timeElapsed;
            }
            else if (observed == track.pendingClass) {
                track.pendingSec += timeElapsed;
                if (track.pendingSec >= CLASSIFICATION_CONFIRM_SEC) {
                    track.classification = observed;
                    track.consistentObservationSec = 0.0f;
                    track.pendingSec = 0.0f;
                }
            }
            else {
                track.pendingClass = observed;
                track.pendingSec = 0.0f;
            }
        }
        else track.consistentObservationSec += timeElapsed;
    }

    // ==========================
    // === New track creation ===
    // ==========================
    RadarTrack CreateNewTrack(const RadarTrack& ping)
    {
        RadarTrack track;

        track.globalTrackId = NO_GLOBAL_TRACK;
        track.pos = ping.pos;
        track.latestPing = ping.pos;
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

    AssignmentResult ResolveAssignments(
        const std::vector<RadarTrack>& observerTracks,
        const std::vector<RadarTrack>& pings,
        std::vector <TrackCorrelation::Candidate>& candidates)
    {
        // Closest pairs win; each ping and each track can be used once
        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const TrackCorrelation::Candidate& a,
             const TrackCorrelation::Candidate& b) {
                return a.distSq < b.distSq;
             });

        std::vector<char> pingUsed(pings.size(), 0);
        std::vector<char> trackUsed(observerTracks.size(), 0);
        std::vector<MatchPair> matchedPairs;

        for (const TrackCorrelation::Candidate& c : candidates) {
            const size_t p = c.sourceIndex;
            const size_t t = c.targetIndex;

            if (pingUsed[p] || trackUsed[t]) continue;

            pingUsed[p] = 1;
            trackUsed[t] = 1;

            matchedPairs.push_back({p, t});
        }

        return {
            std::move(matchedPairs),
            std::move(pingUsed),
            std::move(trackUsed),
            observerTracks.size()
            };
    }

    void AddNewTracks(
        std::vector<RadarTrack>& observerTracks,
        const std::vector<RadarTrack>& pings,
        const std::vector<char>& pingUsed)
    {
        // Pings nobody claimed become new tracks
        for (auto [p, ping] : pings | std::views::enumerate) {
            if (!pingUsed[p]) {
                observerTracks.push_back(CreateNewTrack(ping));
            }
        }
    }

    void ProcessMissingTracks(
        std::vector<RadarTrack>& observerTracks,
        const std::vector<char>& trackUsed,
        size_t originalTrackCount)
    {
        // Handle missing tracks - only for tracks that existed on a previous frame but had no matched pair
        for (auto [t, track] : observerTracks | std::views::take(originalTrackCount) | std::views::enumerate) {
            if (!trackUsed[t]) {
                track.isMissing = true;
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

    // Advance track timers and prune dead ones globally
    UpdateTrackAges(registry, detectionData, deltaTime);

    // Pre-calculate predicted positions for this frame globally
    UpdateTrackPredictions(detectionData);

    // Process radar pings for each active observer
    for (const auto& [observer, pings] : detectionData.framePings) {
        if (registry.valid(observer)) {
            auto& observerTracks = detectionData.activeTracks[observer];
            const MathUtils::Vec2 observerPos = registry.get<Transform2D>(observer).pos;

            // Find potential matches using pre-calculated positions
            auto candidates = TrackCorrelation::GenerateCandidates(pings, observerTracks, TrackCorrelation::SourceKind::PING);

            // Resolve the assignment (closest pairs win)
            AssignmentResult assignment = ResolveAssignments(observerTracks, pings, candidates);

            // Update all confirmed matches
            for (const auto& match : assignment.matchedPairs) {
                UpdateExistingTrack(observerTracks[match.track], pings[match.ping], observerPos);
            }

            // Spawn new tracks for unclaimed pings
            AddNewTracks(observerTracks, pings, assignment.pingUsed);

            // Flag unmatched original tracks as missing
            ProcessMissingTracks(observerTracks, assignment.trackUsed, assignment.originalTrackCount);
        }
    }
}