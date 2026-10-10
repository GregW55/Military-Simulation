#include "../Systems.h"
#include "../Components.h"

#include "../../core/Constants.h"
#include "../../core/Physics.h"
#include "../../utils/ThreatAnalysis.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    // ============================================================
    // Radar scan timing
    // ============================================================

    bool BeginRadarScan(RadarEmitter& radar, float deltaTime, float& timeDelta) {
        radar.timeSinceLastScan += deltaTime;

        if (radar.timeSinceLastScan < radar.scanRateSec) return false;

        timeDelta = radar.timeSinceLastScan;
        radar.timeSinceLastScan = 0.0f;

        return true;
    }


    // ============================================================
    // Track maintenance
    // ============================================================

    void UpdateTrackAges(
        entt::registry& registry,
        RadarDetectionData& detectionData,
        float deltaTime)
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
    // Radar scanning
    // ============================================================

    std::vector<RadarTrack> ScanForTargets(
        entt::registry& registry,
        entt::entity observer)
    {
        auto& map = registry.ctx().get<MapProjection>();

        auto& obsTransform = registry.get<Transform2D>(observer);
        auto& obsRadar = registry.get<RadarEmitter>(observer);
        auto& obsIFF = registry.get<IFF>(observer);

        std::vector<RadarTrack> pings;

        auto targets = registry.view<Transform2D, RadarSignature, IFF>();

        for (auto target : targets)
        {
            if (observer == target) continue;

            auto& tgtTransform = targets.get<Transform2D>(target);
            auto& tgtSig = targets.get<RadarSignature>(target);
            auto& tgtIFF = targets.get<IFF>(target);

            // Ignore friendly targets.
            if (obsIFF.isHostile == tgtIFF.isHostile)
                continue;

            MathUtils::Vec2 diff = MathUtils::Sub(obsTransform.pos, tgtTransform.pos);

            float distSq = MathUtils::LengthSq(diff);

            // Outside radar range.
            if (distSq > obsRadar.rangeNmSq) continue;

            bool detected = Physics::CheckRadarDetection(
                map,
                obsTransform.pos,
                obsTransform.altitude,
                obsRadar.rangeNM,
                tgtTransform.pos,
                tgtTransform.altitude,
                tgtSig.rcsFourthRoot,
                std::sqrt(distSq)
            );

            if (!detected) continue;

            RadarTrack ping;

            ping.pos = tgtTransform.pos;
            ping.altitude = tgtTransform.altitude;

            pings.push_back(ping);
        }

        return pings;
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

    // ============================================================
    // Correlate all pings for one observer
    // ============================================================

    void CorrelatePings(entt::registry& registry,
        entt::entity observer,
        const std::vector<RadarTrack>& pings)
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

                const float distSq = MathUtils::LengthSq(MathUtils::Sub(pings[p].pos, predictedPos));
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
            UpdateExistingTrack(tracks[c.track], pings[c.ping], observerPos);
        }

        // Pings nobody claimed become new tracks
        for (size_t p = 0; p < pings.size(); ++p) {
            if (!pingUsed[p]) tracks.push_back(CreateNewTrack(pings[p]));
        }
    }
}


// ================================================================
// RADAR SYSTEM
// ================================================================

void Systems::RadarSystem(
    entt::registry& registry,
    float deltaTime)
{
    auto& detectionData = registry.ctx().get<RadarDetectionData>();

    // First age and remove old tracks.
    UpdateTrackAges(registry, detectionData, deltaTime);

    auto observers = registry.view<Transform2D, RadarEmitter, IFF>();

    for (auto observer : observers) {
        auto& radar = observers.get<RadarEmitter>(observer);

        float timeDelta = 0.0f;

        // Radar isn't ready to scan yet.
        if (!BeginRadarScan(radar, deltaTime, timeDelta)) continue;

        // Get the actual radar detections.
        std::vector<RadarTrack> pings = ScanForTargets(registry, observer);

        // Match those detections against this observer's existing tracks.
        CorrelatePings(registry, observer, pings);
    }
}