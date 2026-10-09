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
    // Track correlation
    // ============================================================

    int FindBestTrackMatch(
        const std::vector<RadarTrack>& tracks,
        const RadarTrack& ping,
        float timeDelta)
    {
        int bestMatchIndex = -1;

        float bestMatchDistSq = std::numeric_limits<float>::max();

        for (size_t i = 0; i < tracks.size(); ++i)
        {
            RadarTrack& track = const_cast<RadarTrack&>(tracks[i]);

            float gateNm;

            if (!track.hasVelocity) gateNm = TRACK_ACQUISITION_GATE_NM;
            else
            {
                float speedNMPerSec = MathUtils::Length(track.vel);

                gateNm = TRACK_MIN_CORRELATION_GATE_NM + speedNMPerSec * timeDelta;

                gateNm = std::clamp(gateNm, TRACK_MIN_CORRELATION_GATE_NM, TRACK_MAX_CORRELATION_GATE_NM);
            }

            MathUtils::Vec2 predictedPos = track.pos;

            if (track.hasVelocity) predictedPos = MathUtils::ExtrapolatePosition(track.pos, track.vel, track.ageSec);

            float distSq = MathUtils::LengthSq(MathUtils::Sub(ping.pos, predictedPos));

            if (distSq <= gateNm * gateNm && distSq < bestMatchDistSq)
            {
                bestMatchDistSq = distSq;
                bestMatchIndex = static_cast<int>(i);
            }
        }

        return bestMatchIndex;
    }


    // ============================================================
    // Existing track update
    // ============================================================

    void UpdateExistingTrack(
        RadarTrack& track,
        const RadarTrack& ping,
        const MathUtils::Vec2& observerPos,
        float deltaTime)
    {
        constexpr float MAX_PLAUSIBLE_SPEED_KNOTS = 4000.0f;

        if (track.ageSec > 0.0001f)
        {
            MathUtils::Vec2 calculatedVel = MathUtils::VelocityBetweenPositions(track.pos, ping.pos, track.ageSec);
            float derivedSpeedKnots = MathUtils::NmPerSecToKnots(MathUtils::Length(calculatedVel));

            // Reject impossible jumps
            if (derivedSpeedKnots <= MAX_PLAUSIBLE_SPEED_KNOTS)
            {
                track.vel = calculatedVel;
                track.speedKnots = derivedSpeedKnots;
                track.hasVelocity = true;
            }
        }

        // Update the actual track position.
        track.pos = ping.pos;
        track.altitude = ping.altitude;
        track.ageSec = 0.0f;

        // Update closing speed.
        if (MathUtils::GetDistance(observerPos, ping.pos) > 0.001f) {
            track.closingSpeedKnots = MathUtils::NmPerSecToKnots(
                MathUtils::GetClosingSpeedNmPerSec(ping.pos, track.vel, observerPos));
        }

        // Update time to impact.
        track.timeToImpactSec = ThreatAnalysis::EstimateTimeToImpact(observerPos, ping.pos, track.vel);

        // Update threat classification.
        ThreatClass newClassification = ThreatAnalysis::Classify(track.speedKnots, ping.altitude);

        if (newClassification == track.classification) track.consistentObservationSec += deltaTime;
        else  track.consistentObservationSec = 0.0f;

        track.classification = newClassification;

        // Update threat score.
        track.threatScore = ThreatAnalysis::ComputeThreatScore(track);
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


    // ============================================================
    // Correlate all pings for one observer
    // ============================================================

    void CorrelatePings(
        entt::registry& registry,
        entt::entity observer,
        const std::vector<RadarTrack>& pings,
        float timeDelta,
        float deltaTime)
    {
        auto& detectionData = registry.ctx().get<RadarDetectionData>();

        auto& obsTransform = registry.get<Transform2D>(observer);

        auto& tracks = detectionData.activeTracks[observer];

        std::vector<RadarTrack> newlyDiscoveredTracks;

        newlyDiscoveredTracks.reserve( pings.size());

        for (const auto& ping : pings)
        {
            int bestMatchIndex = FindBestTrackMatch( tracks, ping, timeDelta);

            if (bestMatchIndex != -1)
            {
                UpdateExistingTrack(
                    tracks[bestMatchIndex],
                    ping,
                    obsTransform.pos,
                    deltaTime
                );
            }
            else newlyDiscoveredTracks.push_back(CreateNewTrack(ping));
        }

        tracks.insert(tracks.end(), newlyDiscoveredTracks.begin(), newlyDiscoveredTracks.end());
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
        CorrelatePings(registry, observer, pings, timeDelta, deltaTime);
    }
}