#include "../../../utils/TrackCorrelation.h"
#include "../../../utils/KalmanFilter3D.h"
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
    void UpdateTrackPredictions(
        RadarDetectionData& detectionData,
        float deltaTime)
    {
        for (auto& [observer, tracks] : detectionData.activeTracks)
        {
            for (auto& track : tracks)
            {
                if (!track.kalman.initialized)
                    continue;

                KalmanFilter3D::Predict(track.kalman, deltaTime);
                KalmanFilter3D::CopyToTrack(track);
            }
        }
    }

    RadarTrack MeasurementToObservation(
        const RadarMeasurement& measurement,
        const Transform2D& observerTransform,
        const RadarEmitter& radar)
    {
        RadarTrack observation;

        // Convert slant range and elevation into horizontal range.
        const float horizontalRangeNM =
            measurement.rangeNM * std::cos(measurement.elevationRadians);

        const float verticalRangeNM =
            measurement.rangeNM * std::sin(measurement.elevationRadians);

        // Reconstruct noisy world position from measured range and azimuth.
        observation.pos = {
            observerTransform.pos.x +
                horizontalRangeNM * std::cos(measurement.azimuthRadians),

            observerTransform.pos.y +
                horizontalRangeNM * std::sin(measurement.azimuthRadians)
        };

        // Convert vertical distance back into meters.
        observation.altitude =
            observerTransform.altitude +
            verticalRangeNM * 1852.0f;

        const float crossRangeNM = measurement.rangeNM * radar.angleErrorRadians;
        observation.measurementSigmaNM = std::sqrt(
            radar.rangeErrorNM * radar.rangeErrorNM + crossRangeNM * crossRangeNM);

        return observation;
    }

    // =============================
    // === Existing track update ===
    // =============================
    void UpdateExistingTrack(
    RadarTrack& track,
    const RadarMeasurement& measurement,
    const RadarTrack& observation,
    const Transform2D& observerTransform,
    const Kinematics& observerKinematics,
    const RadarEmitter& radar)
{
    const float timeElapsed = track.ageSec;

    // Correct the predicted six-state estimate using the original
    // polar radar measurement, not the reconstructed Cartesian ping.
    const bool corrected = KalmanFilter3D::Correct(
        track.kalman,
        measurement,
        observerTransform,
        observerKinematics,
        radar
    );

    if (!corrected)
        return;

    // The EKF is now the source of truth for position and velocity.
    KalmanFilter3D::CopyToTrack(track);

    // Keep the last actual observed position separate from the
    // filter's estimated/predicted position.
    track.latestPing = observation.pos;

    track.ageSec = 0.0f;
    track.isMissing = false;

    float closingSpeedKnots = 0.0f;
    float timeToImpact = -1.0f;

    const float distance =
        MathUtils::GetDistance(
            observerTransform.pos,
            track.pos
        );

    if (distance > 0.001f && track.hasVelocity)
    {
        const float closingSpeedNmPerSec =
            MathUtils::GetClosingSpeedNmPerSec(
                track.pos,
                track.vel,
                observerTransform.pos
            );

        closingSpeedKnots =
            MathUtils::NmPerSecToKnots(closingSpeedNmPerSec);

        timeToImpact =
            ThreatAnalysis::EstimateTimeToImpact(
                distance,
                closingSpeedNmPerSec
            );
    }

    track.closingSpeedKnots = closingSpeedKnots;
    track.timeToImpactSec = timeToImpact;
    track.threatScore =
        ThreatAnalysis::ComputeThreatScore(track);

    // Preserve your existing classification confirmation logic.
        if (track.classification == ThreatClass::UNKNOWN)
        {
            if (track.hasVelocity)
            {
                const auto& P = track.kalman.P;
                const float speedSigmaKnots =
                    std::sqrt(std::max(0.0f, P[3 * 6 + 3] + P[4 * 6 + 4])) * NM_PER_SEC_TO_KNOTS;
                const float altSigmaM =
                    std::sqrt(std::max(0.0f, P[2 * 6 + 2])) * METERS_PER_NM;

                const ThreatClass observed = ThreatAnalysis::Classify(
                    track.speedKnots, speedSigmaKnots, track.altitude, altSigmaM);

                if (observed == ThreatClass::UNKNOWN) {
                    track.pendingClass = ThreatClass::UNKNOWN;
                    track.pendingSec = 0.0f;
                }
                else if (observed == track.pendingClass) {
                    track.pendingSec += timeElapsed;
                    if (track.pendingSec >= CLASSIFICATION_CONFIRM_SEC) {
                        track.classification = observed;
                        track.kalman.processNoiseAccelPSD = ThreatAnalysis::ProcessNoisePSD(observed);
                        track.consistentObservationSec = 0.0f;
                        track.pendingSec = 0.0f;
                    }
                }
                else {
                    track.pendingClass = observed;
                    track.pendingSec = 0.0f;
                }
            }
        }
        else {
            track.consistentObservationSec += timeElapsed;
        }
}

    // ==========================
    // === New track creation ===
    // ==========================
    RadarTrack CreateNewTrack(
    const RadarMeasurement& measurement,
    const RadarTrack& observation,
    const Transform2D& observerTransform,
    const Kinematics& observerKinematics,
    const RadarEmitter& radar)
    {
        RadarTrack track;

        track.globalTrackId = NO_GLOBAL_TRACK;

        track.latestPing = observation.pos;

        track.pos = observation.pos;
        track.vel = {0.0f, 0.0f};
        track.altitude = observation.altitude;

        track.ageSec = 0.0f;
        track.hasVelocity = false;
        track.isMissing = false;

        track.speedKnots = 0.0f;
        track.closingSpeedKnots = 0.0f;
        track.timeToImpactSec = -1.0f;

        track.classification = ThreatClass::UNKNOWN;
        track.threatScore = 0.0f;
        track.consistentObservationSec = 0.0f;

        track.pendingClass = ThreatClass::UNKNOWN;
        track.pendingSec = 0.0f;

        // Initialize the EKF from the first actual radar measurement.
        KalmanFilter3D::Initialize(
            track.kalman,
            measurement,
            observerTransform,
            observerKinematics,
            radar
        );

        // Copy the initialized filter state into the public track fields.
        KalmanFilter3D::CopyToTrack(track);

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
            [](const TrackCorrelation::Candidate& a, const TrackCorrelation::Candidate& b) {
                return a.score < b.score;
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
    const std::vector<RadarMeasurement>& measurements,
    const std::vector<RadarTrack>& observations,
    const std::vector<char>& pingUsed,
    const Transform2D& observerTransform,
    const Kinematics& observerKinematics,
    const RadarEmitter& radar)
    {
        for (size_t p = 0; p < measurements.size(); ++p)
        {
            if (pingUsed[p])
                continue;

            observerTracks.push_back(
                CreateNewTrack(
                    measurements[p],
                    observations[p],
                    observerTransform,
                    observerKinematics,
                    radar
                )
            );
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
    auto& detectionData =
        registry.ctx().get<RadarDetectionData>();

    // Age tracks and remove expired tracks.
    UpdateTrackAges(
        registry,
        detectionData,
        deltaTime
    );

    // Predict every initialized local track exactly once per update.
    UpdateTrackPredictions(
        detectionData,
        deltaTime
    );

    // Process radar measurements for each observer.
    for (const auto& [observer, measurements] :
         detectionData.frameMeasurements)
    {
        if (!registry.valid(observer))
            continue;

        if (!registry.all_of<Transform2D, Kinematics, RadarEmitter>(observer))
            continue;

        auto& observerTracks =
            detectionData.activeTracks[observer];

        const auto& observerTransform =
            registry.get<Transform2D>(observer);

        const auto& observerKinematics =
            registry.get<Kinematics>(observer);

        const auto& radar =
            registry.get<RadarEmitter>(observer);

        // These reconstructed positions are for association only.
        // The EKF receives the original RadarMeasurement below.
        std::vector<RadarTrack> observations;
        observations.reserve(measurements.size());

        for (const RadarMeasurement& measurement : measurements)
        {
            observations.push_back(
                MeasurementToObservation(
                    measurement,
                    observerTransform,
                    radar
                )
            );
        }

        // Find possible measurement-to-track matches.
        auto candidates =
            TrackCorrelation::GenerateCandidates(
                observations,
                observerTracks,
                TrackCorrelation::SourceKind::PING
            );

        // Resolve one-to-one assignments.
        AssignmentResult assignment =
            ResolveAssignments(
                observerTracks,
                observations,
                candidates
            );

        // Correct matched tracks using the original radar measurements.
        for (const auto& match : assignment.matchedPairs)
        {
            UpdateExistingTrack(
                observerTracks[match.track],
                measurements[match.ping],
                observations[match.ping],
                observerTransform,
                observerKinematics,
                radar
            );
        }

        // Initialize filters for unmatched measurements.
        AddNewTracks(
            observerTracks,
            measurements,
            observations,
            assignment.pingUsed,
            observerTransform,
            observerKinematics,
            radar
        );

        // Preserve existing missing-track handling.
        ProcessMissingTracks(
            observerTracks,
            assignment.trackUsed,
            assignment.originalTrackCount
        );
    }
}