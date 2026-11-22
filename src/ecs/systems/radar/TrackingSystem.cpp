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

    struct AssociationCandidate {
        float score;
        size_t ping;
        size_t track;
    };

    struct AssignmentResult {
        std::vector<MatchPair> matchedPairs;
        std::vector<char> pingUsed;
        std::vector<char> trackUsed;
    };

    constexpr size_t TRACK_CONFIRM_WINDOW = 5;
    constexpr size_t TRACK_CONFIRM_HITS = 3;

    void RecordScanResult(
    RadarTrack& track,
    bool hit)
    {
        track.recentScanHits[track.scanHistoryNext] =
            hit ? 1 : 0;

        track.scanHistoryNext =
            static_cast<uint8_t>(
                (track.scanHistoryNext + 1) %
                TRACK_CONFIRM_WINDOW
            );

        if (track.scanHistoryCount < TRACK_CONFIRM_WINDOW)
            ++track.scanHistoryCount;

        if (hit)
        {
            track.missedScanCount = 0;
            track.isMissing = false;

            // A previously confirmed track resumes confirmation
            // immediately after a successful reacquisition.
            if (track.lifecycle == TrackLifecycle::COASTING)
            {
                track.lifecycle = TrackLifecycle::CONFIRMED;
                return;
            }

            size_t hitCount = 0;

            for (size_t i = 0; i < track.scanHistoryCount; ++i)
            {
                if (track.recentScanHits[i])
                    ++hitCount;
            }

            if (track.lifecycle == TrackLifecycle::TENTATIVE &&
                hitCount >= TRACK_CONFIRM_HITS)
            {
                track.lifecycle = TrackLifecycle::CONFIRMED;
            }
        }
        else
        {
            track.isMissing = true;

            if (track.missedScanCount <
                std::numeric_limits<uint8_t>::max())
            {
                ++track.missedScanCount;
            }

            if (track.lifecycle == TrackLifecycle::CONFIRMED)
                track.lifecycle = TrackLifecycle::COASTING;
        }
    }

    std::vector<int> SolveHungarian(
        const std::vector<std::vector<double>>& cost)
    {
        const size_t n = cost.size();

        if (n == 0)
            return {};

        for (const auto& row : cost)
        {
            if (row.size() != n)
                return {};
        }

        std::vector u(n + 1, 0.0);
        std::vector v(n + 1, 0.0);

        std::vector<size_t> p(n + 1, 0);
        std::vector<size_t> way(n + 1, 0);

        for (size_t i = 1; i <= n; ++i)
        {
            p[0] = i;

            size_t j0 = 0;

            std::vector<double> minv(
                n + 1,
                std::numeric_limits<double>::infinity()
            );

            std::vector<char> used(n + 1, false);

            do
            {
                used[j0] = true;

                const size_t i0 = p[j0];

                double delta =
                    std::numeric_limits<double>::infinity();

                size_t j1 = 0;

                for (size_t j = 1; j <= n; ++j)
                {
                    if (used[j])
                        continue;

                    const double current =
                        cost[i0 - 1][j - 1] -
                        u[i0] -
                        v[j];

                    if (current < minv[j])
                    {
                        minv[j] = current;
                        way[j] = j0;
                    }

                    if (minv[j] < delta)
                    {
                        delta = minv[j];
                        j1 = j;
                    }
                }

                if (!std::isfinite(delta))
                    return {};

                for (size_t j = 0; j <= n; ++j)
                {
                    if (used[j])
                    {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    }
                    else
                    {
                        minv[j] -= delta;
                    }
                }

                j0 = j1;

            } while (p[j0] != 0);

            // Augment the assignment.
            do
            {
                const size_t j1 = way[j0];

                p[j0] = p[j1];

                j0 = j1;

            } while (j0 != 0);
        }

        // Result: row -> assigned column.
        std::vector<int> assignment(n, -1);

        for (size_t j = 1; j <= n; ++j)
        {
            if (p[j] != 0)
            {
                assignment[p[j] - 1] =
                    static_cast<int>(j - 1);
            }
        }

        return assignment;
    }

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
        const Transform2D& observerTransform)
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


        return observation;
    }

    // =============================
    // === Existing track update ===
    // =============================
    bool UpdateExistingTrack(
        RadarTrack& track,
        const RadarMeasurement& measurement,
        const RadarTrack& observation,
        const Transform2D& observerTransform,
        const Kinematics& observerKinematics)
    {
        // Elapsed time since the last successful measurement.
        const float timeElapsed = track.ageSec;

        // Correct must use exactly the same innovation calculation that was used by the association gate.
        if (!KalmanFilter3D::Correct(
            track.kalman,
            measurement,
            observerTransform,
            observerKinematics))
        {
            return false;
        }

        // Copy the corrected six-state estimate.
        KalmanFilter3D::CopyToTrack(track);

        track.latestPing = observation.pos;

        track.ageSec = 0.0f;
        track.isMissing = false;

        // --------------------------------------------------------
        // Recalculate observer-relative threat information.
        // --------------------------------------------------------

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
                MathUtils::NmPerSecToKnots( closingSpeedNmPerSec );

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

        // --------------------------------------------------------
        // Classification confirmation.
        // This remains separate from track confirmation.
        // --------------------------------------------------------

        if (track.classification == ThreatClass::UNKNOWN &&
            track.hasVelocity)
        {
            const auto& P = track.kalman.P;

            const float speedSigmaKnots =
                std::sqrt(
                    std::max(
                        0.0f,
                        P[3 * 6 + 3] + P[4 * 6 + 4]
                    )
                ) * NM_PER_SEC_TO_KNOTS;

            const float altitudeSigmaMeters =
                std::sqrt(
                    std::max(0.0f, P[2 * 6 + 2])
                ) * METERS_PER_NM;

            const ThreatClass observed =
                ThreatAnalysis::Classify(
                    track.speedKnots,
                    speedSigmaKnots,
                    track.altitude,
                    altitudeSigmaMeters
                );

            if (observed == ThreatClass::UNKNOWN)
            {
                track.pendingClass = ThreatClass::UNKNOWN;
                track.pendingSec = 0.0f;
            }
            else if (observed == track.pendingClass)
            {
                track.pendingSec += timeElapsed;

                if (track.pendingSec >= CLASSIFICATION_CONFIRM_SEC)
                {
                    track.classification = observed;

                    track.kalman.processNoiseAccelPSD =
                        ThreatAnalysis::ProcessNoisePSD(observed);

                    track.consistentObservationSec = 0.0f;
                    track.pendingSec = 0.0f;
                }
            }
            else
            {
                track.pendingClass = observed;
                track.pendingSec = 0.0f;
            }
        }
        else if (track.classification != ThreatClass::UNKNOWN)
        {
            track.consistentObservationSec += timeElapsed;
        }

        return true;
    }

    // ==========================
    // === New track creation ===
    // ==========================
    RadarTrack CreateNewTrack(
     const RadarMeasurement& measurement,
     const RadarTrack& observation,
     const Transform2D& observerTransform,
     const Kinematics& observerKinematics)
    {
        RadarTrack track{};

        track.globalTrackId = NO_GLOBAL_TRACK;

        track.latestPing = observation.pos;

        track.pos = observation.pos;
        track.altitude = observation.altitude;

        track.vel = {0.0f, 0.0f};
        track.verticalVelocityNMPerSec = 0.0f;

        track.ageSec = 0.0f;
        track.isMissing = false;
        track.hasVelocity = false;

        track.speedKnots = 0.0f;
        track.closingSpeedKnots = 0.0f;
        track.timeToImpactSec = -1.0f;
        track.threatScore = 0.0f;

        track.lifecycle = TrackLifecycle::TENTATIVE;

        track.classification = ThreatClass::UNKNOWN;
        track.pendingClass = ThreatClass::UNKNOWN;

        track.consistentObservationSec = 0.0f;
        track.pendingSec = 0.0f;

        KalmanFilter3D::Initialize(
            track.kalman,
            measurement,
            observerTransform,
            observerKinematics
        );

        KalmanFilter3D::CopyToTrack(track);

        // The measurement that created this track counts as its first hit.
        RecordScanResult(track, true);

        return track;
    }

    AssignmentResult ResolveAssignments(
        size_t trackCount,
        size_t pingCount,
        const std::vector<AssociationCandidate>& candidates)
    {
        AssignmentResult result{};

        result.pingUsed.assign(pingCount, false);
        result.trackUsed.assign(trackCount, false);

        if (trackCount == 0 || pingCount == 0)
            return result;

        // We add dummy rows and columns to allow both tracks
        // and measurements to remain unmatched.

        const size_t n = trackCount + pingCount;

        constexpr double INVALID_COST = 1e9;

        const double unassignedCost =
            RADAR_ASSOCIATION_GATE_4DOF * 0.5;

        std::vector<std::vector<double>> cost(
            n,
            std::vector<double>(n, INVALID_COST)
        );

        // --------------------------------------------------------
        // Real track -> real measurement costs.
        // --------------------------------------------------------

        for (const auto& candidate : candidates)
        {
            if (candidate.track >= trackCount ||
                candidate.ping >= pingCount)
            {
                continue;
            }

            cost[candidate.track][candidate.ping] =
                candidate.score;
        }

        // --------------------------------------------------------
        // Real track -> dummy measurement.
        // Means the track was not updated this scan.
        // --------------------------------------------------------

        for (size_t t = 0; t < trackCount; ++t)
        {
            for (size_t dummy = 0; dummy < trackCount; ++dummy)
            {
                cost[t][pingCount + dummy] =
                    unassignedCost;
            }
        }

        // --------------------------------------------------------
        // Dummy track -> real measurement.
        // Means the measurement starts a new track.
        // --------------------------------------------------------

        for (size_t dummy = 0; dummy < pingCount; ++dummy)
        {
            const size_t row = trackCount + dummy;

            for (size_t p = 0; p < pingCount; ++p)
            {
                cost[row][p] = unassignedCost;
            }
        }

        // --------------------------------------------------------
        // Dummy track -> dummy measurement.
        // These assignments have zero cost.
        // --------------------------------------------------------

        for (size_t row = trackCount; row < n; ++row)
        {
            for (size_t col = pingCount; col < n; ++col)
            {
                cost[row][col] = 0.0;
            }
        }

        const std::vector<int> assignment =
            SolveHungarian(cost);

        if (assignment.size() != n)
            return result;

        // --------------------------------------------------------
        // Extract real track / measurement matches.
        // --------------------------------------------------------

        for (size_t t = 0; t < trackCount; ++t)
        {
            const int assignedColumn = assignment[t];

            if (assignedColumn < 0)
                continue;

            const size_t p =
                static_cast<size_t>(assignedColumn);

            if (p >= pingCount)
                continue;

            if (cost[t][p] >
                RADAR_ASSOCIATION_GATE_4DOF)
            {
                continue;
            }

            result.matchedPairs.push_back({p, t});

            result.pingUsed[p] = true;
            result.trackUsed[t] = true;
        }

        return result;
    }
}

// ================================================================
// RADAR SYSTEM
// ================================================================

void Systems::TrackingSystem(
    entt::registry& registry,
    float deltaTime)
{
    auto& detectionData =
        registry.ctx().get<RadarDetectionData>();

    // --------------------------------------------------------
    // 1. Age existing tracks.
    // --------------------------------------------------------

    UpdateTrackAges(
        registry,
        detectionData,
        deltaTime
    );

    // --------------------------------------------------------
    // 2. Predict every existing EKF to the current simulation time.
    // --------------------------------------------------------

    UpdateTrackPredictions(
        detectionData,
        deltaTime
    );

    // --------------------------------------------------------
    // 3. Process each observer's completed radar scan.
    // --------------------------------------------------------

    for (const auto& [observer, measurements] :
         detectionData.frameMeasurements)
    {
        if (!registry.valid(observer))
            continue;

        if (!registry.all_of<
            Transform2D,
            Kinematics,
            RadarEmitter>(observer))
        {
            continue;
        }

        auto& observerTracks =
            detectionData.activeTracks[observer];

        const auto& observerTransform =
            registry.get<Transform2D>(observer);

        const auto& observerKinematics =
            registry.get<Kinematics>(observer);

        const size_t originalTrackCount =
            observerTracks.size();

        // ----------------------------------------------------
        // 4. Generate Cartesian positions for logging/display.
        //    They are NOT used to decide associations.
        // ----------------------------------------------------

        std::vector<RadarTrack> observations;
        observations.reserve(measurements.size());

        for (const auto& measurement : measurements)
        {
            observations.push_back(
                MeasurementToObservation(
                    measurement,
                    observerTransform
                )
            );
        }

        // ----------------------------------------------------
        // 5. Generate 4D measurement-to-track candidates.
        // ----------------------------------------------------

        std::vector<AssociationCandidate> candidates;

        for (size_t p = 0; p < measurements.size(); ++p)
        {
            const RadarMeasurement& measurement =
                measurements[p];

            for (size_t t = 0; t < originalTrackCount; ++t)
            {
                RadarTrack& track = observerTracks[t];

                if (!track.kalman.initialized)
                    continue;

                RadarInnovation innovation{};

                if (!KalmanFilter3D::CalculateInnovation(
                    track.kalman,
                    measurement,
                    observerTransform,
                    observerKinematics,
                    innovation))
                {
                    continue;
                }

                if (innovation.mahalanobisSquared >
                    RADAR_ASSOCIATION_GATE_4DOF)
                {
                    continue;
                }

                candidates.push_back({
                    innovation.mahalanobisSquared,
                    p,
                    t
                });
            }
        }

        // ----------------------------------------------------
        // 6. Find the globally optimal one-to-one assignments.
        // ----------------------------------------------------

        AssignmentResult assignment =
            ResolveAssignments(
                originalTrackCount,
                measurements.size(),
                candidates
            );

        // ----------------------------------------------------
        // 7. Correct successfully associated tracks.
        // ----------------------------------------------------

        for (const auto& match : assignment.matchedPairs)
        {
            const bool corrected =
                UpdateExistingTrack(
                    observerTracks[match.track],
                    measurements[match.ping],
                    observations[match.ping],
                    observerTransform,
                    observerKinematics
                );

            if (!corrected)
            {
                // A failed correction must not consume
                // either the ping or the existing track.
                assignment.pingUsed[match.ping] = false;
                assignment.trackUsed[match.track] = false;

                continue;
            }

            // Record one successful detection for this scan.
            RecordScanResult(
                observerTracks[match.track],
                true
            );
        }

        // ----------------------------------------------------
        // 8. Record missed detections for unmatched old tracks.
        // ----------------------------------------------------

        for (size_t t = 0; t < originalTrackCount; ++t)
        {
            if (assignment.trackUsed[t])
                continue;

            RecordScanResult(
                observerTracks[t],
                false
            );
        }

        // ----------------------------------------------------
        // 9. Start tentative tracks from unmatched measurements.
        // ----------------------------------------------------

        for (size_t p = 0; p < measurements.size(); ++p)
        {
            if (assignment.pingUsed[p])
                continue;

            RadarTrack newTrack =
                CreateNewTrack(
                    measurements[p],
                    observations[p],
                    observerTransform,
                    observerKinematics
                );

            observerTracks.push_back(
                std::move(newTrack)
            );
        }
    }
}