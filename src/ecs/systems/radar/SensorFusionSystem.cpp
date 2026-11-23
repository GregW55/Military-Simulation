#include "../../Systems.h"

#include "../../../utils/TrackFusion.h"
#include "../../../utils/KalmanFilter3D.h"

#include <algorithm>
#include <unordered_set>
#include <vector>

namespace
{
    // ============================================================
    // CLASSIFICATION COMPATIBILITY
    // ============================================================

    bool IsClassificationCompatible(
        const RadarTrack& local,
        const RadarTrack& global)
    {
        // UNKNOWN is not a conflict.
        if (local.classification == ThreatClass::UNKNOWN ||
            global.classification == ThreatClass::UNKNOWN)
        {
            return true;
        }

        return local.classification == global.classification;
    }


    // ============================================================
    // TRACK-TO-TRACK ASSOCIATION
    // ============================================================

    uint32_t CorrelateLocalToMaster(
        const MasterTrackTable::SideTable& side,
        const RadarTrack& localTrack,
        const std::unordered_set<uint32_t>& claimedByThisObserver)
    {
        float bestScore =
            std::numeric_limits<float>::infinity();

        uint32_t bestGlobalTrackId =
            NO_GLOBAL_TRACK;

        for (const auto& [id, globalTrack] :
             side.activeGlobalTracks)
        {
            // One observer cannot assign two separate local
            // tracks to the same global track.
            if (claimedByThisObserver.contains(id))
                continue;

            const RadarTrack& global =
                globalTrack.track;

            if (!IsClassificationCompatible(
                localTrack,
                global))
            {
                continue;
            }

            float score = 0.0f;

            if (!TrackFusion::CalculateTrackDistance(
                localTrack,
                global,
                score))
            {
                continue;
            }

            const float gate =
                (localTrack.hasVelocity && global.hasVelocity)
                    ? GLOBAL_ASSOCIATION_GATE_6DOF
                    : GLOBAL_ASSOCIATION_GATE_3DOF;

            if (score > gate)
                continue;

            if (score < bestScore)
            {
                bestScore = score;
                bestGlobalTrackId = id;
            }
        }

        return bestGlobalTrackId;
    }


    // ============================================================
    // GLOBAL TRACK CREATION
    // ============================================================

    void CreateNewMasterTrack(
        MasterTrackTable::SideTable& side,
        entt::entity observer,
        RadarTrack& localTrack,
        uint32_t newId)
    {
        MasterTrackTable::GlobalTrack globalTrack{};

        globalTrack.track = localTrack;

        // A global track's ID is side-local.
        globalTrack.track.globalTrackId = newId;

        globalTrack.confidence = 1.0f;

        // Remember which measurement initialized the global state.
        globalTrack.lastFusedMeasurementTime[observer] =
            localTrack.lastMeasurementTimeSec;

        side.activeGlobalTracks.emplace(
            newId,
            std::move(globalTrack)
        );

        localTrack.globalTrackId = newId;
    }


    // ============================================================
    // UPDATE AN EXISTING GLOBAL TRACK
    // ============================================================

    bool UpdateMasterTrack(
        MasterTrackTable::GlobalTrack& globalTrack,
        entt::entity observer,
        const RadarTrack& localTrack)
    {
        // Do not fuse an old or unchanged local measurement again.
        auto previousMeasurement =
            globalTrack.lastFusedMeasurementTime.find(observer);

        if (previousMeasurement !=
            globalTrack.lastFusedMeasurementTime.end())
        {
            constexpr float TIME_EPSILON = 0.0001f;

            if (localTrack.lastMeasurementTimeSec <=
                previousMeasurement->second + TIME_EPSILON)
            {
                return false;
            }
        }

        // Both filters should already be predicted to the same
        // current simulation timestamp.
        if (!globalTrack.track.kalman.initialized ||
            !localTrack.kalman.initialized)
        {
            return false;
        }

        KalmanState3D fusedState{};

        if (!TrackFusion::CovarianceIntersection(
            globalTrack.track.kalman,
            localTrack.kalman,
            fusedState))
        {
            return false;
        }

        // Commit the fused state.
        globalTrack.track.kalman = fusedState;

        KalmanFilter3D::CopyToTrack(
            globalTrack.track
        );

        // Preserve the global identity.
        // The local track and global track share the same ID.
        globalTrack.track.globalTrackId =
            localTrack.globalTrackId;

        globalTrack.track.latestPing =
            localTrack.latestPing;

        globalTrack.track.ageSec = 0.0f;
        globalTrack.track.isMissing = false;

        globalTrack.track.lastMeasurementTimeSec =
            std::max(
                globalTrack.track.lastMeasurementTimeSec,
                localTrack.lastMeasurementTimeSec
            );

        // Adopt classification when the global track does not
        // yet have one. Conflicts should have been rejected
        // by the association check.
        if (globalTrack.track.classification ==
                ThreatClass::UNKNOWN &&
            localTrack.classification !=
                ThreatClass::UNKNOWN)
        {
            globalTrack.track.classification =
                localTrack.classification;
        }

        globalTrack.track.lifecycle =
            TrackLifecycle::CONFIRMED;

        globalTrack.track.consistentObservationSec =
            std::max(
                globalTrack.track.consistentObservationSec,
                localTrack.consistentObservationSec
            );

        globalTrack.confidence = 1.0f;

        // This observer's latest measurement has now been fused.
        globalTrack.lastFusedMeasurementTime[observer] =
            localTrack.lastMeasurementTimeSec;

        return true;
    }
}


// ================================================================
// SENSOR FUSION SYSTEM
// ================================================================

void Systems::SensorFusionSystem(
    entt::registry& registry,
    float deltaTime)
{
    auto& masterTable =
        registry.ctx().get<MasterTrackTable>();

    auto& detectionData =
        registry.ctx().get<RadarDetectionData>();


    // ============================================================
    // 1. PREDICT BOTH SIDES' GLOBAL TRACKS
    // ============================================================

    for (auto& side : masterTable.sides)
    {
        for (auto& [id, globalTrack] :
             side.activeGlobalTracks)
        {
            RadarTrack& track =
                globalTrack.track;

            track.ageSec += deltaTime;
            track.isMissing = true;

            if (track.kalman.initialized)
            {
                KalmanFilter3D::Predict(
                    track.kalman,
                    deltaTime
                );

                KalmanFilter3D::CopyToTrack(track);
            }
        }
    }


    // ============================================================
    // 2. PROCESS EACH OBSERVER
    // ============================================================

    for (auto& [observer, localTracks] :
         detectionData.activeTracks)
    {
        if (!registry.valid(observer))
            continue;

        const auto* observerIFF =
            registry.try_get<IFF>(observer);

        if (!observerIFF)
            continue;

        // Select the global database belonging to the
        // observer's own side.
        auto& side =
            masterTable.GetSide(observerIFF->isHostile);


        // ========================================================
        // 3. COLLECT GLOBAL IDS ALREADY HELD BY THIS OBSERVER
        // ========================================================

        std::unordered_set<uint32_t> claimed;

        for (const auto& localTrack : localTracks)
        {
            if (localTrack.isMissing)
                continue;

            if (localTrack.lifecycle ==
                TrackLifecycle::TENTATIVE)
            {
                continue;
            }

            if (localTrack.globalTrackId != NO_GLOBAL_TRACK)
            {
                claimed.insert(
                    localTrack.globalTrackId
                );
            }
        }


        // ========================================================
        // 4. PROCESS CONFIRMED LOCAL TRACKS
        // ========================================================

        for (auto& localTrack : localTracks)
        {
            // A local track must have been detected during
            // a completed scan to contribute a new measurement.
            if (localTrack.isMissing)
                continue;

            // Do not publish tentative tracks yet.
            // UNKNOWN classification is allowed.
            if (localTrack.lifecycle !=
                TrackLifecycle::CONFIRMED)
            {
                continue;
            }


            // ----------------------------------------------------
            // A. CHECK EXISTING GLOBAL ID
            // ----------------------------------------------------

            if (localTrack.globalTrackId !=
                NO_GLOBAL_TRACK)
            {
                auto existing =
                    side.activeGlobalTracks.find(
                        localTrack.globalTrackId
                    );

                if (existing !=
                    side.activeGlobalTracks.end())
                {
                    if (IsClassificationCompatible(
                        localTrack,
                        existing->second.track))
                    {
                        // The local-to-global identity is already
                        // established. Fuse only a new measurement.
                        UpdateMasterTrack(
                            existing->second,
                            observer,
                            localTrack
                        );

                        continue;
                    }

                    // A confirmed classification conflict invalidates
                    // this local-to-global association.
                    claimed.erase(
                        localTrack.globalTrackId
                    );

                    localTrack.globalTrackId =
                        NO_GLOBAL_TRACK;
                }
                else
                {
                    // The old global ID has expired.
                    localTrack.globalTrackId =
                        NO_GLOBAL_TRACK;
                }
            }


            // ----------------------------------------------------
            // B. CORRELATE AGAINST THIS SIDE'S GLOBAL TRACKS
            // ----------------------------------------------------

            const uint32_t matchedGlobalTrackId =
                CorrelateLocalToMaster(
                    side,
                    localTrack,
                    claimed
                );


            // ----------------------------------------------------
            // C. UPDATE AN EXISTING GLOBAL TRACK
            // ----------------------------------------------------

            if (matchedGlobalTrackId !=
                NO_GLOBAL_TRACK)
            {
                auto existing =
                    side.activeGlobalTracks.find(
                        matchedGlobalTrackId
                    );

                if (existing !=
                    side.activeGlobalTracks.end())
                {
                    localTrack.globalTrackId =
                        matchedGlobalTrackId;

                    const bool updated =
                        UpdateMasterTrack(
                            existing->second,
                            observer,
                            localTrack
                        );

                    if (updated)
                    {
                        claimed.insert(
                            matchedGlobalTrackId
                        );

                        continue;
                    }

                    // If fusion failed, do not destroy an existing
                    // global track. Leave the local track unassigned
                    // so it can be reconsidered on a later scan.
                    localTrack.globalTrackId =
                        NO_GLOBAL_TRACK;

                    continue;
                }
            }


            // ----------------------------------------------------
            // D. CREATE A NEW GLOBAL TRACK
            // ----------------------------------------------------

            const uint32_t newId =
                side.AllocateTrackId();

            CreateNewMasterTrack(
                side,
                observer,
                localTrack,
                newId
            );

            claimed.insert(newId);
        }
    }


    // ============================================================
    // 5. REMOVE EXPIRED GLOBAL TRACKS FROM BOTH SIDES
    // ============================================================

    for (auto& side : masterTable.sides)
    {
        std::erase_if(
            side.activeGlobalTracks,
            [](const auto& entry)
            {
                return entry.second.track.ageSec >
                    TRACK_DELETION_TIMEOUT_SEC;
            }
        );
    }
}