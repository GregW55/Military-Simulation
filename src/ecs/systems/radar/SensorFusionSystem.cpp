#include "../../Systems.h"
#include "../../../utils/TrackCorrelation.h"
#include "../../../utils/KalmanFilter3D.h"

#include <algorithm>
#include <unordered_set>
#include <vector>

namespace
 {
    bool IsStillSameObject(const RadarTrack& local, const RadarTrack& global)
    {
        return !(local.classification  != ThreatClass::UNKNOWN &&
                 global.classification != ThreatClass::UNKNOWN &&
                 local.classification  != global.classification);
    }

    void UpdateMasterTrack(
        MasterTrackTable::GlobalTrack& globalTrack,
        const RadarTrack& localTrack)
    {
        if (localTrack.ageSec > globalTrack.track.ageSec) return;

        // isMissing is set at the top of the frame, so true here = first report this frame
        const bool firstReport = globalTrack.track.isMissing;

        globalTrack.track.latestPing = localTrack.latestPing;
        globalTrack.track.kalman = localTrack.kalman;
        globalTrack.track.pos = localTrack.pos;
        globalTrack.track.vel = localTrack.vel;
        globalTrack.track.altitude = localTrack.altitude;
        globalTrack.track.ageSec = localTrack.ageSec;
        globalTrack.track.hasVelocity = localTrack.hasVelocity;
        globalTrack.track.isMissing = false;
        globalTrack.track.speedKnots = localTrack.speedKnots;

        if (firstReport) {
            globalTrack.track.timeToImpactSec = localTrack.timeToImpactSec;
            globalTrack.track.threatScore = localTrack.threatScore;
        }
        else
        {
            // Another observer already reported this frame.
            // Keep the lowest known TTI and highest threat score.

            float& globalTTI =
                globalTrack.track.timeToImpactSec;

            const float localTTI =
                localTrack.timeToImpactSec;

            if (localTTI >= 0.0f &&
                (globalTTI < 0.0f || localTTI < globalTTI))
            {
                globalTTI = localTTI;
            }

            globalTrack.track.threatScore =
                std::max(
                    globalTrack.track.threatScore,
                    localTrack.threatScore
                );
        }

        globalTrack.track.consistentObservationSec = localTrack.consistentObservationSec;
        globalTrack.confidence = 1.0f;
    }

    void CreateNewMasterTrack(
        MasterTrackTable::SideTable& side,
        const RadarTrack& localTrack,
        uint32_t newId)
    {
        MasterTrackTable::GlobalTrack globalTrack{};
        globalTrack.track = localTrack;     // Initialize globalTrack with localTrack's parameters
        globalTrack.confidence = 1.0f;      // Confidence

        side.activeGlobalTracks.emplace(newId, globalTrack);
    }

    uint32_t CorrelateLocalToMaster(
        const MasterTrackTable::SideTable& side,
        const RadarTrack& localTrack,
        const std::unordered_set<uint32_t>& claimedByThisObserver)
    {
        // Build a list of pointers so the helper can work with the unordered_map without copying the global tracks
        std::vector<const MasterTrackTable::GlobalTrack*> globalTracks;
        globalTracks.reserve(side.activeGlobalTracks.size());

        for (const auto& [id, globalTrack] : side.activeGlobalTracks) {
            if (claimedByThisObserver.contains(id)) continue;
            globalTracks.push_back(&globalTrack);
        }

        // One local track is being compared against all global tracks
        const std::vector localTracks = {&localTrack};

        const auto candidates = TrackCorrelation::GenerateCandidates(
            localTracks,  // Sources
            globalTracks, // Targets
            TrackCorrelation::SourceKind::TRACK);

        if (candidates.empty()) return NO_GLOBAL_TRACK;

        // Choose the nearest candidate, not the first unordered_map entry.
        const auto best = std::min_element(
            candidates.begin(),
            candidates.end(),
            [](const TrackCorrelation::Candidate& a,
                const TrackCorrelation::Candidate& b) {
                    return a.score < b.score;
            });

        return globalTracks[best->targetIndex]->track.globalTrackId;
    }
}

void Systems::SensorFusionSystem(
    entt::registry& registry,
    float deltaTime)
{
    auto& masterTable = registry.ctx().get<MasterTrackTable>();
    auto& detectionData = registry.ctx().get<RadarDetectionData>();

    // Predict existing global tracks using their own filter state.
    for (auto& side : masterTable.sides) {
        for (auto& [id, globalTrack] : side.activeGlobalTracks)
        {
            RadarTrack& track = globalTrack.track;

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

    for (auto& [observer, tracks] : detectionData.activeTracks) {
        // Ignore destroyed observers
        if (!registry.valid(observer))
            continue;

        // Determine which side owns this observer
        const auto* observerIFF = registry.try_get<IFF>(observer);

        if (!observerIFF)
            continue;
        // Select the global track table belonging to this observer
        // IFF false = Blue
        // IFF true = Red

        auto& side = masterTable.GetSide(observerIFF->isHostile);

        std::unordered_set<uint32_t> claimed;

        for (const auto& track : tracks) {
            // Missing local tracks do not reserve a global ID
            if (track.isMissing) continue;

            if (track.globalTrackId != NO_GLOBAL_TRACK)
            {
                claimed.insert(track.globalTrackId);
            }
        }

        for (auto& localTrack : tracks) {
            if (localTrack.isMissing) continue;
            if (localTrack.classification == ThreatClass::UNKNOWN) continue;

            // Preserve an existing local-to-global association
            if (localTrack.globalTrackId != NO_GLOBAL_TRACK) {
                auto it = side.activeGlobalTracks.find(localTrack.globalTrackId);

                if (it != side.activeGlobalTracks.end()) {
                    if (IsStillSameObject(localTrack, it->second.track)) {
                        UpdateMasterTrack(it->second, localTrack);
                        continue;
                    }
                    claimed.erase(localTrack.globalTrackId);
                    localTrack.globalTrackId = NO_GLOBAL_TRACK;
                }
                else {
                    // The global track no longer exists.
                    localTrack.globalTrackId = NO_GLOBAL_TRACK;
                }
            }

            const uint32_t matchedGlobalTrackId = CorrelateLocalToMaster(side, localTrack, claimed);

            auto it = side.activeGlobalTracks.find(matchedGlobalTrackId);

            if (it != side.activeGlobalTracks.end()) {
                UpdateMasterTrack(it->second, localTrack);
                localTrack.globalTrackId = matchedGlobalTrackId;
            }
            else {
                // No match: create a new global track.
                const uint32_t newId = side.AllocateTrackId();

                localTrack.globalTrackId = newId;
                CreateNewMasterTrack(side, localTrack, newId);
            }

            claimed.insert(localTrack.globalTrackId); // This ID is now taken
        }
    }

    for (auto& side : masterTable.sides) {
        // Drop global tracks that no observer has refreshed within the timeout limit
        std::erase_if(side.activeGlobalTracks, [] (const auto& entry) {
            return entry.second.track.ageSec > TRACK_DELETION_TIMEOUT_SEC;
        });
    }
}