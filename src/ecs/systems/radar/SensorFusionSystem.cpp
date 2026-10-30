#include "../../Systems.h"
#include "../../../utils/TrackCorrelation.h"

namespace {

    void UpdateMasterTrack(
        MasterTrackTable::GlobalTrack& globalTrack,
        const RadarTrack& localTrack)
    {
        if (localTrack.ageSec > globalTrack.track.ageSec) return;

        // isMissing is set at the top of the frame, so true here = first report this frame
        const bool firstReport = globalTrack.track.isMissing;

        globalTrack.track.latestPing = localTrack.latestPing;
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
        else {
            // Another observer already reported this frame: keep the lowest known TTI, highest threat
            float &g = globalTrack.track.timeToImpactSec;
            const float l = localTrack.timeToImpactSec;

            if (l >= 0.0f && (g < 0.0f || l < g)) g = l;

            globalTrack.track.threatScore = std::max(globalTrack.track.threatScore, localTrack.threatScore);
        }

        globalTrack.track.classification = localTrack.classification;

        globalTrack.track.consistentObservationSec = localTrack.consistentObservationSec;
        globalTrack.confidence = 1.0f;
    }

    void CreateNewMasterTrack(
        MasterTrackTable& masterTable,
        const RadarTrack& localTrack,
        uint32_t newId)
    {
        MasterTrackTable::GlobalTrack globalTrack{};
        localTrack.globalTrackId = newId;   // global track id
        globalTrack.track = localTrack;     // Initialize globalTrack with localTrack's parameters
        globalTrack.confidence = 1.0f;      // Confidence

        masterTable.activeGlobalTracks.emplace(newId, globalTrack);
    }

    uint32_t CorrelateLocalToMaster(
        const MasterTrackTable& masterTable,
        const RadarTrack& localTrack)
    {
        // Build a list of pointers so the helper can work with the unordered_map without copying the global tracks
        std::vector<const MasterTrackTable::GlobalTrack*> globalTracks;
        globalTracks.reserve(masterTable.activeGlobalTracks.size());

        for (const auto& [id, globalTrack] : masterTable.activeGlobalTracks) {
            globalTracks.push_back(&globalTrack);
        }

        // One local track is being compared against all global tracks
        const std::vector localTracks = {&localTrack};

        const auto candidates = TrackCorrelation::GenerateCandidates(
            localTracks,  // Sources
            globalTracks, // Targets
            TrackCorrelation::SourceKind::TRACK);

        if (candidates.empty()) {
            return 0;
        }

        // Choose the nearest candidate, not the first unordered_map entry.
        const auto best = std::min_element(
            candidates.begin(),
            candidates.end(),
            [](const TrackCorrelation::Candidate& a,
                const TrackCorrelation::Candidate& b) {
                    return a.distSq < b.distSq;
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

    // Age existing global tracks
    for (auto& [id, globalTrack] : masterTable.activeGlobalTracks) {
        globalTrack.track.ageSec += deltaTime;
        globalTrack.track.isMissing = true;  // Cleared by UpdateMasterTrack if a live local track reports it
    }

    for (auto& [observer, tracks] : detectionData.activeTracks) {
        for (auto& localTrack : tracks) {
            if (localTrack.isMissing) continue;

            // Preserve an existing local-to-global association
            if (localTrack.globalTrackId != 0) {
                auto it = masterTable.activeGlobalTracks.find(localTrack.globalTrackId);

                if (it != masterTable.activeGlobalTracks.end()) {
                    UpdateMasterTrack(it->second, localTrack);
                    continue;
                }
            }
            
            // No valid existing association: try correlating by position.
            const uint32_t matchedGlobalTrackId = CorrelateLocalToMaster(masterTable, localTrack);

            auto it = masterTable.activeGlobalTracks.find(matchedGlobalTrackId);

            if (it != masterTable.activeGlobalTracks.end()) {
                UpdateMasterTrack(it->second, localTrack);

                localTrack.globalTrackId = matchedGlobalTrackId;
            }
            else {
                // No match: create a new global track.
                const uint32_t newId = masterTable.AllocateTrackId();

                CreateNewMasterTrack(masterTable, localTrack, newId);

                localTrack.globalTrackId = newId;
            }
        }
    }

    // Drop global tracks that no observer has refreshed within the timeout limit
    std::erase_if(masterTable.activeGlobalTracks, [] (const auto& entry) {
        return entry.second.track.ageSec > TRACK_DELETION_TIMEOUT_SEC;
    });
}