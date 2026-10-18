#include "../Systems.h"

namespace {
    // Correlation Step: Returns the System Track ID if found, or 0 if no match
    uint32_t CorrelateLocalToMaster(const MasterTrackTable& masterTable, const RadarTrack& localTrack) {
        for (const auto& [sysId, sysTrack] : masterTable.activeSystemTracks) {
            if (MathUtils::GetDistance(localTrack.pos, sysTrack.fusedPos) < 1.5f) {
                return sysId;
            }
        }
        return 0; // No match found
    }

    // Mutation Step: Updates an existing master track with new sensor data
    void UpdateMasterTrack(MasterTrackTable& masterTable, uint32_t systemId, const RadarTrack& localTrack) {
        auto it = masterTable.activeSystemTracks.find(systemId);
        if (it != masterTable.activeSystemTracks.end()) {
            auto& sysTrack = it->second;
            sysTrack.fusedPos = localTrack.pos;
            sysTrack.fusedVel = localTrack.vel;
            sysTrack.classification = localTrack.classification;
            sysTrack.ageSec = 0.0f; // Reset age on update
        }
    }

    void CreateNewMasterTrack(MasterTrackTable& masterTable, const RadarTrack& localTrack) {
        uint32_t newId = masterTable.AllocateTrackId();
        masterTable.activeSystemTracks[newId] = {
            newId,
            localTrack.pos,
            localTrack.vel,
            localTrack.altitude,
            localTrack.classification,
            1.0f,
            0.0f
        };
    }
}

void SensorFusionSystem(entt::registry& registry, float deltaTime) {
    auto& masterTable = registry.ctx().get<MasterTrackTable>();
    auto& detectionData = registry.ctx().get<RadarDetectionData>();

    // Age out existing tracks
    for (auto& [id, track] : masterTable.activeSystemTracks) {
        track.ageSec += deltaTime;
    }

    // Correlate and update/create local tracks to master track table
    for (const auto& [observerEntity, tracks] : detectionData.activeTracks) {
        for (const auto& localTrack : tracks) {
            // Check if this local track matches an existing System Track within a correlation window
            uint32_t matchedSystemId = CorrelateLocalToMaster(masterTable, localTrack);

            if (matchedSystemId != 0) {
                // Update existing Master Track with fused data
                UpdateMasterTrack(masterTable, matchedSystemId, localTrack);

            } else {
                // Allocate a brand new globally unique System Track ID
                CreateNewMasterTrack(masterTable, localTrack);
            }

        }
    }
}