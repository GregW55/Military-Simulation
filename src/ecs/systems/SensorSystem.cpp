#include "../components.h"
#include "../Systems.h"
#include "../../core/Physics.h"

namespace
{
    // =========================
    // === Radar scan timing ===
    // =========================

    bool ShouldBeginRadarScan(RadarEmitter& radar, float deltaTime) {
        radar.timeSinceLastScan += deltaTime;

        if (radar.timeSinceLastScan < radar.scanRateSec) return false;

        radar.timeSinceLastScan = 0.0f;

        return true;
    }

    // ======================
    // === Radar scanning ===
    // ======================

    std::optional<RadarTrack> ScanForTargets(
        const MapProjection &map,
        const Transform2D &obsTransform,
        const RadarEmitter &obsRadar,
        const Transform2D &targetTransform,
        const RadarSignature &targetRadarSignature)
    {
        float distSq = MathUtils::GetDistanceSq(obsTransform.pos, targetTransform.pos);

        // Outside radar range.
        if (distSq > obsRadar.rangeNmSq) return std::nullopt;

        bool detected = Physics::CanRadarDetectTarget(
            map,
            obsTransform.pos,
            obsTransform.altitude,
            obsRadar.rangeNM,
            targetTransform.pos,
            targetTransform.altitude,
            targetRadarSignature.rcsFourthRoot,
            std::sqrt(distSq)
        );

        if (!detected) return std::nullopt;

        RadarTrack ping;
        ping.pos = targetTransform.pos;
        ping.altitude = targetTransform.altitude;

        return ping;
    }
}

void Systems::SensorSystem(entt::registry& registry, const float deltaTime) {
    const auto& map = registry.ctx().get<MapProjection>();
    auto& detectionData = registry.ctx().get<RadarDetectionData>();

    // Clear out last frame's pings
    detectionData.framePings.clear();

    const auto observerView = registry.view<Transform2D, RadarEmitter, IFF>();

    observerView.each([&] (
        const entt::entity observer,
        const Transform2D& obsTransform,
        RadarEmitter& obsRadar,
        const IFF& obsIFF)
    {
        if (ShouldBeginRadarScan(obsRadar, deltaTime)) {
            std::vector<RadarTrack> pings;
            pings.reserve(10);

            const auto obsTargetView = registry.view<Transform2D, RadarSignature, IFF>();

            obsTargetView.each([&] (
                const entt::entity target,
                const Transform2D& targetTransform,
                const RadarSignature& targetRadarSignature,
                const IFF& targetIFF)
            {
                if (target != observer && obsIFF.isHostile != targetIFF.isHostile) {
                    const auto ping = ScanForTargets(map, obsTransform, obsRadar,
                                               targetTransform, targetRadarSignature);
                    if (ping.has_value()) {
                        pings.push_back(*ping);
                    }
                }
            });

            // Store the generated pingsi n the global context for Tracking to pick up
            detectionData.framePings[observer] = std::move(pings);
        }
    });
}