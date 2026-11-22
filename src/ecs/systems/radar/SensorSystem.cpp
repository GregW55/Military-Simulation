#include "../../components.h"
#include "../../Systems.h"

#include "../../../core/Physics.h"
#include "../../../core/Constants.h"

#include <cmath>
#include <optional>
#include <random>

namespace
{
    // ======================
    // === Radar scanning ===
    // ======================
    // Generate a noisy radar measurement for one detected target.
    std::optional<RadarMeasurement> ScanForTargets(
        const MapProjection &map,
        const Transform2D &obsTransform,
        const Kinematics& obsKinematics,
        const RadarEmitter &obsRadar,
        const Transform2D &targetTransform,
        const Kinematics& targetKinematics,
        const RadarSignature &targetRadarSignature,
        float timestampSec,
        std::mt19937& rng)
    {
        // ------------------------------------------------
        //  Calculate relative horizontal position
        // ------------------------------------------------

        const MathUtils::Vec2 relativePos {
            targetTransform.pos.x - obsTransform.pos.x,
            targetTransform.pos.y - obsTransform.pos.y
        };

        const float horizontalRangeSq =
            relativePos.x * relativePos.x +
            relativePos.y * relativePos.y;

        // Fast horizontal range check.
        const float effectiveRangeNM = obsRadar.maxRangeNM * targetRadarSignature.rcsFourthRoot;
        if (horizontalRangeSq > effectiveRangeNM * effectiveRangeNM)
            return std::nullopt;

        const float horizontalRangeNM = std::sqrt(horizontalRangeSq);

         // ------------------------------------------------
        // Check whether the target can be detected
        // ------------------------------------------------

        const bool detected = Physics::CanRadarDetectTarget(
            map,
            obsTransform.pos,
            obsTransform.altitude,
            obsRadar.maxRangeNM,
            targetTransform.pos,
            targetTransform.altitude,
            targetRadarSignature.rcsFourthRoot,
            horizontalRangeNM
        );

        if (!detected) return std::nullopt;

        // ------------------------------------------------
        // Calculate true slant range
        // ------------------------------------------------

        const float altitudeDifferenceNM = (targetTransform.altitude - obsTransform.altitude) / METERS_PER_NM;

        const float slantRangeNM = std::sqrt(
            horizontalRangeSq +
            altitudeDifferenceNM * altitudeDifferenceNM
        );

        // ------------------------------------------------
        // Calculate true azimuth and elevation
        // ------------------------------------------------

        const float trueAzimuthRadians = std::atan2(relativePos.y, relativePos.x);

        const float trueElevationRadians = std::atan2(altitudeDifferenceNM, horizontalRangeNM);

        // ------------------------------------------------
        // Calculate true 3D radial velocity
        // ------------------------------------------------

        float trueRadialVelocityKnots = 0.0f;

        if (slantRangeNM > 0.0001f)
        {
            const float horizontalUnitX =
                relativePos.x / slantRangeNM;

            const float horizontalUnitY =
                relativePos.y / slantRangeNM;

            const float verticalUnit =
                altitudeDifferenceNM / slantRangeNM;

            const float relativeVelocityX =
                targetKinematics.velocity.x -
                obsKinematics.velocity.x;

            const float relativeVelocityY =
                targetKinematics.velocity.y -
                obsKinematics.velocity.y;

            const float relativeVelocityZ =
                targetKinematics.verticalVelocityNMPerSec -
                obsKinematics.verticalVelocityNMPerSec;

            const float radialVelocityNMPerSec =
                relativeVelocityX * horizontalUnitX +
                relativeVelocityY * horizontalUnitY +
                relativeVelocityZ * verticalUnit;

            trueRadialVelocityKnots =
                radialVelocityNMPerSec * NM_PER_SEC_TO_KNOTS;
        }

        // ------------------------------------------------
        // Apply measurement noise
        // ------------------------------------------------

        const float rangeError = std::max(0.0f, obsRadar.rangeErrorNM);
        const float angleError = std::max(0.0f, obsRadar.angleErrorRadians);
        const float velocityError = std::max(0.0f, obsRadar.velocityErrorKnots);

        std::normal_distribution rangeNoise(
            0.0f, rangeError > 0.0f ? rangeError : 1.0f
        );

        std::normal_distribution angleNoise(
            0.0f, angleError > 0.0f ? angleError : 1.0f
        );

        std::normal_distribution velocityNoise(
            0.0f, velocityError > 0.0f ? velocityError : 1.0f
        );

        // Generate noise only when the configured standard deviation is positive.
        const float rangeNoiseValue =
            rangeError > 0.0f ? rangeNoise(rng) : 0.0f;

        const float azimuthNoiseValue =
            angleError > 0.0f ? angleNoise(rng) : 0.0f;

        const float elevationNoiseValue =
            angleError > 0.0f ? angleNoise(rng) : 0.0f;

        const float velocityNoiseValue =
            velocityError > 0.0f ? velocityNoise(rng) : 0.0f;

        // Construct measurement.
        RadarMeasurement measurement{};

        measurement.rangeNM =
            std::max(0.001f, slantRangeNM + rangeNoiseValue);

        measurement.azimuthRadians =
            trueAzimuthRadians + azimuthNoiseValue;

        measurement.elevationRadians =
            trueElevationRadians + elevationNoiseValue;

        measurement.radialVelocityKnots =
            trueRadialVelocityKnots + velocityNoiseValue;

        measurement.timestampSec = timestampSec;

        return measurement;
    }
}

void Systems::SensorSystem(entt::registry& registry, const float deltaTime)
{
    const auto& map = registry.ctx().get<MapProjection>();
    auto& detectionData = registry.ctx().get<RadarDetectionData>();
    auto& rng = registry.ctx().get<std::mt19937>();

    const float timestampSec = registry.ctx().get<float>();

    // Clear measurements from the previous simulation frame
    detectionData.frameMeasurements.clear();

    const auto observerView = registry.view<Transform2D, Kinematics, RadarEmitter, IFF>();

    observerView.each([&] (
        const entt::entity observer,
        const Transform2D& obsTransform,
        const Kinematics& obsKinematics,
        RadarEmitter& obsRadar,
        const IFF& obsIFF)
    {
        obsRadar.timeSinceLastScan += deltaTime;

        if (obsRadar.timeSinceLastScan < obsRadar.scanRateSec) return;

        obsRadar.timeSinceLastScan -= obsRadar.scanRateSec;

        std::vector<RadarMeasurement> measurements;
        measurements.reserve(10);

        const auto targetView = registry.view<Transform2D, Kinematics, RadarSignature, IFF>();

        targetView.each([&] (
            const entt::entity target,
            const Transform2D& targetTransform,
            const Kinematics& targetKinematics,
            const RadarSignature& targetRadarSignature,
            const IFF& targetIFF)
        {
            if (target == observer) return;
            if (obsIFF.isHostile == targetIFF.isHostile) return;

            const auto measurement = ScanForTargets(
                map,
                obsTransform,
                obsKinematics,
                obsRadar,
                targetTransform,
                targetKinematics,
                targetRadarSignature,
                timestampSec,
                rng
            );

            if (measurement.has_value()) measurements.push_back(*measurement);
        });

        detectionData.frameMeasurements[observer] = std::move(measurements);
    });
}