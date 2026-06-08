#pragma once
#include <cmath>

namespace Physics {
    // Constants from US Standard Atmosphere 1976
    constexpr float SEA_LEVEL_TEMP_K = 288.15f;    // 15C
    constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;
    constexpr float GRAVITY = 9.80665f;
    constexpr float GAS_CONSTANT_AIR = 287.05f;    // R specific for air
    constexpr float ADIABATIC_GAS_CONSTANT = 1.4f * GAS_CONSTANT_AIR;
    constexpr float LAPSE_RATE_TROPO = -0.0065f;   // Temp drops 6.5C per km
    constexpr float ALTITUDE_TROPOPAUSE = 11000.0f; // 11km
    constexpr float TROPO_EXPONENT = -GRAVITY / (LAPSE_RATE_TROPO * GAS_CONSTANT_AIR);
    constexpr float TEMP_STRATO_K = 216.65f;
    constexpr float STRATO_MULTIPLIER = -GRAVITY / (GAS_CONSTANT_AIR * TEMP_STRATO_K);

    // Internal helper to get Temperature (Kelvin) based on Altitude
    inline float GetTemperatureK(float altMeters) {
        if (altMeters < ALTITUDE_TROPOPAUSE) {
            // Troposphere: Temp drops linearly
            return SEA_LEVEL_TEMP_K + (LAPSE_RATE_TROPO * altMeters);
        }
        // Stratosphere (Lower): Temp is constant -56.5C (216.65K)
        // Note: This holds true up to ~20km, covering 99% of air-breathing threats
        return TEMP_STRATO_K;
    }

    // Internal helper for Pressure
    inline float GetPressurePa(float altMeters, float tempK) {
        if (altMeters < ALTITUDE_TROPOPAUSE) {
            // Troposphere Formula (Power Law)
            return SEA_LEVEL_PRESSURE_PA * std::pow(tempK / SEA_LEVEL_TEMP_K, TROPO_EXPONENT);
        }
        // Stratosphere Formula (Exponential)
        static const float pressureAt11km = SEA_LEVEL_PRESSURE_PA * std::pow(TEMP_STRATO_K / SEA_LEVEL_TEMP_K, TROPO_EXPONENT);

        float heightAboveTropo = altMeters - ALTITUDE_TROPOPAUSE;
        return pressureAt11km * std::exp(STRATO_MULTIPLIER * heightAboveTropo);
    }

    // Calculate Air Density (rho) using Ideal Gas Law: rho = P / (RT)
    inline float GetAirDensity(float altMeters) {
        if (altMeters < 0) altMeters = 0;

        float tempK = GetTemperatureK(altMeters);
        float pressure = GetPressurePa(altMeters, tempK);

        return pressure / (GAS_CONSTANT_AIR * tempK);
    }

    // Accurate Speed of Sound (Mach 1) calculation
    inline float GetSpeedOfSound(float altMeters) {
        if (altMeters < 0) altMeters = 0;
        float tempK = GetTemperatureK(altMeters);

        // c = sqrt(gamma * R * T)
        // adiabatic index (gamma) for air is 1.4
        return std::sqrt(ADIABATIC_GAS_CONSTANT * tempK);
    }

    inline bool CheckRadarDetection(
        MapProjection& map,
        MathUtils::Vec2 observerPos, float observerAltMeters, float radarRangeNM,
        MathUtils::Vec2 targetPos, float targetAltMeters, float targetRcsFourthRoot, float distNM) {

        // RADAR HORIZON (EARTH'S CURVATURE)
        float obsHorizon = MathUtils::GetRadarHorizonNM(observerAltMeters);
        float tgtHorizon = MathUtils::GetRadarHorizonNM(targetAltMeters);

        if (distNM > (obsHorizon + tgtHorizon)) return false;

        float effectiveRange = radarRangeNM * targetRcsFourthRoot;
        if (distNM > effectiveRange) return false;

        // TERRAIN MASKING
        if (!map.HasLineOfSight(observerPos, targetPos, observerAltMeters, targetAltMeters)) {
            return false;
        }
        return true;
    }
}