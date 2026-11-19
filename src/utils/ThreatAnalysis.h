#pragma once
#include "../ecs/components.h"
#include "MathUtils.h"

namespace ThreatAnalysis {
    inline ThreatClass Classify(float speedKnots, float speedSigmaKnots,
                            float altitudeM, float altSigmaM) {
        if (speedKnots - 3.0f * speedSigmaKnots >= MAX_SHIP_SPEED_KTS ||
            altitudeM  - 3.0f * altSigmaM       >  MAX_SHIP_ALTITUDE_M)
            return ThreatClass::MISSILE_INBOUND;

        if (speedKnots + 3.0f * speedSigmaKnots <= MAX_SHIP_SPEED_KTS)
            return ThreatClass::SURFACE_SHIP;

        return ThreatClass::UNKNOWN;
    }

    inline float ProcessNoisePSD(ThreatClass c) {
        switch (c) {
            case ThreatClass::SURFACE_SHIP:    return KALMAN_PSD_SHIP;
            case ThreatClass::MISSILE_INBOUND:
            case ThreatClass::AIRCRAFT:        return KALMAN_PSD_MISSILE;
            default:                           return KALMAN_PSD_UNKNOWN;
        }
    }

    inline float ComputeThreatScore(const RadarTrack& track) {
        if (track.timeToImpactSec < 0.0f) return 0.0f;

        const float urgency = 1.0f / (track.timeToImpactSec + 1.0f);
        const float closingSpeedKnots = std::max(0.0f, track.closingSpeedKnots);

        float classMult = 1.0f;
        switch (track.classification) {
            case ThreatClass::MISSILE_INBOUND:  classMult = 10.0f; break;
            case ThreatClass::AIRCRAFT:         classMult = 3.0f; break;
            case ThreatClass::SURFACE_SHIP:     classMult = 1.0f; break;
            default:                            classMult = 0.5f; break;
        }

        return urgency * classMult * closingSpeedKnots;
    }

    inline float EstimateTimeToImpact(float distance, float closingSpeedNmPerSec)
    {
        if (closingSpeedNmPerSec <= 0.0f) return -1.0f; // Moving away

        return distance / closingSpeedNmPerSec; // Seconds until arrival
    }
}