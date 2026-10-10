#pragma once
#include "../ecs/components.h"
#include "MathUtils.h"

namespace ThreatAnalysis {
    inline ThreatClass Classify(float speedKnots, float altitude) {
        if (speedKnots >= MIN_MISSILE_SPEED_KTS || altitude > 5000.0f) {
            return ThreatClass::MISSILE_INBOUND;
        }
        if (speedKnots >= MIN_AIRCRAFT_SPEED_KTS) {
            return ThreatClass::AIRCRAFT;
        }
        if (speedKnots <= MAX_SHIP_SPEED_KTS) {
            return ThreatClass::SURFACE_SHIP;
        }
        return ThreatClass::UNKNOWN;
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