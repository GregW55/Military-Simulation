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
        if (track.timeToImpactSec < 0.0f) return 0.0f; // Not on intercept course

        float urgency = 1.0f / (track.timeToImpactSec + 1.0f);

        float classMult = 1.0f;
        switch (track.classification) {
            case ThreatClass::MISSILE_INBOUND:  classMult = 10.0f; break;
            case ThreatClass::AIRCRAFT:         classMult = 3.0f; break;
            case ThreatClass::SURFACE_SHIP:     classMult = 1.0f; break;
            default:                            classMult = 0.5f; break;
        }
        return urgency * classMult * track.closingSpeedKnots;
    }

    inline float EstimateTimeToImpact(
        MathUtils::Vec2 observerPos,
        MathUtils::Vec2 trackPos,
        MathUtils::Vec2 trackVel)
    {
        float speedNmps = MathUtils::Length(trackVel);
        if (speedNmps < 0.0001f) return -1.0f;

        // Positive = closing, negative = opening
        float dist = MathUtils::GetDistance(observerPos, trackPos);
        if (dist < 0.001f) return 0.0f;

        float closingSpeedNmPerSec = MathUtils::GetClosingSpeedNmPerSec(observerPos, trackVel, trackPos); // NM/sec

        if (closingSpeedNmPerSec <= 0.0f) return -1.0f; // Moving away

        return dist / closingSpeedNmPerSec; // Seconds until arrival
    }
}