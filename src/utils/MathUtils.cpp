#include "MathUtils.h"
#include <algorithm>
#include "../core/Constants.h"
#include <cmath>

namespace MathUtils {
    // --- Internal Math Helpers (Not Exposed in header) ---
    float LengthSq(Vec2 v) { return v.x * v.x + v.y * v.y; }
    float Length(Vec2 v) { return std::sqrt(LengthSq(v)); }
    float Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
    Vec2 Sub(Vec2 a, Vec2 b) { return { a.x - b.x, a.y - b.y }; }
    Vec2 Add(Vec2 a, Vec2 b) { return { a.x + b.x, a.y + b.y }; }
    Vec2 Scale(Vec2 v, float s) { return { v.x * s, v.y * s }; }
    Vec2 Lerp(Vec2 start, Vec2 end, float t) {
        return {
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t
        };
    }

    float WrapAngle(float angle) {
        while (angle > 180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        return angle;
    }

    float GetShortestAngleDiff(float current, float target) {
        return WrapAngle(target - current);
    }

    float GetRadarHorizonNM(float altitudeMeters) {
        if (altitudeMeters < 0) altitudeMeters = 0;
        return EARTH_REFRACTION_FACTOR * std::sqrt(MetersToFeet(altitudeMeters));
    }

    Vec2 PredictIntercept(Vec2 shooterPos, float missileSpeedKnots, Vec2 targetPos, Vec2 targetVel) {
        float missileSpeedNmps = KnotsToNmPerSec(missileSpeedKnots);
        Vec2 relativePos = Sub(targetPos, shooterPos);

        float a = LengthSq(targetVel) - (missileSpeedNmps * missileSpeedNmps);
        float b = 2.0f * Dot(targetVel, relativePos);
        float c = LengthSq(relativePos);

        float t = 0.0f;
        if (std::abs(a) < MATH_EPSILON) {
            if (std::abs(b) > MATH_EPSILON) t = -c / b;
            else return targetPos;
        } else {
            float discriminant = (b * b) - (4.0f * a * c);
            if (discriminant < 0.0f) return targetPos;
            float t1 = (-b - std::sqrt(discriminant)) / (2.0f * a);
            float t2 = (-b + std::sqrt(discriminant)) / (2.0f * a);
            if (t1 > 0 && t2 > 0) t = std::min(t1, t2);
            else t = std::max(t1, t2);
        }

        if (t < MATH_EPSILON) return targetPos;
        return Add(targetPos, Scale(targetVel, t));
    }

    float CalculateProNavTurnRate(Vec2 missilePos, Vec2 missileVel, Vec2 targetPos, Vec2 targetVel, float navConstant) {
        Vec2 los = Sub(targetPos, missilePos);
        float distSq = LengthSq(los);
        if (distSq < 0.1f) return 0.0f;

        float dist = std::sqrt(distSq);
        Vec2 losDir = Scale(los, 1.0f / dist);

        Vec2 relVel = Sub(targetVel, missileVel);
        float closingSpeed = -Dot(relVel, losDir);

        float crossProd = (los.x * relVel.y) - (los.y * relVel.x);
        float losRate = crossProd / distSq;

        float lateralAccel = navConstant * closingSpeed * losRate;
        float speed = Length(missileVel);

        if (speed > 0.01f) {
            return (lateralAccel / speed) * RAD_TO_DEG; // RAD2DEG
        }
        return 0.0f;
    }

    float GetDistance(Vec2 a, Vec2 b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    float GetAngleDegrees(Vec2 start, Vec2 end) {
        float dx = end.x - start.x;
        float dy = end.y - start.y;
        return std::atan2(dy, dx) * RAD_TO_DEG;
    }

    // --- Formations & Spawning ---
    int CalculateGridColumns(int totalCount) {
        return static_cast<int>(std::ceil(std::sqrt(totalCount)));
    }
}