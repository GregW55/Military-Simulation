#include "MathUtils.h"
#include <algorithm>
#include "../core/Constants.h"
#include <cmath>

namespace MathUtils {

    // =================================================
    // === VECTOR MATH & BASICS (Inline & Constexpr) ===
    // =================================================
    float Length(Vec2 v) noexcept{ return std::sqrtf(LengthSq(v)); }

    float LengthSq(Vec2 v) noexcept { return v.x * v.x + v.y * v.y; }

    float GetDistance(Vec2 a, Vec2 b) noexcept {
        Vec2 delta = Sub(a, b);
        return Length(delta);
     }

     float GetDistanceSq(Vec2 a, Vec2 b) noexcept {
        Vec2 delta = Sub(a, b);
        return LengthSq(delta);
    }


    // ===========================
    // === NAVIGATION & ANGLES ===
    // ===========================
    float GetAngleDegrees(Vec2 start, Vec2 end) noexcept {
        Vec2 delta = Sub(end, start);
        float rad = std::atan2f(delta.y, delta.x);
        return rad * RAD_TO_DEG;
    }

    float WrapAngle(float angle) noexcept {
        angle = std::fmod(angle + 180.0f, 360.0f);
        if (angle < 0.0f) angle += 360.0f;
        return angle - 180.0f;
    }

    float GetShortestAngleDiff(float current, float target) noexcept {
        return WrapAngle(target - current);
    }

    float GetMaxTurnRateDegSec(float speedKnots, float maxLateralGs) noexcept {
        float speedMps = speedKnots * MPS_PER_KNOT;
        if (speedMps < 1.0f) return 0.0;

        float maxLateralAccelMps2 = maxLateralGs * GRAVITY_MPS2;
        float maxTurnRateRadSec = maxLateralAccelMps2 / speedMps;  // a = v * ω  →  ω = a / v
        return maxTurnRateRadSec * RAD_TO_DEG;
    }

    float AngleOffHeadingDegrees(Vec2 headingDir, Vec2 from, Vec2 to) noexcept {
        Vec2 toTarget = Sub(to, from);
        float dist = Length(toTarget);

        Vec2 toTargetDir = Scale(toTarget, 1.0f / dist);
        float dot = Dot(headingDir, toTargetDir);
        return std::acos(std::clamp(dot, -1.0f, 1.0f)) * RAD_TO_DEG;
    }



    // ======================================
    // === GEOMETRY, PHYSICS & KINEMATICS ===
    // ======================================
    float GetRadarHorizonNM(float altitudeMeters) noexcept {
        if (altitudeMeters < 0) altitudeMeters = 0;
        return EARTH_REFRACTION_FACTOR * std::sqrt(MetersToFeet(altitudeMeters));
    }


    // ==================================
    // === INTERCEPT & GUIDANCE LOGIC ===
    // ==================================
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

    float CalculateProNavTurnRate(Vec2 missilePos, Vec2 missileVel, Vec2 targetPos, Vec2 targetVel, float navConstant) noexcept {
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


    // ==============================
    // === UTILITIES & FORMATIONS ===
    // ==============================
    // --- Formations & Spawning ---
    int CalculateGridColumns(int totalCount) noexcept {
        if (totalCount <= 0) return 0;
        return static_cast<int>(std::ceil(std::sqrt(static_cast<float>(totalCount))));
    }
}