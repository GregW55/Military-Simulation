#pragma once
#include "../core/Constants.h"

namespace MathUtils {

    // =======================
    // === PRIMITIVE TYPES ===
    // =======================
    struct Vec2 {
        float x = 0.0f;
        float y = 0.0f;
    };


    // =================================================
    // === VECTOR MATH & BASICS (Inline & Constexpr) ===
    // =================================================
    constexpr Vec2 Add(Vec2 a, Vec2 b) noexcept {
        return { a.x + b.x, a.y + b.y };
    }

    constexpr Vec2 Sub(Vec2 a, Vec2 b) noexcept {
        return { a.x - b.x, a.y - b.y };
    }

    constexpr Vec2 Scale(Vec2 v, float s) noexcept {
        return { v.x * s, v.y * s };
    }

    constexpr float Dot(Vec2 a, Vec2 b) noexcept {
        return (a.x * b.x) + (a.y * b.y);
    }

    constexpr Vec2 Lerp(Vec2 start, Vec2 end, float t) noexcept {
        return {start.x + (end.x - start.x) * t,
                   start.y + (end.y - start.y) * t
        };
    }
    float Length(Vec2 v) noexcept;
    float LengthSq(Vec2 v) noexcept;
    float GetDistance(Vec2 a, Vec2 b) noexcept;
    float GetDistanceSq(Vec2 a, Vec2 b) noexcept;


    // ========================
    // === UNIT CONVERSIONS ===
    // ========================
    constexpr float KnotsToPixelsPerSec(float knots) noexcept {
        return knots * PX_PER_KNOT_SEC;
    }

    constexpr float KnotsToNmPerSec(float knots) noexcept{
        return knots * KNOTS_TO_NM_PER_SEC;
    }

    constexpr float NmPerSecToKnots(float nmPerSec) noexcept {
        return nmPerSec * NM_PER_SEC_TO_KNOTS;
    }

    constexpr float MetersToFeet(float meters) noexcept {
        return meters * METERS_TO_FT;
    }

    constexpr float NmToPixels(float nm) noexcept {
        return nm * PX_PER_NM;
    }

    constexpr float PixelsToNm(float pixels) noexcept {
        return pixels * NM_PER_PX;
    }


    // ===========================
    // === NAVIGATION & ANGLES ===
    // ===========================
    float GetAngleDegrees(Vec2 start, Vec2 end) noexcept;
    float WrapAngle(float angle) noexcept;
    float GetShortestAngleDiff(float current, float target) noexcept;
    float GetMaxTurnRateDegSec(float speedKnots, float maxLateralGs) noexcept;

    // Angle in degrees between `headingDir` and the direction from `from` to `to`.
    // 0 = dead ahead, 180 = directly behind.
    float AngleOffHeadingDegrees(Vec2 headingDir, Vec2 from, Vec2 to) noexcept;


   // ======================================
   // === GEOMETRY, PHYSICS & KINEMATICS ===
   // ======================================
    float GetRadarHorizonNM(float altitudeMeters) noexcept;

    inline Vec2 ExtrapolatePosition(Vec2 pos, Vec2 vel, float seconds) noexcept {
        return Add(pos, Scale(vel, seconds));
    }

    inline Vec2 GetVelocityNmPerSec(Vec2 headingDir, float speedKnots) noexcept {
        return Scale(headingDir, KnotsToNmPerSec(speedKnots));
    }

    inline Vec2 VelocityBetweenPositions(Vec2 from, Vec2 to, float seconds) noexcept {
        return Scale(Sub(to, from), 1.0f / seconds);
    }

    inline float VelocityDifferenceKnots(Vec2 velA, Vec2 velB) noexcept {
        return NmPerSecToKnots(Length(Sub(velA, velB)));
    }

    inline Vec2 GetDirection(Vec2 from, Vec2 to) noexcept {
        Vec2 delta = Sub(to, from);
        return Scale(delta, 1.0f / Length(delta));
    }

    inline float GetClosingSpeedNmPerSec(Vec2 from, Vec2 vel, Vec2 to) noexcept {
        return Dot(vel, GetDirection(from, to));
    }


    // ==================================
    // === INTERCEPT & GUIDANCE LOGIC ===
    // ==================================
    Vec2 PredictIntercept(
        Vec2 shooterPos,
        float missileSpeedKnots,
        Vec2 targetPos,
        Vec2 targetVel
        );

    float CalculateProNavTurnRate(
        Vec2 missilePos,
        Vec2 missileVel,
        Vec2 targetPos,
        Vec2 targetVel,
        float navConstant = 4.0f
        ) noexcept;


    // ==============================
    // === UTILITIES & FORMATIONS ===
    // ==============================
    int CalculateGridColumns(int totalCount) noexcept;
}
