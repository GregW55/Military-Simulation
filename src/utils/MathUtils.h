#pragma once

namespace MathUtils {
    constexpr float PX_PER_KNOT_SEC = (399.78f / 60.0f) / 3600.0f;
    constexpr float FT_PER_METER = 3.28084f;
    constexpr float PX_PER_NM = 399.78f / 60.0f;
    constexpr float NM_PER_PX = 60.0f / 399.78f;

    struct Vec2 {
        float x, y;
    };

    float Length(Vec2 v);
    float LengthSq(Vec2 v);
    float GetDistance(Vec2 a, Vec2 b);
    float GetAngleDegrees(Vec2 start, Vec2 end);
    float Dot(Vec2 a, Vec2 b);
    Vec2 Sub(Vec2 a, Vec2 b);
    Vec2 Add(Vec2 a, Vec2 b);
    Vec2 Scale(Vec2 v, float s);
    Vec2 Lerp(Vec2 start, Vec2 end, float t);

    // --- Unit Conversions ---
    inline float KnotsToPixelsPerSec(float knots) {
        return knots * PX_PER_KNOT_SEC;
    }
    inline float KnotsToNmPerSec(float knots) {
        return knots / 3600.0f;
    }
    inline float MetersToFeet(float meters) {
        return meters * FT_PER_METER;
    }
    inline float NmToPixels(float nm) {
        return nm * PX_PER_NM;
    }
    inline float PixelsToNm(float pixels) {
        return pixels * NM_PER_PX;
    }

    // --- Navigation & Angles ---
    float WrapAngle(float angle);
    float GetShortestAngleDiff(float current, float target);

    // --- Physics & Geometry ---
    float GetRadarHorizonNM(float altitudeMeters);

    // --- Intercept Logic ---
    Vec2 PredictIntercept(
        Vec2 shooterPos,
        float missileSpeedKnots,
        Vec2 targetPos,
        Vec2 targetVel
        );

    // --- Proportional Navigation Logic ---
    float CalculateProNavTurnRate(
        Vec2 missilePos,
        Vec2 missileVel,
        Vec2 targetPos,
        Vec2 targetVel,
        float navConstant = 4.0f
        );

    // --- Formations & Spawning ---
    int CalculateGridColumns(int totalCount);
}
