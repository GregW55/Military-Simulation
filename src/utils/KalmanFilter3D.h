#pragma once

#include "../ecs/components.h"

struct RadarInnovation {
    // z - h(x)
    std::array<float, 4> residual{};

    // S = HPH^T + R
    std::array<float, 16> covariance{};

    // Inverse of S.
    std::array<float, 16> inverseCovariance{};

    // Measurement Jacobian, 4x6
    std::array<float, 24> H{};

    // P * H^T, 6x4
    std::array<float, 24> PHt{};

    // Diagonal of measurement noise covariance R
    std::array<float, 4> measurementVariance{};

    float mahalanobisSquared = 0.0f;

    bool valid = false;
};
class KalmanFilter3D
{
public:
    static void Initialize(
        KalmanState3D& state,
        const RadarMeasurement& measurement,
        const Transform2D& observer,
        const Kinematics& observerKinematics
    );

    static void Predict(
        KalmanState3D& state,
        float deltaTime
    );

    static bool CalculateInnovation(
        const KalmanState3D& state,
        const RadarMeasurement& measurement,
        const Transform2D& observer,
        const Kinematics& observerKinematics,
        RadarInnovation& result
    );

    static bool Correct(
        KalmanState3D& state,
        const RadarMeasurement& measurement,
        const Transform2D& observer,
        const Kinematics& observerKinematics
    );

    static void CopyToTrack(RadarTrack& track);
};