#pragma once

#include "../ecs/components.h"

class KalmanFilter3D
{
public:
    static void Initialize(
        KalmanState3D& state,
        const RadarMeasurement& measurement,
        const Transform2D& observer,
        const Kinematics& observerKinematics,
        const RadarEmitter& radar);

    static void Predict(
        KalmanState3D& state,
        float deltaTime);

    static bool Correct(
        KalmanState3D& state,
        const RadarMeasurement& measurement,
        const Transform2D& observer,
        const Kinematics& observerKinematics,
        const RadarEmitter& radar);

    static void CopyToTrack(RadarTrack& track);
};