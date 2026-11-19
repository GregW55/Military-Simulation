#include "KalmanFilter3D.h"

#include "../core/Constants.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace
{
    constexpr int STATE_SIZE = 6;
    constexpr int MEASUREMENT_SIZE = 4;

    constexpr float ACCELERATION_NOISE_NM_S2 = 0.05f;
    constexpr float MIN_RANGE_NM = 0.0001f;
    constexpr float MIN_HORIZONTAL_RANGE_NM = 0.0001f;

    using Matrix6 = std::array<float, 36>;
    using Matrix4 = std::array<float, 16>;
    using Matrix6x4 = std::array<float, 24>;
    using Matrix4x6 = std::array<float, 24>;

    constexpr int Index6(int row, int col)
    {
        return row * STATE_SIZE + col;
    }

    constexpr int Index4(int row, int col)
    {
        return row * MEASUREMENT_SIZE + col;
    }

    constexpr int Index6x4(int row, int col)
    {
        return row * MEASUREMENT_SIZE + col;
    }

    constexpr int Index4x6(int row, int col)
    {
        return row * STATE_SIZE + col;
    }

    float WrapRadians(float angle)
    {
        while (angle > MATH_PI)
            angle -= 2.0f * MATH_PI;

        while (angle < -MATH_PI)
            angle += 2.0f * MATH_PI;

        return angle;
    }

    float Square(float value)
    {
        return value * value;
    }

    struct ObserverState
    {
        std::array<float, 3> position{};
        std::array<float, 3> velocity{};
    };

    ObserverState GetObserverState(
        const Transform2D& transform,
        const Kinematics& kinematics)
    {
        ObserverState observer{};

        observer.position = {
            transform.pos.x,
            transform.pos.y,
            transform.altitude / METERS_PER_NM
        };

        observer.velocity = {
            kinematics.velocity.x,
            kinematics.velocity.y,
            kinematics.verticalVelocityNMPerSec
        };

        return observer;
    }

    bool Invert4x4(const Matrix4& input, Matrix4& inverse)
    {
        float augmented[4][8]{};

        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
                augmented[row][col] = input[Index4(row, col)];

            augmented[row][row + 4] = 1.0f;
        }

        for (int col = 0; col < 4; ++col)
        {
            int pivotRow = col;

            for (int row = col + 1; row < 4; ++row)
            {
                if (std::abs(augmented[row][col]) >
                    std::abs(augmented[pivotRow][col]))
                {
                    pivotRow = row;
                }
            }

            if (std::abs(augmented[pivotRow][col]) < 1e-10f)
                return false;

            if (pivotRow != col)
            {
                for (int k = 0; k < 8; ++k)
                    std::swap(augmented[col][k], augmented[pivotRow][k]);
            }

            const float pivot = augmented[col][col];

            for (int k = 0; k < 8; ++k)
                augmented[col][k] /= pivot;

            for (int row = 0; row < 4; ++row)
            {
                if (row == col)
                    continue;

                const float factor = augmented[row][col];

                for (int k = 0; k < 8; ++k)
                    augmented[row][k] -= factor * augmented[col][k];
            }
        }

        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
                inverse[Index4(row, col)] = augmented[row][col + 4];
        }

        return true;
    }

    void BuildMeasurementModel(
        const KalmanState3D& state,
        const ObserverState& observer,
        std::array<float, 4>& predicted,
        Matrix4x6& H)
    {
        const float dx = state.x[0] - observer.position[0];
        const float dy = state.x[1] - observer.position[1];
        const float dz = state.x[2] - observer.position[2];

        const float dvx = state.x[3] - observer.velocity[0];
        const float dvy = state.x[4] - observer.velocity[1];
        const float dvz = state.x[5] - observer.velocity[2];

        const float horizontalSq = dx * dx + dy * dy;
        const float horizontal = std::sqrt(horizontalSq);

        const float rangeSq = horizontalSq + dz * dz;
        const float range = std::sqrt(rangeSq);

        if (range < MIN_RANGE_NM ||
            horizontal < MIN_HORIZONTAL_RANGE_NM)
        {
            predicted.fill(0.0f);
            H.fill(0.0f);
            return;
        }

        const float radialNumerator =
            dx * dvx + dy * dvy + dz * dvz;

        predicted[0] = range;
        predicted[1] = std::atan2(dy, dx);
        predicted[2] = std::atan2(dz, horizontal);
        predicted[3] =
            (radialNumerator / range) * NM_PER_SEC_TO_KNOTS;

        H.fill(0.0f);

        // Range derivatives.
        H[Index4x6(0, 0)] = dx / range;
        H[Index4x6(0, 1)] = dy / range;
        H[Index4x6(0, 2)] = dz / range;

        // Azimuth derivatives.
        H[Index4x6(1, 0)] = -dy / horizontalSq;
        H[Index4x6(1, 1)] = dx / horizontalSq;

        // Elevation derivatives.
        H[Index4x6(2, 0)] =
            -dz * dx / (rangeSq * horizontal);

        H[Index4x6(2, 1)] =
            -dz * dy / (rangeSq * horizontal);

        H[Index4x6(2, 2)] =
            horizontal / rangeSq;

        // Radial velocity derivatives.
        const float rangeCubed = rangeSq * range;

        H[Index4x6(3, 0)] =
            (dvx / range -
             radialNumerator * dx / rangeCubed)
            * NM_PER_SEC_TO_KNOTS;

        H[Index4x6(3, 1)] =
            (dvy / range -
             radialNumerator * dy / rangeCubed)
            * NM_PER_SEC_TO_KNOTS;

        H[Index4x6(3, 2)] =
            (dvz / range -
             radialNumerator * dz / rangeCubed)
            * NM_PER_SEC_TO_KNOTS;

        H[Index4x6(3, 3)] =
            (dx / range) * NM_PER_SEC_TO_KNOTS;

        H[Index4x6(3, 4)] =
            (dy / range) * NM_PER_SEC_TO_KNOTS;

        H[Index4x6(3, 5)] =
            (dz / range) * NM_PER_SEC_TO_KNOTS;
    }
}

void KalmanFilter3D::Initialize(
    KalmanState3D& state,
    const RadarMeasurement& measurement,
    const Transform2D& observer,
    const Kinematics& observerKinematics,
    const RadarEmitter& radar)
{
    state = {};

    const ObserverState observerState =
        GetObserverState(observer, observerKinematics);

    const float horizontalRange =
        measurement.rangeNM * std::cos(measurement.elevationRadians);

    const float verticalRange =
        measurement.rangeNM * std::sin(measurement.elevationRadians);

    const float losX =
        std::cos(measurement.elevationRadians) *
        std::cos(measurement.azimuthRadians);

    const float losY =
        std::cos(measurement.elevationRadians) *
        std::sin(measurement.azimuthRadians);

    const float losZ =
        std::sin(measurement.elevationRadians);

    // Initialize absolute position from the polar measurement.
    state.x[0] =
        observerState.position[0] +
        horizontalRange * std::cos(measurement.azimuthRadians);

    state.x[1] =
        observerState.position[1] +
        horizontalRange * std::sin(measurement.azimuthRadians);

    state.x[2] =
        observerState.position[2] + verticalRange;

    // A single radar observation cannot determine tangential velocity.
    // Initialize the observable radial component and retain large
    // uncertainty in velocity until subsequent observations arrive.
    const float radialVelocity =
        measurement.radialVelocityKnots / NM_PER_SEC_TO_KNOTS;

    state.x[3] =
        observerState.velocity[0] + radialVelocity * losX;

    state.x[4] =
        observerState.velocity[1] + radialVelocity * losY;

    state.x[5] =
        observerState.velocity[2] + radialVelocity * losZ;

    const float rangeSigma =
        std::max(0.0001f, radar.rangeErrorNM);

    const float angleSigma =
        std::max(0.000001f, radar.angleErrorRadians);

    const float positionVariance =
        Square(rangeSigma) +
        Square(measurement.rangeNM * angleSigma);

    const float radialVelocitySigma =
        std::max(0.001f, radar.velocityErrorKnots) /
        NM_PER_SEC_TO_KNOTS;

    const float velocityVariance =
        Square(radialVelocitySigma) + Square(0.25f);

    for (int i = 0; i < 3; ++i)
        state.P[Index6(i, i)] = positionVariance;

    for (int i = 3; i < 6; ++i)
        state.P[Index6(i, i)] = velocityVariance;

    state.timestampSec = measurement.timestampSec;
    state.measurementCount = 1;
    state.initialized = true;
}

void KalmanFilter3D::Predict(
    KalmanState3D& state,
    float deltaTime)
{
    if (!state.initialized || deltaTime <= 0.0f)
        return;

    // Constant-velocity state prediction.
    for (int axis = 0; axis < 3; ++axis)
        state.x[axis] += state.x[axis + 3] * deltaTime;

    // F * P * F^T.
    Matrix6 predictedP{};

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            float value = state.P[Index6(row, col)];

            if (row < 3)
                value += deltaTime * state.P[Index6(row + 3, col)];

            if (col < 3)
                value += deltaTime * state.P[Index6(row, col + 3)];

            if (row < 3 && col < 3)
                value += deltaTime * deltaTime *
                         state.P[Index6(row + 3, col + 3)];

            predictedP[Index6(row, col)] = value;
        }
    }

    // White-acceleration process noise, independently per axis.
    const float q = state.processNoiseAccelPSD;
    const float dt2 = deltaTime * deltaTime;
    const float dt3 = dt2 * deltaTime;
    const float qPosition = q * dt3 / 3.0f;
    const float qCross    = q * dt2 * 0.5f;
    const float qVelocity = q * deltaTime;

    for (int axis = 0; axis < 3; ++axis)
    {
        predictedP[Index6(axis, axis)] += qPosition;
        predictedP[Index6(axis, axis + 3)] += qCross;
        predictedP[Index6(axis + 3, axis)] += qCross;
        predictedP[Index6(axis + 3, axis + 3)] += qVelocity;
    }

    state.P = predictedP;
    state.timestampSec += deltaTime;
}

bool KalmanFilter3D::Correct(
    KalmanState3D& state,
    const RadarMeasurement& measurement,
    const Transform2D& observer,
    const Kinematics& observerKinematics,
    const RadarEmitter& radar)
{
    if (!state.initialized)
        return false;

    const ObserverState observerState =
        GetObserverState(observer, observerKinematics);

    std::array<float, 4> predictedMeasurement{};
    Matrix4x6 H{};

    BuildMeasurementModel(
        state,
        observerState,
        predictedMeasurement,
        H
    );

    const float dx = state.x[0] - observerState.position[0];
    const float dy = state.x[1] - observerState.position[1];

    if (std::sqrt(dx * dx + dy * dy) <
        MIN_HORIZONTAL_RANGE_NM)
    {
        return false;
    }

    const std::array<float, 4> measured{
        measurement.rangeNM,
        measurement.azimuthRadians,
        measurement.elevationRadians,
        measurement.radialVelocityKnots
    };

    std::array<float, 4> residual{};

    for (int i = 0; i < 4; ++i)
        residual[i] = measured[i] - predictedMeasurement[i];

    // Angular residuals must use the shortest angular difference.
    residual[1] = WrapRadians(residual[1]);
    residual[2] = WrapRadians(residual[2]);

    Matrix6x4 PHt{};

    // P * H^T.
    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            float value = 0.0f;

            for (int k = 0; k < 6; ++k)
            {
                value +=
                    state.P[Index6(row, k)] *
                    H[Index4x6(col, k)];
            }

            PHt[Index6x4(row, col)] = value;
        }
    }

    // S = H * P * H^T + R.
    Matrix4 S{};

    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            float value = 0.0f;

            for (int k = 0; k < 6; ++k)
            {
                value +=
                    H[Index4x6(row, k)] *
                    PHt[Index6x4(k, col)];
            }

            S[Index4(row, col)] = value;
        }
    }

    S[Index4(0, 0)] += Square(std::max(0.0001f, radar.rangeErrorNM));
    S[Index4(1, 1)] += Square(std::max(0.000001f, radar.angleErrorRadians));
    S[Index4(2, 2)] += Square(std::max(0.000001f, radar.angleErrorRadians));
    S[Index4(3, 3)] += Square(std::max(0.001f, radar.velocityErrorKnots));

    Matrix4 inverseS{};

    if (!Invert4x4(S, inverseS))
        return false;

    // K = P * H^T * inverse(S).
    Matrix6x4 K{};

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            float value = 0.0f;

            for (int k = 0; k < 4; ++k)
            {
                value +=
                    PHt[Index6x4(row, k)] *
                    inverseS[Index4(k, col)];
            }

            K[Index6x4(row, col)] = value;
        }
    }

    // x = x + K * residual.
    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            state.x[row] +=
                K[Index6x4(row, col)] * residual[col];
        }
    }

    // Joseph covariance update:
    // P = (I - KH)P(I - KH)^T + KRK^T.
    Matrix6 A{};

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            float value = row == col ? 1.0f : 0.0f;

            for (int k = 0; k < 4; ++k)
            {
                value -=
                    K[Index6x4(row, k)] *
                    H[Index4x6(k, col)];
            }

            A[Index6(row, col)] = value;
        }
    }

    Matrix6 AP{};
    Matrix6 newP{};

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            for (int k = 0; k < 6; ++k)
            {
                AP[Index6(row, col)] +=
                    A[Index6(row, k)] *
                    state.P[Index6(k, col)];
            }
        }
    }

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            for (int k = 0; k < 6; ++k)
            {
                newP[Index6(row, col)] +=
                    AP[Index6(row, k)] *
                    A[Index6(col, k)];
            }
        }
    }

    // R is diagonal.
    const std::array<float, 4> R{
        Square(std::max(0.0001f, radar.rangeErrorNM)),
        Square(std::max(0.000001f, radar.angleErrorRadians)),
        Square(std::max(0.000001f, radar.angleErrorRadians)),
        Square(std::max(0.001f, radar.velocityErrorKnots))
    };

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            for (int k = 0; k < 4; ++k)
            {
                newP[Index6(row, col)] +=
                    K[Index6x4(row, k)] *
                    R[k] *
                    K[Index6x4(col, k)];
            }
        }
    }

    // Remove tiny floating-point asymmetries.
    for (int row = 0; row < 6; ++row)
    {
        for (int col = row; col < 6; ++col)
        {
            const float average =
                0.5f * (
                    newP[Index6(row, col)] +
                    newP[Index6(col, row)]
                );

            newP[Index6(row, col)] = average;
            newP[Index6(col, row)] = average;
        }

        newP[Index6(row, row)] =
            std::max(1e-10f, newP[Index6(row, row)]);
    }

    state.P = newP;
    ++state.measurementCount;

    return true;
}

void KalmanFilter3D::CopyToTrack(RadarTrack& track)
{
    if (!track.kalman.initialized)
        return;

    track.pos = {
        track.kalman.x[0],
        track.kalman.x[1]
    };

    track.vel = {
        track.kalman.x[3],
        track.kalman.x[4]
    };

    track.altitude =
        track.kalman.x[2] * METERS_PER_NM;

    track.speedKnots =
        std::sqrt(
            track.kalman.x[3] * track.kalman.x[3] +
            track.kalman.x[4] * track.kalman.x[4]
        ) * NM_PER_SEC_TO_KNOTS;

    // One measurement gives radial velocity but not a full 3D velocity.
    track.hasVelocity =
        track.kalman.measurementCount >= 2;
}