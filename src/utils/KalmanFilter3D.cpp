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

    bool IsPositiveDefinite4(const Matrix4& matrix)
    {
        float L[4][4]{};

        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col <= row; ++col)
            {
                float sum = matrix[Index4(row, col)];

                for (int k = 0; k < col; ++k)
                    sum -= L[row][k] * L[col][k];

                if (row == col)
                {
                    if (!std::isfinite(sum) || sum <= 1e-12f)
                        return false;

                    L[row][col] = std::sqrt(sum);
                }
                else
                {
                    if (L[col][col] <= 0.0f)
                        return false;

                    L[row][col] = sum / L[col][col];
                }
            }
        }

        return true;
    }

    bool BuildMeasurementModel(
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
            return false;
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

        return true;
    }
}

bool KalmanFilter3D::CalculateInnovation(
    const KalmanState3D& state,
    const RadarMeasurement& measurement,
    const Transform2D& observer,
    const Kinematics& observerKinematics,
    RadarInnovation& result)
{
    result = {};

    if (!state.initialized)
        return false;

    // The track must already have been predicted to the
    // timestamp of this radar measurement.
    constexpr float TIMESTAMP_TOLERANCE_SEC = 0.01f;

    if (std::abs(
        state.timestampSec - measurement.timestampSec
    ) > TIMESTAMP_TOLERANCE_SEC)
    {
        return false;
    }

    const ObserverState observerState =
        GetObserverState(observer, observerKinematics);

    std::array<float, 4> predictedMeasurement{};

    Matrix4x6 H{};

    if (!BuildMeasurementModel(
        state,
        observerState,
        predictedMeasurement,
        H))
    {
        return false;
    }

    // --------------------------------------------------------
    // Measurement noise covariance R.
    // --------------------------------------------------------

    const std::array<float, 4> R{
        Square(std::max(0.000001f, measurement.rangeSigmaNM)),
        Square(std::max(0.000001f, measurement.azimuthSigmaRadians)),
        Square(std::max(0.000001f, measurement.elevationSigmaRadians)),
        Square(std::max(0.001f, measurement.radialVelocitySigmaKnots))
    };

    // --------------------------------------------------------
    // Residual: actual measurement - predicted measurement.
    // --------------------------------------------------------

    const std::array<float, 4> measured{
        measurement.rangeNM,
        measurement.azimuthRadians,
        measurement.elevationRadians,
        measurement.radialVelocityKnots
    };

    std::array<float, 4> residual{};

    for (int i = 0; i < 4; ++i)
    {
        if (!std::isfinite(measured[i]) ||
            !std::isfinite(predictedMeasurement[i]))
        {
            return false;
        }

        residual[i] = measured[i] - predictedMeasurement[i];
    }

    // Angles must use the shortest angular difference.
    residual[1] = WrapRadians(residual[1]);
    residual[2] = WrapRadians(residual[2]);

    // --------------------------------------------------------
    // Calculate P * H^T.
    // --------------------------------------------------------

    Matrix6x4 PHt{};

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

    // --------------------------------------------------------
    // Calculate S = H * P * H^T + R.
    // --------------------------------------------------------

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

    for (int i = 0; i < 4; ++i)
        S[Index4(i, i)] += R[i];

    if (!IsPositiveDefinite4(S))
        return false;

    Matrix4 inverseS{};

    if (!Invert4x4(S, inverseS))
        return false;

    // --------------------------------------------------------
    // Calculate Mahalanobis distance squared.
    // d² = residual^T * inverse(S) * residual.
    // --------------------------------------------------------

    double distanceSquared = 0.0;

    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            distanceSquared +=
                static_cast<double>(residual[row]) *
                inverseS[Index4(row, col)] *
                residual[col];
        }
    }

    if (!std::isfinite(distanceSquared) ||
        distanceSquared < -1e-6)
    {
        return false;
    }

    // --------------------------------------------------------
    // Return everything needed by both association and correction.
    // --------------------------------------------------------

    result.residual = residual;
    result.covariance = S;
    result.inverseCovariance = inverseS;
    result.H = H;
    result.PHt = PHt;
    result.measurementVariance = R;

    result.mahalanobisSquared =
        static_cast<float>(std::max(0.0, distanceSquared));

    result.valid = true;

    return true;
}

void KalmanFilter3D::Initialize(
    KalmanState3D& state,
    const RadarMeasurement& measurement,
    const Transform2D& observer,
    const Kinematics& observerKinematics)
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

    // --------------------------------------------------------
    // 1. Get the measurement's standard deviations.
    // --------------------------------------------------------

    const float rangeSigma =
        std::max(0.000001f, measurement.rangeSigmaNM);

    const float azimuthSigma =
        std::max(0.000001f, measurement.azimuthSigmaRadians);

    const float elevationSigma =
        std::max(0.000001f, measurement.elevationSigmaRadians);

    const float radialVelocitySigma =
        std::max(0.001f, measurement.radialVelocitySigmaKnots)
        / NM_PER_SEC_TO_KNOTS;


    // --------------------------------------------------------
    // 2. Transform polar position uncertainty into Cartesian
    //    position uncertainty.
    // --------------------------------------------------------

    const float r = measurement.rangeNM;
    const float az = measurement.azimuthRadians;
    const float el = measurement.elevationRadians;

    const float G[3][3] =
    {
        {
            std::cos(el) * std::cos(az),
            -r * std::cos(el) * std::sin(az),
            -r * std::sin(el) * std::cos(az)
        },
        {
            std::cos(el) * std::sin(az),
            r * std::cos(el) * std::cos(az),
            -r * std::sin(el) * std::sin(az)
        },
        {
            std::sin(el),
            0.0f,
            r * std::cos(el)
        }
    };

    // These are variances, not standard deviations.
    // R = sigma squared.
    const float polarVariance[3] =
    {
        rangeSigma * rangeSigma,
        azimuthSigma * azimuthSigma,
        elevationSigma * elevationSigma
    };

    // Calculate P_position = G * R * G^T.

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            float variance = 0.0f;

            for (int k = 0; k < 3; ++k)
            {
                variance +=
                    G[row][k] *
                    polarVariance[k] *
                    G[col][k];
            }

            state.P[Index6(row, col)] = variance;
        }

        // Prevent an exactly zero diagonal variance.
        state.P[Index6(row, row)] =
            std::max(1e-10f, state.P[Index6(row, row)]);
    }


    // --------------------------------------------------------
    // 3. Initialize velocity uncertainty.
    // --------------------------------------------------------

    const float u[3] =
    {
        losX,
        losY,
        losZ
    };

    constexpr float TANGENTIAL_VELOCITY_SIGMA_NM_S = 0.25f;

    const float tangentialVariance =
        TANGENTIAL_VELOCITY_SIGMA_NM_S *
        TANGENTIAL_VELOCITY_SIGMA_NM_S;

    const float radialVariance =
        radialVelocitySigma * radialVelocitySigma;

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            const float identity =
                row == col ? 1.0f : 0.0f;

            state.P[Index6(row + 3, col + 3)] =
                radialVariance * u[row] * u[col] +
                tangentialVariance *
                    (identity - u[row] * u[col]);
        }
    }

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
    const Kinematics& observerKinematics)
{
    RadarInnovation innovation{};

    if (!CalculateInnovation(
        state,
        measurement,
        observer,
        observerKinematics,
        innovation))
    {
        return false;
    }

    // --------------------------------------------------------
    // Calculate Kalman gain:
    // K = P * H^T * inverse(S).
    // --------------------------------------------------------

    Matrix6x4 K{};

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            float value = 0.0f;

            for (int k = 0; k < 4; ++k)
            {
                value +=
                    innovation.PHt[Index6x4(row, k)] *
                    innovation.inverseCovariance[Index4(k, col)];
            }

            K[Index6x4(row, col)] = value;
        }
    }

    // --------------------------------------------------------
    // Calculate updated state without modifying the real state.
    // --------------------------------------------------------

    std::array<float, 6> newX = state.x;

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            newX[row] +=
                K[Index6x4(row, col)] *
                innovation.residual[col];
        }

        if (!std::isfinite(newX[row]))
            return false;
    }

    // --------------------------------------------------------
    // Joseph covariance update:
    // P = (I - KH)P(I - KH)^T + KRK^T.
    // --------------------------------------------------------

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
                    innovation.H[Index4x6(k, col)];
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

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            for (int k = 0; k < 4; ++k)
            {
                newP[Index6(row, col)] +=
                    K[Index6x4(row, k)] *
                    innovation.measurementVariance[k] *
                    K[Index6x4(col, k)];
            }
        }
    }

    // Restore numerical symmetry.
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

        if (!std::isfinite(newP[Index6(row, row)]) ||
            newP[Index6(row, row)] <= 0.0f)
        {
            return false;
        }

        newP[Index6(row, row)] =
            std::max(1e-10f, newP[Index6(row, row)]);
    }

    // --------------------------------------------------------
    // Commit only after every calculation succeeded.
    // --------------------------------------------------------

    state.x = newX;
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
    track.hasVelocity = track.kalman.measurementCount >= 2;

    track.verticalVelocityNMPerSec = track.kalman.x[5];
}