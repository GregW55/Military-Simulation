#pragma once

#include "../ecs/components.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace TrackFusion
{
    constexpr int STATE_SIZE = 6;

    using Matrix6 = std::array<float, 36>;
    using Vector6 = std::array<float, 6>;

    constexpr int Index(int row, int col)
    {
        return row * STATE_SIZE + col;
    }


    // --------------------------------------------------------
    // General matrix inversion for a 3x3 or 6x6 matrix.
    // --------------------------------------------------------

    inline bool Invert(
        const Matrix6& input,
        Matrix6& inverse,
        int dimensions)
    {
        if (dimensions != 3 && dimensions != 6)
            return false;

        double augmented[6][12]{};

        for (int row = 0; row < dimensions; ++row)
        {
            for (int col = 0; col < dimensions; ++col)
            {
                const float value =
                    input[Index(row, col)];

                if (!std::isfinite(value))
                    return false;

                augmented[row][col] = value;
            }

            augmented[row][row + dimensions] = 1.0;
        }

        for (int col = 0; col < dimensions; ++col)
        {
            int pivotRow = col;

            for (int row = col + 1; row < dimensions; ++row)
            {
                if (std::abs(augmented[row][col]) >
                    std::abs(augmented[pivotRow][col]))
                {
                    pivotRow = row;
                }
            }

            if (std::abs(augmented[pivotRow][col]) < 1e-12)
                return false;

            if (pivotRow != col)
            {
                for (int k = 0; k < dimensions * 2; ++k)
                {
                    std::swap(
                        augmented[col][k],
                        augmented[pivotRow][k]
                    );
                }
            }

            const double pivot =
                augmented[col][col];

            for (int k = 0; k < dimensions * 2; ++k)
                augmented[col][k] /= pivot;

            for (int row = 0; row < dimensions; ++row)
            {
                if (row == col)
                    continue;

                const double factor =
                    augmented[row][col];

                for (int k = 0; k < dimensions * 2; ++k)
                {
                    augmented[row][k] -=
                        factor * augmented[col][k];
                }
            }
        }

        inverse = {};

        for (int row = 0; row < dimensions; ++row)
        {
            for (int col = 0; col < dimensions; ++col)
            {
                const double value =
                    augmented[row][col + dimensions];

                if (!std::isfinite(value))
                    return false;

                inverse[Index(row, col)] =
                    static_cast<float>(value);
            }
        }

        return true;
    }


    // --------------------------------------------------------
    // Compare two local/global track estimates.
    //
    // When both tracks have velocity estimates:
    //     use x,y,z,vx,vy,vz (6 dimensions).
    //
    // Otherwise:
    //     use x,y,z (3 dimensions).
    //
    // The covariance sum assumes independent estimates.
    // --------------------------------------------------------

    inline bool CalculateTrackDistance(
        const RadarTrack& a,
        const RadarTrack& b,
        float& mahalanobisSquared)
    {
        mahalanobisSquared = 0.0f;

        if (!a.kalman.initialized ||
            !b.kalman.initialized)
        {
            return false;
        }

        constexpr float TIMESTAMP_TOLERANCE_SEC = 0.01f;

        if (std::abs(
            a.kalman.timestampSec -
            b.kalman.timestampSec
        ) > TIMESTAMP_TOLERANCE_SEC)
        {
            return false;
        }

        const int dimensions =
            (a.hasVelocity && b.hasVelocity) ? 6 : 3;

        Matrix6 S{};

        Vector6 difference{};

        for (int i = 0; i < dimensions; ++i)
        {
            difference[i] =
                a.kalman.x[i] -
                b.kalman.x[i];

            for (int j = 0; j < dimensions; ++j)
            {
                S[Index(i, j)] =
                    a.kalman.P[Index(i, j)] +
                    b.kalman.P[Index(i, j)];
            }

            // Small numerical regularization.
            S[Index(i, i)] += 1e-8f;
        }

        Matrix6 inverseS{};

        if (!Invert(S, inverseS, dimensions))
            return false;

        double score = 0.0;

        for (int row = 0; row < dimensions; ++row)
        {
            for (int col = 0; col < dimensions; ++col)
            {
                score +=
                    difference[row] *
                    inverseS[Index(row, col)] *
                    difference[col];
            }
        }

        if (!std::isfinite(score) || score < -1e-6)
            return false;

        mahalanobisSquared =
            static_cast<float>(std::max(0.0, score));

        return true;
    }


    // --------------------------------------------------------
    // Covariance Intersection.
    //
    // Fuses two state estimates without assuming their
    // estimation errors are statistically independent.
    // --------------------------------------------------------

    inline bool CovarianceIntersection(
        const KalmanState3D& a,
        const KalmanState3D& b,
        KalmanState3D& result)
    {
        if (!a.initialized || !b.initialized)
            return false;

        constexpr float TIMESTAMP_TOLERANCE_SEC = 0.01f;

        if (std::abs(a.timestampSec - b.timestampSec) >
            TIMESTAMP_TOLERANCE_SEC)
        {
            return false;
        }

        Matrix6 inverseA{};
        Matrix6 inverseB{};

        if (!Invert(a.P, inverseA, 6) ||
            !Invert(b.P, inverseB, 6))
        {
            return false;
        }

        float bestTrace =
            std::numeric_limits<float>::infinity();

        Matrix6 bestP{};
        Vector6 bestX{};

        // Search for a weight minimizing covariance trace.
        // A finer search can be added after testing.
        for (int step = 0; step <= 100; ++step)
        {
            const float weight =
                static_cast<float>(step) / 100.0f;

            Matrix6 information{};

            for (int row = 0; row < 6; ++row)
            {
                for (int col = 0; col < 6; ++col)
                {
                    information[Index(row, col)] =
                        weight * inverseA[Index(row, col)] +
                        (1.0f - weight) * inverseB[Index(row, col)];
                }
            }

            Matrix6 fusedP{};

            if (!Invert(information, fusedP, 6))
                continue;

            float trace = 0.0f;

            for (int i = 0; i < 6; ++i)
                trace += fusedP[Index(i, i)];

            if (!std::isfinite(trace) || trace >= bestTrace)
                continue;

            // Weighted information vector.
            Vector6 informationVector{};

            for (int row = 0; row < 6; ++row)
            {
                float infoA = 0.0f;
                float infoB = 0.0f;

                for (int col = 0; col < 6; ++col)
                {
                    infoA +=
                        inverseA[Index(row, col)] * a.x[col];

                    infoB +=
                        inverseB[Index(row, col)] * b.x[col];
                }

                informationVector[row] =
                    weight * infoA +
                    (1.0f - weight) * infoB;
            }

            Vector6 fusedX{};

            for (int row = 0; row < 6; ++row)
            {
                for (int col = 0; col < 6; ++col)
                {
                    fusedX[row] +=
                        fusedP[Index(row, col)] *
                        informationVector[col];
                }

                if (!std::isfinite(fusedX[row]))
                    return false;
            }

            bestTrace = trace;
            bestP = fusedP;
            bestX = fusedX;
        }

        if (!std::isfinite(bestTrace))
            return false;

        // Commit the result.
        result = a;

        result.x = bestX;
        result.P = bestP;

        result.timestampSec =
            std::max(a.timestampSec, b.timestampSec);

        result.measurementCount =
            std::max(a.measurementCount, b.measurementCount);

        result.initialized = true;

        result.processNoiseAccelPSD =
            std::max(
                a.processNoiseAccelPSD,
                b.processNoiseAccelPSD
            );

        return true;
    }
}