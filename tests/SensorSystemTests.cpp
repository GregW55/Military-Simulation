
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>
#include <algorithm>

#include "ecs/Systems.h"
#include "ecs/components.h"
#include "core/MapProjection.h"
#include "core/Constants.h"

class SensorSystemTests : public ::testing::Test
{
protected:
    entt::registry registry;
    entt::entity observer = entt::null;

    static constexpr float ALTITUDE = 10000.0f;

    void SetUp() override
    {
        registry.ctx().emplace<MapProjection>();
        registry.ctx().emplace<RadarDetectionData>();
        registry.ctx().emplace<std::mt19937>(12345);
        registry.ctx().emplace<float>(100.0f);
    }

    entt::entity CreateObserver(
        MathUtils::Vec2 position = {0.0f, 0.0f},
        bool hostile = false)
    {
        auto entity = registry.create();

        registry.emplace<Transform2D>(
            entity, Transform2D{position, ALTITUDE, 0.0f});

        registry.emplace<Kinematics>(entity);

        registry.emplace<RadarEmitter>(entity);
        auto& radar = registry.get<RadarEmitter>(entity);

        radar.maxRangeNM = 100.0f;
        radar.maxRangeNmSq = 10000.0f;
        radar.scanRateSec = 1.0f;
        radar.timeSinceLastScan = 1.0f;
        radar.rangeErrorNM = 0.000001f;
        radar.angleErrorRadians = 0.000001f;
        radar.velocityErrorKnots = 0.000001f;

        registry.emplace<RadarSignature>(
            entity, RadarSignature{1.0f, 1.0f});

        registry.emplace<IFF>(entity, IFF{hostile});

        return entity;
    }

    entt::entity CreateTarget(
        MathUtils::Vec2 position,
        float altitude = ALTITUDE,
        bool hostile = true,
        float rcsFourthRoot = 1.0f,
        MathUtils::Vec2 velocity = {0.0f, 0.0f})
    {
        auto entity = registry.create();

        registry.emplace<Transform2D>(
            entity, Transform2D{position, altitude, 0.0f});

        registry.emplace<Kinematics>(entity);
        registry.get<Kinematics>(entity).velocity = velocity;

        registry.emplace<RadarSignature>(
            entity, RadarSignature{1.0f, rcsFourthRoot});

        registry.emplace<IFF>(entity, IFF{hostile});

        return entity;
    }

    RadarDetectionData& Data()
    {
        return registry.ctx().get<RadarDetectionData>();
    }

    RadarEmitter& Radar()
    {
        return registry.get<RadarEmitter>(observer);
    }

    void Scan(float dt = 0.0f)
    {
        Systems::SensorSystem(registry, dt);
    }

    const std::vector<RadarMeasurement>& Measurements() const
    {
        return registry.ctx()
            .get<RadarDetectionData>()
            .frameMeasurements.at(observer);
    }

    void ForceNextScan()
    {
        Radar().timeSinceLastScan = Radar().scanRateSec;
    }
};

// ============================================================
// SCAN SCHEDULING
// ============================================================

TEST_F(SensorSystemTests, DoesNotScanBeforeInterval)
{
    observer = CreateObserver();
    Radar().timeSinceLastScan = 0.0f;

    Scan(0.5f);

    EXPECT_FALSE(Data().frameMeasurements.contains(observer));
}

TEST_F(SensorSystemTests, ScansWhenIntervalIsReached)
{
    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f});

    Radar().timeSinceLastScan = 0.5f;
    Scan(0.5f);

    ASSERT_TRUE(Data().frameMeasurements.contains(observer));
    EXPECT_EQ(Measurements().size(), 1);
}

TEST_F(SensorSystemTests, ScanTimerResetsAfterScan)
{
    observer = CreateObserver();

    Radar().timeSinceLastScan = 1.0f;
    Scan();

    EXPECT_NEAR(Radar().timeSinceLastScan, 0.0f, 0.0001f);
}

TEST_F(SensorSystemTests, PreservesRemainingScanTimer)
{
    observer = CreateObserver();

    Radar().timeSinceLastScan = 1.25f;
    Scan();

    EXPECT_NEAR(Radar().timeSinceLastScan, 0.25f, 0.0001f);
}

TEST_F(SensorSystemTests, ClearsPreviousFrameMeasurements)
{
    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f});

    Scan();
    ASSERT_EQ(Measurements().size(), 1);

    Scan(0.1f);

    EXPECT_TRUE(Data().frameMeasurements.empty());
}

// ============================================================
// ENTITY FILTERING
// ============================================================

TEST_F(SensorSystemTests, DetectsHostileTarget)
{
    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f}, ALTITUDE, true);

    Scan();

    ASSERT_TRUE(Data().frameMeasurements.contains(observer));
    EXPECT_EQ(Measurements().size(), 1);
}

TEST_F(SensorSystemTests, IgnoresFriendlyTarget)
{
    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f}, ALTITUDE, false);

    Scan();

    ASSERT_TRUE(Data().frameMeasurements.contains(observer));
    EXPECT_TRUE(Measurements().empty());
}

TEST_F(SensorSystemTests, DoesNotDetectItself)
{
    observer = CreateObserver();

    Scan();

    ASSERT_TRUE(Data().frameMeasurements.contains(observer));
    EXPECT_TRUE(Measurements().empty());
}

TEST_F(SensorSystemTests, DetectsMultipleHostileTargets)
{
    observer = CreateObserver();

    CreateTarget({10.0f, 0.0f});
    CreateTarget({20.0f, 0.0f});
    CreateTarget({30.0f, 0.0f});

    Scan();

    EXPECT_EQ(Measurements().size(), 3);
}

TEST_F(SensorSystemTests, FiltersMixedFriendlyAndHostileTargets)
{
    observer = CreateObserver();

    CreateTarget({10.0f, 0.0f}, ALTITUDE, true);
    CreateTarget({20.0f, 0.0f}, ALTITUDE, false);
    CreateTarget({30.0f, 0.0f}, ALTITUDE, true);
    CreateTarget({40.0f, 0.0f}, ALTITUDE, false);

    Scan();

    EXPECT_EQ(Measurements().size(), 2);
}

TEST_F(SensorSystemTests, IgnoresTargetWithoutRadarSignature)
{
    observer = CreateObserver();

    auto target = registry.create();
    registry.emplace<Transform2D>(
        target, Transform2D{{10.0f, 0.0f}, ALTITUDE, 0.0f});
    registry.emplace<Kinematics>(target);
    registry.emplace<IFF>(target, IFF{true});

    Scan();

    EXPECT_TRUE(Measurements().empty());
}

TEST_F(SensorSystemTests, IgnoresTargetWithoutIFF)
{
    observer = CreateObserver();

    auto target = registry.create();
    registry.emplace<Transform2D>(
        target, Transform2D{{10.0f, 0.0f}, ALTITUDE, 0.0f});
    registry.emplace<Kinematics>(target);
    registry.emplace<RadarSignature>(
        target, RadarSignature{1.0f, 1.0f});

    Scan();

    EXPECT_TRUE(Measurements().empty());
}

TEST_F(SensorSystemTests, IgnoresTargetWithoutKinematics)
{
    observer = CreateObserver();

    auto target = registry.create();
    registry.emplace<Transform2D>(
        target, Transform2D{{10.0f, 0.0f}, ALTITUDE, 0.0f});
    registry.emplace<RadarSignature>(
        target, RadarSignature{1.0f, 1.0f});
    registry.emplace<IFF>(target, IFF{true});

    Scan();

    EXPECT_TRUE(Measurements().empty());
}

TEST_F(SensorSystemTests, IgnoresTargetWithoutTransform)
{
    observer = CreateObserver();

    auto target = registry.create();
    registry.emplace<Kinematics>(target);
    registry.emplace<RadarSignature>(
        target, RadarSignature{1.0f, 1.0f});
    registry.emplace<IFF>(target, IFF{true});

    Scan();

    EXPECT_TRUE(Measurements().empty());
}

TEST_F(SensorSystemTests, DoesNotProcessObserverWithoutRadarEmitter)
{
    auto entity = registry.create();
    registry.emplace<Transform2D>(
        entity, Transform2D{{0.0f, 0.0f}, ALTITUDE, 0.0f});
    registry.emplace<Kinematics>(entity);
    registry.emplace<IFF>(entity, IFF{false});

    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f});

    Scan();

    EXPECT_FALSE(Data().frameMeasurements.contains(entity));
}

// ============================================================
// DETECTION RANGE AND RCS
// ============================================================

TEST_F(SensorSystemTests, DetectsTargetInsideMaximumRange)
{
    observer = CreateObserver();
    CreateTarget({50.0f, 0.0f});

    Scan();

    EXPECT_EQ(Measurements().size(), 1);
}

TEST_F(SensorSystemTests, DetectsTargetAtMaximumRange)
{
    observer = CreateObserver();
    CreateTarget({100.0f, 0.0f});

    Scan();

    EXPECT_EQ(Measurements().size(), 1);
}

TEST_F(SensorSystemTests, RejectsTargetOutsideMaximumRange)
{
    observer = CreateObserver();
    CreateTarget({100.1f, 0.0f});

    Scan();

    EXPECT_TRUE(Measurements().empty());
}

TEST_F(SensorSystemTests, DetectsTargetInsideEffectiveRcsRange)
{
    observer = CreateObserver();

    CreateTarget({40.0f, 0.0f}, ALTITUDE, true, 0.5f);

    Scan();

    EXPECT_EQ(Measurements().size(), 1);
}

TEST_F(SensorSystemTests, RejectsTargetOutsideEffectiveRcsRange)
{
    observer = CreateObserver();

    CreateTarget({60.0f, 0.0f}, ALTITUDE, true, 0.5f);

    Scan();

    EXPECT_TRUE(Measurements().empty());
}

TEST_F(
    SensorSystemTests,
    LargeRcsCanExtendDetectionBeyondNominalRange)
{
    observer = CreateObserver();

    CreateTarget({100.1f, 0.0f}, ALTITUDE, true, 10.0f);

    Scan();

    ASSERT_EQ(Measurements().size(), 1u);

    EXPECT_NEAR(
        Measurements().front().rangeNM,
        100.1f,
        0.001f
    );
}

TEST_F(SensorSystemTests, RejectsTargetBeyondRadarHorizon)
{
    observer = CreateObserver();

    registry.get<Transform2D>(observer).altitude = 0.0f;

    CreateTarget({10.0f, 0.0f}, 0.0f);

    Scan();

    EXPECT_TRUE(Measurements().empty());
}

// ============================================================
// MEASUREMENT GEOMETRY
// ============================================================

TEST_F(SensorSystemTests, MeasuresSlantRangeAtEqualAltitude)
{
    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f});

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(Measurements()[0].rangeNM, 10.0f, 0.0001f);
}

TEST_F(SensorSystemTests, MeasuresSlantRangeCorrectly)
{
    observer = CreateObserver();

    CreateTarget({10.0f, 0.0f}, ALTITUDE + 1852.0f);

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].rangeNM,
        std::sqrt(101.0f),
        0.0001f
    );
}

TEST_F(SensorSystemTests, MeasuresAzimuthAlongPositiveXAxis)
{
    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f});

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(Measurements()[0].azimuthRadians, 0.0f, 0.0001f);
}

TEST_F(SensorSystemTests, MeasuresAzimuthAlongPositiveYAxis)
{
    observer = CreateObserver();
    CreateTarget({0.0f, 10.0f});

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].azimuthRadians,
        MATH_PI / 2.0f,
        0.0001f
    );
}

TEST_F(SensorSystemTests, MeasuresAzimuthAlongNegativeXAxis)
{
    observer = CreateObserver();
    CreateTarget({-10.0f, 0.0f});

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        std::abs(Measurements()[0].azimuthRadians),
        MATH_PI,
        0.0001f
    );
}

TEST_F(SensorSystemTests, MeasuresAzimuthAlongNegativeYAxis)
{
    observer = CreateObserver();
    CreateTarget({0.0f, -10.0f});

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].azimuthRadians,
        -MATH_PI / 2.0f,
        0.0001f
    );
}

TEST_F(SensorSystemTests, MeasuresElevationCorrectly)
{
    observer = CreateObserver();

    CreateTarget({10.0f, 0.0f}, ALTITUDE + 1852.0f);

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].elevationRadians,
        std::atan2(1.0f, 10.0f),
        0.0001f
    );
}

TEST_F(SensorSystemTests, MeasuresNegativeElevation)
{
    observer = CreateObserver();

    CreateTarget({10.0f, 0.0f}, ALTITUDE - 1852.0f);

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].elevationRadians,
        std::atan2(-1.0f, 10.0f),
        0.0001f
    );
}

TEST_F(SensorSystemTests, MeasurementsContainFiniteValues)
{
    observer = CreateObserver();

    CreateTarget({10.0f, 5.0f}, ALTITUDE + 500.0f);

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    const auto& m = Measurements()[0];

    EXPECT_TRUE(std::isfinite(m.rangeNM));
    EXPECT_TRUE(std::isfinite(m.azimuthRadians));
    EXPECT_TRUE(std::isfinite(m.elevationRadians));
    EXPECT_TRUE(std::isfinite(m.radialVelocityKnots));
    EXPECT_TRUE(std::isfinite(m.timestampSec));
}

// ============================================================
// RADIAL VELOCITY
// ============================================================

TEST_F(SensorSystemTests, MeasuresTargetMovingAway)
{
    observer = CreateObserver();

    CreateTarget(
        {10.0f, 0.0f},
        ALTITUDE,
        true,
        1.0f,
        {0.1f, 0.0f}
    );

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].radialVelocityKnots,
        360.0f,
        0.01f
    );
}

TEST_F(SensorSystemTests, MeasuresTargetMovingTowardObserver)
{
    observer = CreateObserver();

    CreateTarget(
        {10.0f, 0.0f},
        ALTITUDE,
        true,
        1.0f,
        {-0.1f, 0.0f}
    );

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].radialVelocityKnots,
        -360.0f,
        0.01f
    );
}

TEST_F(SensorSystemTests, TangentialMovementProducesZeroRadialVelocity)
{
    observer = CreateObserver();

    CreateTarget(
        {10.0f, 0.0f},
        ALTITUDE,
        true,
        1.0f,
        {0.0f, 0.1f}
    );

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].radialVelocityKnots,
        0.0f,
        0.01f
    );
}

TEST_F(SensorSystemTests, ObserverVelocityAffectsRadialVelocity)
{
    observer = CreateObserver();

    registry.get<Kinematics>(observer).velocity = {0.1f, 0.0f};

    CreateTarget(
        {10.0f, 0.0f},
        ALTITUDE,
        true,
        1.0f,
        {0.0f, 0.0f}
    );

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].radialVelocityKnots,
        -360.0f,
        0.01f
    );
}

TEST_F(SensorSystemTests, EqualObserverAndTargetVelocityProducesZeroRadialVelocity)
{
    observer = CreateObserver();

    registry.get<Kinematics>(observer).velocity = {0.1f, 0.0f};

    CreateTarget(
        {10.0f, 0.0f},
        ALTITUDE,
        true,
        1.0f,
        {0.1f, 0.0f}
    );

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].radialVelocityKnots,
        0.0f,
        0.01f
    );
}

TEST_F(SensorSystemTests, StationaryTargetAndObserverProduceZeroRadialVelocity)
{
    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f});

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(
        Measurements()[0].radialVelocityKnots,
        0.0f,
        0.01f
    );
}

TEST_F(SensorSystemTests, VerticalMovementContributesToRadialVelocity)
{
    observer = CreateObserver({0.0f, 0.0f});
    const auto target = CreateTarget(
        {10.0f, 0.0f},
        ALTITUDE + 1852.0f,
        true,
        1.0f,
        {0.0f, 0.0f});

    registry.get<Kinematics>(target).verticalVelocityNMPerSec = 0.1f;

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    const float expected =
        0.1f / std::sqrt(101.0f) * NM_PER_SEC_TO_KNOTS;

    EXPECT_NEAR(
        Measurements()[0].radialVelocityKnots,
        expected,
        0.01f);
}

TEST_F(SensorSystemTests, ObserverVerticalVelocityContributesToRelativeRadialVelocity)
{
    observer = CreateObserver({0.0f, 0.0f});

    // Target is 1 NM above the observer and has zero vertical velocity.
    // The observer climbs at 0.1 NM/sec, so the relative radial component
    // is negative and scaled by the vertical LOS component (1/sqrt(101)).
    registry.get<Kinematics>(observer).verticalVelocityNMPerSec = 0.1f;

    CreateTarget({10.0f, 0.0f}, ALTITUDE + 1852.0f);

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    const float expected =
        -0.1f / std::sqrt(101.0f) * NM_PER_SEC_TO_KNOTS;

    EXPECT_NEAR(
        Measurements()[0].radialVelocityKnots,
        expected,
        0.01f
    );
}

TEST_F(SensorSystemTests, MeasurementCarriesConfiguredUncertaintyValues)
{
    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f});

    Radar().rangeErrorNM = 0.025f;
    Radar().angleErrorRadians = 0.002f;
    Radar().velocityErrorKnots = 7.5f;

    Scan();

    ASSERT_EQ(Measurements().size(), 1);
    const RadarMeasurement& measurement = Measurements().front();

    EXPECT_FLOAT_EQ(measurement.rangeSigmaNM, 0.025f);
    EXPECT_FLOAT_EQ(measurement.azimuthSigmaRadians, 0.002f);
    EXPECT_FLOAT_EQ(measurement.elevationSigmaRadians, 0.002f);
    EXPECT_FLOAT_EQ(measurement.radialVelocitySigmaKnots, 7.5f);
}

// ============================================================
// TIMESTAMPS
// ============================================================

TEST_F(SensorSystemTests, MeasurementUsesCurrentSimulationTimestamp)
{
    observer = CreateObserver();
    CreateTarget({10.0f, 0.0f});

    registry.ctx().get<float>() = 250.0f;

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_FLOAT_EQ(
        Measurements()[0].timestampSec,
        250.0f
    );
}

// ============================================================
// MULTIPLE OBSERVERS
// ============================================================

TEST_F(SensorSystemTests, MultipleObserversDetectSameTarget)
{
    observer = CreateObserver({0.0f, 0.0f});

    const auto observer2 = CreateObserver({0.0f, 10.0f});

    CreateTarget({10.0f, 0.0f});

    Scan();

    ASSERT_TRUE(Data().frameMeasurements.contains(observer));
    ASSERT_TRUE(Data().frameMeasurements.contains(observer2));

    EXPECT_EQ(Data().frameMeasurements.at(observer).size(), 1);
    EXPECT_EQ(Data().frameMeasurements.at(observer2).size(), 1);
}

TEST_F(SensorSystemTests, ObserversMaintainIndependentScanTimers)
{
    observer = CreateObserver();

    const auto observer2 = CreateObserver();

    Radar().timeSinceLastScan = 1.0f;
    registry.get<RadarEmitter>(observer2).timeSinceLastScan = 0.0f;

    CreateTarget({10.0f, 0.0f});

    Scan();

    EXPECT_TRUE(Data().frameMeasurements.contains(observer));
    EXPECT_FALSE(Data().frameMeasurements.contains(observer2));
}

TEST_F(SensorSystemTests, ObserversProduceDifferentMeasurementsForDifferentPositions)
{
    observer = CreateObserver({0.0f, 0.0f});

    const auto observer2 = CreateObserver({0.0f, 10.0f});

    CreateTarget({10.0f, 0.0f});

    Scan();

    ASSERT_EQ(Data().frameMeasurements.at(observer).size(), 1);
    ASSERT_EQ(Data().frameMeasurements.at(observer2).size(), 1);

    const auto& m1 = Data().frameMeasurements.at(observer)[0];
    const auto& m2 = Data().frameMeasurements.at(observer2)[0];

    EXPECT_NE(m1.rangeNM, m2.rangeNM);
}

// ============================================================
// NOISE
// ============================================================

TEST_F(SensorSystemTests, RangeNoiseHasApproximatelyCorrectStandardDeviation)
{
    observer = CreateObserver();

    CreateTarget({10.0f, 0.0f});

    constexpr int SAMPLE_COUNT = 2000;
    constexpr float EXPECTED_STDDEV = 0.2f;

    Radar().rangeErrorNM = EXPECTED_STDDEV;

    std::vector<float> samples;
    samples.reserve(SAMPLE_COUNT);

    for (int i = 0; i < SAMPLE_COUNT; ++i)
    {
        ForceNextScan();
        Scan();

        ASSERT_EQ(Measurements().size(), 1);
        samples.push_back(Measurements()[0].rangeNM);
    }

    float mean = 0.0f;

    for (float value : samples)
        mean += value;

    mean /= static_cast<float>(samples.size());

    float variance = 0.0f;

    for (float value : samples)
        variance += (value - mean) * (value - mean);

    variance /= static_cast<float>(samples.size());

    const float standardDeviation = std::sqrt(variance);

    EXPECT_NEAR(mean, 10.0f, 0.03f);
    EXPECT_NEAR(standardDeviation, EXPECTED_STDDEV, 0.03f);
}

TEST_F(SensorSystemTests, ZeroNoiseProducesAccurateMeasurements)
{
    observer = CreateObserver();

    CreateTarget({10.0f, 0.0f});

    Radar().rangeErrorNM = 0.0f;
    Radar().angleErrorRadians = 0.0f;
    Radar().velocityErrorKnots = 0.0f;

    Scan();

    ASSERT_EQ(Measurements().size(), 1);

    EXPECT_NEAR(Measurements()[0].rangeNM, 10.0f, 0.0001f);
    EXPECT_NEAR(Measurements()[0].azimuthRadians, 0.0f, 0.0001f);
    EXPECT_NEAR(Measurements()[0].elevationRadians, 0.0f, 0.0001f);
    EXPECT_NEAR(Measurements()[0].radialVelocityKnots, 0.0f, 0.0001f);
}

// ============================================================
// EMPTY / GENERAL EDGE CASES
// ============================================================

TEST_F(SensorSystemTests, NoObserversDoesNotCrash)
{
    EXPECT_NO_THROW(Scan());
    EXPECT_TRUE(Data().frameMeasurements.empty());
}

TEST_F(SensorSystemTests, ObserverWithNoTargetsProducesEmptyMeasurements)
{
    observer = CreateObserver();

    Scan();

    ASSERT_TRUE(Data().frameMeasurements.contains(observer));
    EXPECT_TRUE(Measurements().empty());
}

TEST_F(SensorSystemTests, MultipleTargetsProduceFiniteMeasurements)
{
    observer = CreateObserver();

    CreateTarget({10.0f, 0.0f});
    CreateTarget({0.0f, 20.0f});
    CreateTarget({-30.0f, 10.0f});

    Scan();

    ASSERT_EQ(Measurements().size(), 3);

    for (const auto& m : Measurements())
    {
        EXPECT_TRUE(std::isfinite(m.rangeNM));
        EXPECT_TRUE(std::isfinite(m.azimuthRadians));
        EXPECT_TRUE(std::isfinite(m.elevationRadians));
        EXPECT_TRUE(std::isfinite(m.radialVelocityKnots));
    }
}
