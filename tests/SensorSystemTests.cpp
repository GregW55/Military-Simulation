#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "core/GeoEngine.h"
#include "ecs/Systems.h"
#include "ecs/components.h"
#include "RadarTestHelpers.h"

namespace {

class SensorSystemTests : public ::testing::Test {
protected:
    entt::registry registry;

    void SetUp() override
    {
        registry.ctx().emplace<MapProjection>();
        registry.ctx().emplace<RadarDetectionData>();
    }

    MapProjection& Map()
    {
        return registry.ctx().get<MapProjection>();
    }

    RadarDetectionData& DetectionData()
    {
        return registry.ctx().get<RadarDetectionData>();
    }

    entt::entity AddObserver(
        MathUtils::Vec2 position = {0.0f, -30.0f},
        bool hostile = false,
        float rangeNM = 10.0f,
        float scanRateSec = 0.5f,
        float timerSec = 0.5f,
        float altitudeM = radar_test::kObserverAltitudeM)
    {
        const entt::entity observer = registry.create();
        registry.emplace<Transform2D>(observer, position, altitudeM, 0.0f);
        registry.emplace<RadarEmitter>(
            observer, rangeNM, rangeNM * rangeNM, scanRateSec, timerSec);
        registry.emplace<IFF>(observer, hostile);
        return observer;
    }

    entt::entity AddTarget(
        MathUtils::Vec2 position,
        bool hostile,
        float rcsFourthRoot = 1.0f,
        float altitudeM = radar_test::kObserverAltitudeM)
    {
        const entt::entity target = registry.create();
        registry.emplace<Transform2D>(target, position, altitudeM, 0.0f);

        const float rcs = rcsFourthRoot * rcsFourthRoot *
                          rcsFourthRoot * rcsFourthRoot;
        registry.emplace<RadarSignature>(target, rcs, rcsFourthRoot);
        registry.emplace<IFF>(target, hostile);
        return target;
    }
};

TEST_F(SensorSystemTests, DetectsOnlyHostileTargetsInsideEffectiveRange)
{
    const entt::entity observer = AddObserver();
    AddTarget({5.0f, -30.0f}, true);
    AddTarget({4.0f, -30.0f}, false);
    AddTarget({12.0f, -30.0f}, true);

    Systems::SensorSystem(registry, 0.02f);

    const auto& data = DetectionData();
    ASSERT_TRUE(data.framePings.contains(observer));
    const auto& pings = data.framePings.at(observer);
    ASSERT_EQ(pings.size(), 1u);
    EXPECT_NEAR(pings.front().pos.x, 5.0f, radar_test::kEpsilon);
    EXPECT_NEAR(pings.front().pos.y, -30.0f, radar_test::kEpsilon);
}

TEST_F(SensorSystemTests, DoesNotScanBeforeTheScanIntervalHasElapsed)
{
    const entt::entity observer = AddObserver(
        {0.0f, -30.0f}, false, 10.0f, 0.5f, 0.0f);
    AddTarget({5.0f, -30.0f}, true);

    Systems::SensorSystem(registry, 0.49f);

    EXPECT_FALSE(DetectionData().framePings.contains(observer));
}

TEST_F(SensorSystemTests, EmitsAnEmptyPingListWhenAScanFindsNothing)
{
    const entt::entity observer = AddObserver();

    Systems::SensorSystem(registry, 0.02f);

    ASSERT_TRUE(DetectionData().framePings.contains(observer));
    EXPECT_TRUE(DetectionData().framePings.at(observer).empty());
}

TEST_F(SensorSystemTests, AppliesRcsAdjustedDetectionRange)
{
    const entt::entity observer = AddObserver();
    // RCS fourth root .25 scales the nominal 10 NM range to 2.5 NM.
    AddTarget({2.0f, -30.0f}, true, 0.25f);
    AddTarget({3.0f, -30.0f}, true, 0.25f);

    Systems::SensorSystem(registry, 0.02f);

    ASSERT_TRUE(DetectionData().framePings.contains(observer));
    const auto& pings = DetectionData().framePings.at(observer);
    ASSERT_EQ(pings.size(), 1u);
    EXPECT_NEAR(pings.front().pos.x, 2.0f, radar_test::kEpsilon);
}

TEST_F(SensorSystemTests, TerrainCanBlockAnOtherwiseInRangeTarget)
{
    constexpr int width = 300;
    constexpr int height = 300;
    std::vector<uint8_t> elevations(
        static_cast<std::size_t>(width * height), 0);

    // Place a 4,000 m ridge across the observer-target path.
    for (int x = 10; x <= 25; ++x) {
        elevations[static_cast<std::size_t>(199 * width + x)] = 255;
    }

    Map().SetHeightMap(
        std::move(elevations), width, height, -528.0f, -930.0f, 1.0f);

    const entt::entity observer = AddObserver({0.0f, -30.0f});
    AddTarget({5.0f, -30.0f}, true);

    Systems::SensorSystem(registry, 0.02f);

    ASSERT_TRUE(DetectionData().framePings.contains(observer));
    EXPECT_TRUE(DetectionData().framePings.at(observer).empty());
}

    TEST_F(SensorSystemTests, PreservesTheExcessScanTimerInsteadOfDiscardingIt)
{
    const entt::entity observer = AddObserver(
        {0.0f, -30.0f}, false, 10.0f, 0.5f, 0.0f);

    Systems::SensorSystem(registry, 0.3f);
    EXPECT_FALSE(DetectionData().framePings.contains(observer));

    Systems::SensorSystem(registry, 0.3f);

    ASSERT_TRUE(DetectionData().framePings.contains(observer));
    EXPECT_NEAR(
        registry.get<RadarEmitter>(observer).timeSinceLastScan,
        0.1f,
        radar_test::kEpsilon);
}

} // namespace
