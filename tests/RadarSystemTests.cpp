#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "core/Constants.h"
#include "core/GeoEngine.h"
#include "ecs/Systems.h"
#include "ecs/components.h"
#include "utils/TrackCorrelation.h"

namespace {

    constexpr float kEpsilon = 1e-3f;
    constexpr float kObserverAltitudeM = 30.0f;

    RadarTrack MakePing(MathUtils::Vec2 position,
                        float altitude = kObserverAltitudeM)
    {
        RadarTrack ping{};
        ping.pos = position;
        ping.altitude = altitude;
        return ping;
    }

    RadarTrack MakeLocalTrack(const MathUtils::Vec2 position,
                              const ThreatClass classification = ThreatClass::UNKNOWN,
                              const bool hasVelocity = true,
                              const uint32_t globalId = 0,
                              const float ageSec = 0.0f,
                              const bool missing = false)
    {
        RadarTrack track{};
        track.globalTrackId = globalId;
        track.pos = position;
        track.latestPing = position;
        track.altitude = kObserverAltitudeM;
        track.ageSec = ageSec;
        track.isMissing = missing;
        track.classification = classification;
        track.hasVelocity = hasVelocity;
        return track;
    }

    RadarTrack MakeCorrelationTrack(
        MathUtils::Vec2 latestPing,
        MathUtils::Vec2 velocity = {0.0f, 0.0f},
        float ageSec = 1.0f,
        ThreatClass classification = ThreatClass::UNKNOWN,
        bool hasVelocity = false)
    {
        RadarTrack track{};

        track.latestPing = latestPing;
        track.pos = latestPing;
        track.vel = velocity;
        track.ageSec = ageSec;
        track.classification = classification;
        track.hasVelocity = hasVelocity;

        track.speedKnots =
            MathUtils::NmPerSecToKnots(MathUtils::Length(velocity));

        track.isMissing = false;

        return track;
    }

class RadarSystemsTest : public ::testing::Test {
protected:
    entt::registry registry;

    void SetUp() override
    {
        registry.ctx().emplace<MapProjection>();
        registry.ctx().emplace<RadarDetectionData>();
        registry.ctx().emplace<MasterTrackTable>();
    }

    MapProjection& Map()
    {
        return registry.ctx().get<MapProjection>();
    }

    RadarDetectionData& DetectionData()
    {
        return registry.ctx().get<RadarDetectionData>();
    }

    MasterTrackTable& MasterTable()
    {
        return registry.ctx().get<MasterTrackTable>();
    }

    entt::entity AddObserver(
        MathUtils::Vec2 position = {0.0f, -30.0f},
        bool hostile = false,
        float rangeNM = 10.0f,
        float scanRateSec = 0.5f,
        float timerSec = 0.5f,
        float altitudeM = kObserverAltitudeM)
    {
        const entt::entity observer = registry.create();
        registry.emplace<Transform2D>(observer, position, altitudeM, 0.0f);
        registry.emplace<RadarEmitter>(observer, rangeNM, rangeNM * rangeNM,
                                      scanRateSec, timerSec);
        registry.emplace<IFF>(observer, hostile);
        return observer;
    }

    entt::entity AddTarget(
        MathUtils::Vec2 position,
        bool hostile,
        float rcsFourthRoot = 1.0f,
        float altitudeM = kObserverAltitudeM)
    {
        const entt::entity target = registry.create();
        registry.emplace<Transform2D>(target, position, altitudeM, 0.0f);
        const float rcs = rcsFourthRoot * rcsFourthRoot *
                          rcsFourthRoot * rcsFourthRoot;
        registry.emplace<RadarSignature>(target, rcs, rcsFourthRoot);
        registry.emplace<IFF>(target, hostile);
        return target;
    }

    void SubmitPings(entt::entity observer, std::vector<RadarTrack> pings)
    {
        auto& data = DetectionData();
        data.framePings.clear();
        data.framePings.emplace(observer, std::move(pings));
    }

    std::vector<RadarTrack>& LocalTracks(entt::entity observer)
    {
        return DetectionData().activeTracks[observer];
    }
};

// -----------------------------------------------------------------------------
// SensorSystem tests
// -----------------------------------------------------------------------------

    TEST_F(RadarSystemsTest, DetectsOnlyHostileTargetsInsideEffectiveRange)
    {
        const entt::entity observer = AddObserver();
        AddTarget({5.0f, -30.0f}, true);   // Enemy, in range.
        AddTarget({4.0f, -30.0f}, false);  // Friendly, in range.
        AddTarget({12.0f, -30.0f}, true);  // Enemy, outside a 10 NM radar.

        Systems::SensorSystem(registry, 0.02f); // 0.49 + 0.02 >= 0.5.

        const auto& data = DetectionData();
        ASSERT_TRUE(data.framePings.contains(observer));
        const auto& pings = data.framePings.at(observer);
        ASSERT_EQ(pings.size(), 1u);
        EXPECT_NEAR(pings.front().pos.x, 5.0f, kEpsilon);
        EXPECT_NEAR(pings.front().pos.y, -30.0f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, DoesNotScanBeforeTheScanIntervalHasElapsed)
    {
        const entt::entity observer = AddObserver(
            {0.0f, -30.0f}, false, 10.0f, 0.5f, 0.0f);
        AddTarget({5.0f, -30.0f}, true);

        Systems::SensorSystem(registry, 0.49f);

        EXPECT_FALSE(DetectionData().framePings.contains(observer));
    }

    TEST_F(RadarSystemsTest, EmitsAnEmptyPingListWhenAScanFindsNothing)
    {
        const entt::entity observer = AddObserver();

        Systems::SensorSystem(registry, 0.02f);

        ASSERT_TRUE(DetectionData().framePings.contains(observer));
        EXPECT_TRUE(DetectionData().framePings.at(observer).empty());
    }

    TEST_F(RadarSystemsTest, AppliesRcsAdjustedDetectionRange)
    {
        const entt::entity observer = AddObserver();
        // RCS fourth root = 0.25, so effective range is 10 * .25 = 2.5 NM.
        AddTarget({2.0f, -30.0f}, true, 0.25f);
        AddTarget({3.0f, -30.0f}, true, 0.25f);

        Systems::SensorSystem(registry, 0.02f);

        ASSERT_TRUE(DetectionData().framePings.contains(observer));
        const auto& pings = DetectionData().framePings.at(observer);
        ASSERT_EQ(pings.size(), 1u);
        EXPECT_NEAR(pings.front().pos.x, 2.0f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, TerrainCanBlockAnOtherwiseInRangeTarget)
    {
        // The map transform makes world y = -30 NM land near heightmap row 199.
        // Put a 4,000 m ridge across the observer-target path.
        constexpr int width = 300;
        constexpr int height = 300;
        std::vector<uint8_t> elevations(static_cast<std::size_t>(width * height), 0);
        for (int x = 10; x <= 25; ++x) {
            elevations[static_cast<std::size_t>(199 * width + x)] = 255;
        }
        Map().SetHeightMap(std::move(elevations), width, height,
                           -528.0f, -930.0f, 1.0f);

        const entt::entity observer = AddObserver({0.0f, -30.0f});
        AddTarget({5.0f, -30.0f}, true);

        Systems::SensorSystem(registry, 0.02f);

        ASSERT_TRUE(DetectionData().framePings.contains(observer));
        EXPECT_TRUE(DetectionData().framePings.at(observer).empty());
    }

    TEST_F(RadarSystemsTest, PreservesTheExcessScanTimerInsteadOfDiscardingIt)
    {
        const entt::entity observer = AddObserver(
            {0.0f, -30.0f}, false, 10.0f, 0.5f, 0.0f);

        Systems::SensorSystem(registry, 0.3f); // Timer should now be 0.3.
        EXPECT_FALSE(DetectionData().framePings.contains(observer));

        Systems::SensorSystem(registry, 0.3f); // Scan at 0.6, leaving 0.1 seconds.

        ASSERT_TRUE(DetectionData().framePings.contains(observer));
        EXPECT_NEAR(registry.get<RadarEmitter>(observer).timeSinceLastScan,
                    0.1f, kEpsilon);
    }

    // -----------------------------------------------------------------------------
    // TrackingSystem tests
    // -----------------------------------------------------------------------------

    TEST_F(RadarSystemsTest, CreatesOneTrackFromTheFirstPing)
    {
        const entt::entity observer = AddObserver();
        SubmitPings(observer, {MakePing({0.0f, 10.0f})});

        Systems::TrackingSystem(registry, 0.1f);

        ASSERT_EQ(LocalTracks(observer).size(), 1u);
        const RadarTrack& track = LocalTracks(observer).front();
        EXPECT_FALSE(track.hasVelocity);
        EXPECT_FALSE(track.isMissing);
        EXPECT_NEAR(track.pos.x, 0.0f, kEpsilon);
        EXPECT_NEAR(track.pos.y, 10.0f, kEpsilon);
        EXPECT_NEAR(track.ageSec, 0.0f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, EstimatesVelocityAndKeepsOneTrackForLinearMotion)
    {
        const entt::entity observer = AddObserver();

        SubmitPings(observer, {MakePing({0.0f, 10.0f})});
        Systems::TrackingSystem(registry, 1.0f);

        SubmitPings(observer, {MakePing({0.005f, 10.0f})});
        Systems::TrackingSystem(registry, 1.0f);

        SubmitPings(observer, {MakePing({0.010f, 10.0f})});
        Systems::TrackingSystem(registry, 1.0f);

        ASSERT_EQ(LocalTracks(observer).size(), 1u)
            << "A continuing target must update its existing local track, not create a duplicate.";
        const RadarTrack& track = LocalTracks(observer).front();
        EXPECT_TRUE(track.hasVelocity);
        EXPECT_NEAR(track.vel.x, 0.005f, kEpsilon); // NM/s = 18 knots.
        EXPECT_NEAR(track.vel.y, 0.0f, kEpsilon);
        EXPECT_NEAR(track.speedKnots, 18.0f, 0.1f);
        EXPECT_NEAR(track.latestPing.x, 0.010f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, StationaryTargetDoesNotCreateDuplicateTracks)
    {
        const entt::entity observer = AddObserver();
        const MathUtils::Vec2 stationary{3.0f, 8.0f};

        SubmitPings(observer, {MakePing(stationary)});
        Systems::TrackingSystem(registry, 1.0f);

        SubmitPings(observer, {MakePing(stationary)});
        Systems::TrackingSystem(registry, 1.0f);

        ASSERT_EQ(LocalTracks(observer).size(), 1u);
        EXPECT_TRUE(LocalTracks(observer).front().hasVelocity);
        EXPECT_NEAR(MathUtils::Length(LocalTracks(observer).front().vel),
                    0.0f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, ExtrapolatesBetweenScansWithoutMarkingTrackMissing)
    {
        const entt::entity observer = AddObserver();

        SubmitPings(observer, {MakePing({0.0f, 10.0f})});
        Systems::TrackingSystem(registry, 1.0f);
        SubmitPings(observer, {MakePing({0.005f, 10.0f})});
        Systems::TrackingSystem(registry, 1.0f);

        // No scan occurred for this observer during the next interval.
        DetectionData().framePings.clear();
        Systems::TrackingSystem(registry, 1.0f);

        ASSERT_EQ(LocalTracks(observer).size(), 1u);
        const RadarTrack& track = LocalTracks(observer).front();
        EXPECT_FALSE(track.isMissing);
        EXPECT_NEAR(track.ageSec, 1.0f, kEpsilon);
        EXPECT_NEAR(track.pos.x, 0.010f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, MarksTrackMissingWhenAScanDoesNotDetectIt)
    {
        const entt::entity observer = AddObserver();
        SubmitPings(observer, {MakePing({2.0f, 10.0f})});
        Systems::TrackingSystem(registry, 0.1f);

        // An empty vector means that a scan happened but saw no target.
        SubmitPings(observer, {});
        Systems::TrackingSystem(registry, 1.0f);

        ASSERT_EQ(LocalTracks(observer).size(), 1u);
        EXPECT_TRUE(LocalTracks(observer).front().isMissing);
        EXPECT_NEAR(LocalTracks(observer).front().ageSec, 1.0f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, DeletesLocalTrackOnlyAfterTheDeletionTimeout)
    {
        const entt::entity observer = AddObserver();
        SubmitPings(observer, {MakePing({2.0f, 10.0f})});
        Systems::TrackingSystem(registry, 0.1f);

        SubmitPings(observer, {});
        Systems::TrackingSystem(registry, TRACK_DELETION_TIMEOUT_SEC);
        EXPECT_EQ(LocalTracks(observer).size(), 1u);

        SubmitPings(observer, {});
        Systems::TrackingSystem(registry, 0.02f);
        EXPECT_TRUE(LocalTracks(observer).empty());
    }

    TEST_F(RadarSystemsTest, KeepsTwoTargetsSeparateAsTheirPathsApproachAndCross)
    {
        const entt::entity observer = AddObserver();

        // Target A travels east at 0.4 NM/s; target B travels north at 0.3 NM/s.
        // Their paths cross spatially, but they do not occupy the same location at
        // the same sample time, so their identities remain distinguishable.
        const std::array<MathUtils::Vec2, 5> a{{
            {-1.0f, 0.0f}, {-0.6f, 0.0f}, {-0.2f, 0.0f},
            {0.2f, 0.0f}, {0.6f, 0.0f}}};
        const std::array<MathUtils::Vec2, 5> b{{
            {0.0f, -1.0f}, {0.0f, -0.7f}, {0.0f, -0.4f},
            {0.0f, -0.1f}, {0.0f, 0.2f}}};

        for (std::size_t i = 0; i < a.size(); ++i) {
            SubmitPings(observer, {MakePing(a[i]), MakePing(b[i])});
            Systems::TrackingSystem(registry, 1.0f);
            ASSERT_EQ(LocalTracks(observer).size(), 2u)
                << "Unexpected duplicate or lost track at sample " << i;
        }

        const auto& tracks = LocalTracks(observer);
        EXPECT_NEAR(tracks[0].vel.x, 0.4f, 0.02f);
        EXPECT_NEAR(tracks[0].vel.y, 0.0f, 0.02f);
        EXPECT_NEAR(tracks[1].vel.x, 0.0f, 0.02f);
        EXPECT_NEAR(tracks[1].vel.y, 0.3f, 0.02f);
    }

    TEST_F(RadarSystemsTest, RejectedImpossibleJumpMustNotBeReportedAsAValidUpdate)
    {
        const entt::entity observer = AddObserver();
        SubmitPings(observer, {MakePing({0.0f, 10.0f})});
        Systems::TrackingSystem(registry, 0.1f);

        // Mark the existing track missing, then provide a jump that falls inside
        // the broad unconfirmed-track gate but exceeds MAX_PLAUSIBLE_SPEED_KNOTS.
        SubmitPings(observer, {});
        Systems::TrackingSystem(registry, 0.01f);
        ASSERT_EQ(LocalTracks(observer).size(), 1u);
        ASSERT_TRUE(LocalTracks(observer).front().isMissing);

        LocalTracks(observer).front().classification = ThreatClass::SURFACE_SHIP;
        SubmitPings(observer, {MakePing({0.5f, 10.0f})});
        Systems::TrackingSystem(registry, 1.0f);

        ASSERT_EQ(LocalTracks(observer).size(), 2u);
        const RadarTrack& track = LocalTracks(observer).front();
        EXPECT_TRUE(track.isMissing)
            << "A measurement rejected by the speed limit must not clear missing status.";
        EXPECT_NEAR(track.latestPing.x, 0.0f, kEpsilon);
    }

    // -----------------------------------------------------------------------------
    // SensorFusionSystem tests
    // -----------------------------------------------------------------------------

    TEST_F(RadarSystemsTest, CreatesAGlobalTrackAndWritesItsIdBackToLocalTrack)
    {
        const entt::entity observer = registry.create();
        DetectionData().activeTracks[observer].push_back(
            MakeLocalTrack({10.0f, 20.0f}, ThreatClass::SURFACE_SHIP));


        Systems::SensorFusionSystem(registry, 1.0f / 60.0f);

        ASSERT_EQ(MasterTable().activeGlobalTracks.size(), 1u);
        const uint32_t id = DetectionData().activeTracks.at(observer).front().globalTrackId;
        EXPECT_NE(id, 0u);
        ASSERT_TRUE(MasterTable().activeGlobalTracks.contains(id));
        EXPECT_EQ(MasterTable().activeGlobalTracks.at(id).track.globalTrackId, id);
        EXPECT_NEAR(MasterTable().activeGlobalTracks.at(id).track.pos.x, 10.0f, kEpsilon);
        EXPECT_NEAR(MasterTable().activeGlobalTracks.at(id).track.pos.y, 20.0f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, TwoObserversReportingTheSameTargetShareOneGlobalId)
    {
        const entt::entity observerA = registry.create();
        const entt::entity observerB = registry.create();
        DetectionData().activeTracks[observerA].push_back(
            MakeLocalTrack({10.000f, 20.0f}, ThreatClass::SURFACE_SHIP));
        DetectionData().activeTracks[observerB].push_back(
            MakeLocalTrack({10.010f, 20.0f}, ThreatClass::SURFACE_SHIP)); // Within the initial 0.02 NM gate.

        Systems::SensorFusionSystem(registry, 1.0f / 60.0f);

        ASSERT_EQ(MasterTable().activeGlobalTracks.size(), 1u);
        const uint32_t idA = DetectionData().activeTracks.at(observerA).front().globalTrackId;
        const uint32_t idB = DetectionData().activeTracks.at(observerB).front().globalTrackId;
        EXPECT_NE(idA, 0u);
        EXPECT_EQ(idA, idB);
    }

    TEST_F(RadarSystemsTest, TwoObserversKeepOneGlobalIdForMovingTarget)
    {
        const entt::entity observerA = registry.create();
        const entt::entity observerB = registry.create();

        auto& tracksA = DetectionData().activeTracks[observerA];
        auto& tracksB = DetectionData().activeTracks[observerB];

        tracksA.push_back(
            MakeLocalTrack({10.000f, 20.0f}, ThreatClass::SURFACE_SHIP));

        tracksB.push_back(
            MakeLocalTrack({10.010f, 20.0f}, ThreatClass::SURFACE_SHIP));

        Systems::SensorFusionSystem(registry, 0.1f);

        ASSERT_EQ(MasterTable().activeGlobalTracks.size(), 1u);

        const uint32_t sharedIdA = tracksA.front().globalTrackId;
        const uint32_t sharedIdB = tracksB.front().globalTrackId;

        ASSERT_NE(sharedIdA, NO_GLOBAL_TRACK);
        ASSERT_EQ(sharedIdA, sharedIdB);

        // Move both local reports together over repeated fusion updates.
        for (int i = 1; i <= 20; ++i) {
            const MathUtils::Vec2 positionA{
                10.0f + 0.1f * i,
                20.0f
            };

            const MathUtils::Vec2 positionB{
                10.010f + 0.1f * i,
                20.0f
            };

            tracksA.front().pos = positionA;
            tracksA.front().latestPing = positionA;
            tracksA.front().ageSec = 0.0f;

            tracksB.front().pos = positionB;
            tracksB.front().latestPing = positionB;
            tracksB.front().ageSec = 0.0f;

            Systems::SensorFusionSystem(registry, 0.1f);

            ASSERT_EQ(MasterTable().activeGlobalTracks.size(), 1u)
                << "Global duplicate created at iteration " << i;

            EXPECT_EQ(tracksA.front().globalTrackId, sharedIdA)
                << "Observer A lost its global association at iteration " << i;

            EXPECT_EQ(tracksB.front().globalTrackId, sharedIdA)
                << "Observer B lost its global association at iteration " << i;
        }
    }

    TEST_F(RadarSystemsTest, TrackingSystemAssignsEachPingToOnlyOneTrack)
    {
        const entt::entity observer = AddObserver();

        auto& tracks = LocalTracks(observer);

        RadarTrack trackA = MakeCorrelationTrack(
            {0.0f, 10.0f},
            {0.0f, 0.0f},
            1.0f,
            ThreatClass::UNKNOWN,
            false);

        RadarTrack trackB = MakeCorrelationTrack(
            {0.15f, 10.0f},
            {0.0f, 0.0f},
            1.0f,
            ThreatClass::UNKNOWN,
            false);

        tracks.push_back(trackA);
        tracks.push_back(trackB);

        // Ping A is closest to track A.
        // Ping B is closest to track B after Ping A claims track A.
        SubmitPings(observer, {
            MakePing({0.02f, 10.0f}),
            MakePing({0.08f, 10.0f})
        });

        Systems::TrackingSystem(registry, 0.1f);

        ASSERT_EQ(tracks.size(), 2u)
            << "One-to-one assignment should not create duplicate tracks.";

        EXPECT_NEAR(tracks[0].latestPing.x, 0.02f, kEpsilon);
        EXPECT_NEAR(tracks[1].latestPing.x, 0.08f, kEpsilon);

        EXPECT_FALSE(tracks[0].isMissing);
        EXPECT_FALSE(tracks[1].isMissing);
    }

    TEST_F(RadarSystemsTest, DistinctTargetsReceiveDistinctGlobalIds)
    {
        const entt::entity observer = registry.create();
        auto& tracks = DetectionData().activeTracks[observer];
        tracks.push_back(MakeLocalTrack({10.0f, 20.0f}, ThreatClass::SURFACE_SHIP));
        tracks.push_back(MakeLocalTrack({100.0f, 120.0f}, ThreatClass::SURFACE_SHIP));

        Systems::SensorFusionSystem(registry, 1.0f / 60.0f);

        ASSERT_EQ(MasterTable().activeGlobalTracks.size(), 2u);
        EXPECT_NE(tracks[0].globalTrackId, 0u);
        EXPECT_NE(tracks[1].globalTrackId, 0u);
        EXPECT_NE(tracks[0].globalTrackId, tracks[1].globalTrackId);
    }

    TEST_F(RadarSystemsTest, GlobalIdRemainsStableAcrossRepeatedFusionUpdates)
    {
        const entt::entity observer = registry.create();
        auto& tracks = DetectionData().activeTracks[observer];
        tracks.push_back(MakeLocalTrack({10.0f, 20.0f}, ThreatClass::SURFACE_SHIP));

        Systems::SensorFusionSystem(registry, 0.1f);
        const uint32_t firstId = tracks.front().globalTrackId;
        ASSERT_NE(firstId, 0u);

        tracks.front().pos = {10.1f, 20.0f};
        tracks.front().latestPing = tracks.front().pos;
        Systems::SensorFusionSystem(registry, 0.1f);

        EXPECT_EQ(tracks.front().globalTrackId, firstId);
        EXPECT_EQ(MasterTable().activeGlobalTracks.size(), 1u);
    }

    TEST_F(RadarSystemsTest, StaleOrMissingLocalTrackDoesNotRefreshGlobalObservation)
    {
        constexpr uint32_t id = 1000;
        MasterTrackTable::GlobalTrack global{};
        global.track.globalTrackId = id;
        global.track.latestPing = {10.0f, 20.0f};
        global.track.pos = {10.5f, 20.0f}; // Predicted position, not a real ping.
        global.track.ageSec = 3.0f;
        global.track.isMissing = true;
        MasterTable().activeGlobalTracks.emplace(id, global);

        const entt::entity observer = registry.create();
        DetectionData().activeTracks[observer].push_back(
            MakeLocalTrack({10.5f, 20.0f}, ThreatClass::SURFACE_SHIP, true, id, 4.0f, true));

        Systems::SensorFusionSystem(registry, 0.1f);

        const auto& fused = MasterTable().activeGlobalTracks.at(id).track;
        EXPECT_GE(fused.ageSec, 3.1f)
            << "Fusion must not reset the time since the last real observation.";
        EXPECT_TRUE(fused.isMissing);
        EXPECT_NEAR(fused.latestPing.x, 10.0f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, GlobalTrackAgesWhenThereAreNoLocalReports)
    {
        constexpr uint32_t id = 1000;
        MasterTrackTable::GlobalTrack global{};
        global.track.globalTrackId = id;
        global.track.pos = {5.0f, 6.0f};
        global.track.latestPing = {5.0f, 6.0f};
        global.track.ageSec = 2.0f;
        MasterTable().activeGlobalTracks.emplace(id, global);

        Systems::SensorFusionSystem(registry, 1.0f);

        ASSERT_TRUE(MasterTable().activeGlobalTracks.contains(id));
        EXPECT_NEAR(MasterTable().activeGlobalTracks.at(id).track.ageSec,
                    3.0f, kEpsilon);
    }

    TEST_F(RadarSystemsTest, DeletesGlobalTrackAfterItsConfiguredTimeout)
    {
        constexpr uint32_t id = 1000;
        MasterTrackTable::GlobalTrack global{};
        global.track.globalTrackId = id;
        global.track.pos = {5.0f, 6.0f};
        global.track.latestPing = {5.0f, 6.0f};
        global.track.ageSec = TRACK_DELETION_TIMEOUT_SEC;
        MasterTable().activeGlobalTracks.emplace(id, global);

        Systems::SensorFusionSystem(registry, 0.02f);

        EXPECT_FALSE(MasterTable().activeGlobalTracks.contains(id));
    }

    TEST_F(RadarSystemsTest, FastAcceleratingTargetKeepsOneGlobalIdAndTracksTruth)
    {
        const entt::entity observer =
            AddObserver({0.0f, -30.0f}, false, 20.0f, 0.5f, 0.0f);
        const entt::entity target = AddTarget({10.0f, -30.0f}, true, 1.0f, 100.0f);

        float speedNmPerSec = 0.0f;
        uint32_t trackedId = 0;
        constexpr float dt = 1.0f / 60.0f;

        for (int frame = 0; frame < 60 * 8; ++frame) {
            speedNmPerSec = std::min(speedNmPerSec + 0.2f * dt, 0.8f);  // 0 -> ~2900 kt
            registry.get<Transform2D>(target).pos.x -= speedNmPerSec * dt;

            Systems::SensorSystem(registry, dt);
            Systems::TrackingSystem(registry, dt);
            Systems::SensorFusionSystem(registry, dt);

            if (frame < 60 * 3) continue;               // let it confirm

            // Local correlation must not create duplicate tracks for the same target.
            if (LocalTracks(observer).size() != 1u)
            {
                const auto& tracks = LocalTracks(observer);
                const auto& truth = registry.get<Transform2D>(target).pos;

                std::fprintf(
                    stderr,
                    "\n[LOCAL TRACK DUPLICATION] frame=%d, count=%zu\n"
                    "Truth position=(%.6f, %.6f), speed=%.6f NM/s\n",
                    frame,
                    tracks.size(),
                    truth.x,
                    truth.y,
                    speedNmPerSec);

                for (std::size_t i = 0; i < tracks.size(); ++i)
                {
                    const RadarTrack& t = tracks[i];

                    std::fprintf(
                        stderr,
                        "Track[%zu]: pos=(%.6f, %.6f), latestPing=(%.6f, %.6f), "
                        "age=%.6f, missing=%d, class=%d, hasVelocity=%d, "
                        "speed=%.3f, vel=(%.6f, %.6f)\n",
                        i,
                        t.pos.x,
                        t.pos.y,
                        t.latestPing.x,
                        t.latestPing.y,
                        t.ageSec,
                        t.isMissing,
                        static_cast<int>(t.classification),
                        t.hasVelocity,
                        t.speedKnots,
                        t.vel.x,
                        t.vel.y);
                }
            }

            ASSERT_EQ(LocalTracks(observer).size(), 1u)
                << "Local track duplication at frame " << frame;

            // The local track must continue updating its actual ping anchor.
            // A 0.5-second scan interval at 0.8 NM/s allows up to roughly
            // 0.4 NM of measurement age error.
            const RadarTrack& local = LocalTracks(observer).front();

            const float localPingError = MathUtils::GetDistance(
                local.latestPing,
                registry.get<Transform2D>(target).pos);

            EXPECT_LT(localPingError, 0.5f)
                << "Local ping anchor stopped following truth at frame " << frame;

            const auto& table = MasterTable().activeGlobalTracks;
            if (trackedId == 0) {
                ASSERT_FALSE(table.empty());
                trackedId = table.begin()->first;
            }
            ASSERT_TRUE(table.contains(trackedId)) << "global ID lost at frame " << frame;

            const auto& g = table.at(trackedId).track;
            const float err = MathUtils::GetDistance(g.pos, registry.get<Transform2D>(target).pos);
            EXPECT_LT(err, 0.5f) << "global track drifted from truth at frame " << frame;
            EXPECT_FALSE(g.isMissing) << "frame " << frame;
        }
    }

    // -----------------------------------------------------------------------------
    // TrackCorrelation::GetTrackGateNM tests
    // -----------------------------------------------------------------------------

    TEST_F(RadarSystemsTest, UnconfirmedTrackGateUsesClassMaximumSpeed)
    {
        RadarTrack track = MakeCorrelationTrack(
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            2.0f,
            ThreatClass::SURFACE_SHIP,
            false);

        const float gate =
            TrackCorrelation::GetTrackGateNM(track);

        const float expected =
            TRACK_POSITION_TOLERANCE_NM +
            (MAX_SHIP_SPEED_KTS * KNOTS_TO_NM_PER_SEC * 2.0f);

        EXPECT_NEAR(gate, expected, kEpsilon);
    }

    TEST_F(RadarSystemsTest, UnknownTrackGateUsesLooseMaximumSpeed)
    {
        RadarTrack track = MakeCorrelationTrack(
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            2.0f,
            ThreatClass::UNKNOWN,
            false);

        const float gate =
            TrackCorrelation::GetTrackGateNM(track);

        const float expected =
            TRACK_POSITION_TOLERANCE_NM +
            (MAX_PLAUSIBLE_SPEED_KNOTS * KNOTS_TO_NM_PER_SEC * 2.0f);

        EXPECT_NEAR(gate, expected, kEpsilon);
    }


    // -----------------------------------------------------------------------------
    // TrackCorrelation::GenerateCandidates tests
    // -----------------------------------------------------------------------------

    TEST_F(RadarSystemsTest, GenerateCandidatesReturnsCandidateInsideGate)
    {
        RadarTrack track = MakeCorrelationTrack(
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            1.0f,
            ThreatClass::SURFACE_SHIP,
            false);

        RadarTrack ping = MakePing({0.01f, 0.0f}, 30.0f);

        const std::vector pings{ping};
        const std::vector tracks{track};

        const auto candidates = TrackCorrelation::GenerateCandidates(
            pings,
            tracks,
            TrackCorrelation::SourceKind::PING);

        ASSERT_EQ(candidates.size(), 1u);
        EXPECT_EQ(candidates.front().sourceIndex, 0u);
        EXPECT_EQ(candidates.front().targetIndex, 0u);
    }

    TEST_F(RadarSystemsTest, GenerateCandidatesRejectsCandidateOutsideDistanceGate)
    {
        RadarTrack track = MakeCorrelationTrack(
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            1.0f,
            ThreatClass::SURFACE_SHIP,
            false);

        RadarTrack ping = MakePing({1.0f, 0.0f}, 30.0f);

        const std::vector pings{ping};
        const std::vector tracks{track};

        const auto candidates = TrackCorrelation::GenerateCandidates(
            pings,
            tracks,
            TrackCorrelation::SourceKind::PING);

        EXPECT_TRUE(candidates.empty());
    }

    TEST_F(RadarSystemsTest, GenerateCandidatesRejectsSpatiallyCloseButIncompatiblePing)
    {
        RadarTrack track = MakeCorrelationTrack(
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            1.0f,
            ThreatClass::SURFACE_SHIP,
            false);

        // This ping is close enough to be inside the distance gate,
        // but its implied velocity exceeds the ship's maximum speed.
        RadarTrack ping = MakePing({0.02f, 0.0f}, 30.0f);

        const std::vector<RadarTrack> pings{ping};
        const std::vector<RadarTrack> tracks{track};

        const auto candidates = TrackCorrelation::GenerateCandidates(
            pings,
            tracks,
            TrackCorrelation::SourceKind::PING);

        EXPECT_TRUE(candidates.empty());
    }

    TEST_F(RadarSystemsTest, GenerateCandidatesPreservesSourceAndTargetIndices)
    {
        RadarTrack trackA = MakeCorrelationTrack(
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            1.0f,
            ThreatClass::UNKNOWN,
            false);

        RadarTrack trackB = MakeCorrelationTrack(
            {10.0f, 0.0f},
            {0.0f, 0.0f},
            1.0f,
            ThreatClass::UNKNOWN,
            false);

        RadarTrack pingA = MakePing({0.01f, 0.0f});
        RadarTrack pingB = MakePing({10.01f, 0.0f});

        const std::vector<RadarTrack> pings{pingA, pingB};
        const std::vector<RadarTrack> tracks{trackA, trackB};

        const auto candidates = TrackCorrelation::GenerateCandidates(
            pings,
            tracks,
            TrackCorrelation::SourceKind::PING);

        ASSERT_EQ(candidates.size(), 2u);

        EXPECT_EQ(candidates[0].sourceIndex, 0u);
        EXPECT_EQ(candidates[0].targetIndex, 0u);

        EXPECT_EQ(candidates[1].sourceIndex, 1u);
        EXPECT_EQ(candidates[1].targetIndex, 1u);
    }

    TEST_F(RadarSystemsTest, ReacquiresTrackAfterOneMissedScanWithoutCreatingDuplicate)
    {
        const entt::entity observer = AddObserver();

        SubmitPings(observer, {MakePing({2.0f, 10.0f})});
        Systems::TrackingSystem(registry, 0.1f);

        ASSERT_EQ(LocalTracks(observer).size(), 1u);

        // A scan happened, but no ping was returned.
        SubmitPings(observer, {});
        Systems::TrackingSystem(registry, 0.5f);

        ASSERT_EQ(LocalTracks(observer).size(), 1u);
        ASSERT_TRUE(LocalTracks(observer).front().isMissing);

        // The same target is detected again before the deletion timeout.
        SubmitPings(observer, {MakePing({2.0f, 10.0f})});
        Systems::TrackingSystem(registry, 0.5f);

        ASSERT_EQ(LocalTracks(observer).size(), 1u)
            << "Reacquisition must update the old track rather than create a new one.";

        EXPECT_FALSE(LocalTracks(observer).front().isMissing);
        EXPECT_NEAR(
            LocalTracks(observer).front().latestPing.x,
            2.0f,
            kEpsilon);
    }

    TEST_F(RadarSystemsTest, AcceleratingMissileRemainsCorrelatedAcrossRadarScans)
    {
        const entt::entity observer =
            AddObserver(
                {0.0f, -30.0f},
                false,
                20.0f,
                0.5f,
                0.0f);

        const entt::entity target =
            AddTarget(
                {10.0f, -30.0f},
                true,
                1.0f,
                100.0f);

        float speedNmPerSec = 0.0f;

        constexpr float dt = 1.0f / 60.0f;
        constexpr float accelerationNmPerSecSq = 0.2f;
        constexpr float maxSpeedNmPerSec = 0.8f;

        constexpr int totalFrames = 60 * 8;

        for (int frame = 0; frame < totalFrames; ++frame)
        {
            // Simulate the missile's acceleration.
            speedNmPerSec = std::min(
                speedNmPerSec + accelerationNmPerSecSq * dt,
                maxSpeedNmPerSec);

            // Move the missile along its flight path.
            registry.get<Transform2D>(target).pos.x -=
                speedNmPerSec * dt;

            // Run the actual radar detection system.
            Systems::SensorSystem(registry, dt);

            // Determine whether this frame produced a radar scan.
            const bool didScan =
                DetectionData().framePings.contains(observer);

            // Run the actual local tracking system.
            Systems::TrackingSystem(registry, dt);

            // Only assert correlation on frames where a radar scan occurred.
            if (!didScan)
                continue;

            ASSERT_EQ(LocalTracks(observer).size(), 1u)
                << "Missile created a duplicate local track at frame "
                << frame
                << " (sim time = " << frame * dt << " sec, "
                << "speed = " << speedNmPerSec << " NM/sec)";

            const RadarTrack& track =
                LocalTracks(observer).front();

            // The track must continue receiving actual radar pings.
            EXPECT_FALSE(track.isMissing)
                << "Track marked missing at frame " << frame;

            // The latest real ping should continue following the missile.
            const float pingError =
                MathUtils::GetDistance(
                    track.latestPing,
                    registry.get<Transform2D>(target).pos);

            EXPECT_LT(pingError, 0.5f)
                << "Track stopped updating its ping anchor at frame "
                << frame;

            // Once velocity is established, it should remain established.
            if (frame > 60)
            {
                EXPECT_TRUE(track.hasVelocity)
                    << "Track lost velocity at frame " << frame;
            }
        }
    }

    TEST_F(RadarSystemsTest, TwoObserversKeepOneGlobalIdForMovingMissile)
{
    const entt::entity observerA = registry.create();
    const entt::entity observerB = registry.create();

    auto& tracksA = DetectionData().activeTracks[observerA];
    auto& tracksB = DetectionData().activeTracks[observerB];

    RadarTrack missileA =
        MakeLocalTrack({10.000f, 20.0f}, ThreatClass::UNKNOWN);

    RadarTrack missileB =
        MakeLocalTrack({10.010f, 20.0f}, ThreatClass::UNKNOWN);

    // Both reports represent the same moving missile.
    missileA.hasVelocity = true;
    missileB.hasVelocity = true;

    missileA.vel = {1.0f, 0.0f};
    missileB.vel = {1.0f, 0.0f};

    missileA.speedKnots = 3600.0f;
    missileB.speedKnots = 3600.0f;

    // SensorFusion ignores UNKNOWN-class local tracks.
    missileA.classification = ThreatClass::MISSILE_INBOUND;
    missileB.classification = ThreatClass::MISSILE_INBOUND;

    tracksA.push_back(missileA);
    tracksB.push_back(missileB);

    Systems::SensorFusionSystem(registry, 0.1f);

    ASSERT_EQ(MasterTable().activeGlobalTracks.size(), 1u)
        << "The two initial reports should correlate to one global track.";

    const uint32_t sharedId = tracksA.front().globalTrackId;

    ASSERT_NE(sharedId, NO_GLOBAL_TRACK);
    ASSERT_EQ(tracksB.front().globalTrackId, sharedId);

    // Continue moving both reports together.
    for (int i = 1; i <= 20; ++i)
    {
        const MathUtils::Vec2 positionA{
            10.000f + 0.1f * i,
            20.0f
        };

        const MathUtils::Vec2 positionB{
            10.010f + 0.1f * i,
            20.0f
        };

        tracksA.front().pos = positionA;
        tracksA.front().latestPing = positionA;
        tracksA.front().ageSec = 0.0f;

        tracksB.front().pos = positionB;
        tracksB.front().latestPing = positionB;
        tracksB.front().ageSec = 0.0f;

        Systems::SensorFusionSystem(registry, 0.1f);

        ASSERT_EQ(MasterTable().activeGlobalTracks.size(), 1u)
            << "A duplicate global track was created at iteration " << i;

        EXPECT_EQ(tracksA.front().globalTrackId, sharedId)
            << "Observer A lost the missile's global ID at iteration " << i;

        EXPECT_EQ(tracksB.front().globalTrackId, sharedId)
            << "Observer B lost the missile's global ID at iteration " << i;
    }
}
} // namespace
