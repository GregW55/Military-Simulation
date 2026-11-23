#include <gtest/gtest.h>

#include "RadarTestHelpers.h"
#include "ecs/Systems.h"
#include "ecs/components.h"

class MissileGuidanceTests : public ::testing::Test
{
protected:
    entt::registry registry;
    entt::entity missile = entt::null;

    void SetUp() override
    {
        registry.ctx().emplace<RadarDetectionData>();
        registry.ctx().emplace<MasterTrackTable>();
        registry.ctx().emplace<float>(0.0f);

        missile = registry.create();
        registry.emplace<Transform2D>(
            missile, Transform2D{{0.0f, 0.0f}, 1000.0f, 0.0f});

        Kinematics kin{};
        kin.currentSpeedKnots = 500.0f;
        kin.velocity = {500.0f * KNOTS_TO_NM_PER_SEC, 0.0f};
        kin.headingVector = {1.0f, 0.0f};
        registry.emplace<Kinematics>(missile, kin);

        registry.emplace<IFF>(missile, IFF{false});
    }

    SeekerHead& Seeker()
    {
        return registry.get<SeekerHead>(missile);
    }
};

TEST_F(MissileGuidanceTests, MidcourseUsesAssignedGlobalTrackInsteadOfNearestLocalDecoy)
{
    auto& seeker = registry.emplace<SeekerHead>(missile);
    seeker.targetGlobalTrackId = 1001u;
    seeker.type = SeekerType::SEMI_ACTIVE;
    seeker.phase = GuidancePhase::MIDCOURSE_DATALINK;
    seeker.targetPos = {5.0f, 0.0f};
    seeker.targetVel = {0.0f, 0.0f};
    seeker.targetAltitude = 1000.0f;
    seeker.rangeNM = 0.25f;
    seeker.maxLateralGs = 10.0f;

    // The assigned global track is at 10 NM and 2,000 m altitude.
    MasterTrackTable::GlobalTrack global{};
    global.track = radar_test::MakeInitializedTrack(
        {10.0f, 0.0f}, 2000.0f, {0.0f, 0.0f}, 0.0f, 1.0f);
    global.track.globalTrackId = 1001u;
    registry.ctx().get<MasterTrackTable>()
        .GetSide(false).activeGlobalTracks.emplace(1001u, global);

    // A friendly observer reports a different track exactly where the
    // missile's old target estimate currently is. The old implementation
    // follows this decoy because it only searches for the nearest track.
    const entt::entity friendlyObserver = registry.create();
    registry.emplace<IFF>(friendlyObserver, IFF{false});
    registry.emplace<RadarEmitter>(friendlyObserver);

    RadarTrack decoy = radar_test::MakeLocalTrack(
        {5.0f, 0.0f}, ThreatClass::AIRCRAFT,
        {0.0f, 0.0f}, true, 1002u, 0.0f, false, 9000.0f);
    decoy.lifecycle = TrackLifecycle::CONFIRMED;
    decoy.consistentObservationSec = 2.0f;
    decoy.lastMeasurementTimeSec = 1.0f;

    radar_test::SubmitTracks(
        registry.ctx().get<RadarDetectionData>(),
        friendlyObserver,
        {decoy}
    );

    Systems::GuidanceSystem(registry, 1.0f / 60.0f);

    EXPECT_EQ(Seeker().targetGlobalTrackId, 1001u);
    EXPECT_NEAR(Seeker().targetPos.x, 10.0f, 1.0e-4f);
    EXPECT_NEAR(Seeker().targetAltitude, 2000.0f, 1.0e-3f);
}

TEST_F(MissileGuidanceTests, ActiveTerminalSeekerCannotAcquireDifferentGlobalTrack)
{
    auto& seeker = registry.emplace<SeekerHead>(missile);
    seeker.targetGlobalTrackId = 1001u;
    seeker.type = SeekerType::ACTIVE_RADAR;
    seeker.phase = GuidancePhase::TERMINAL_PITBULL;
    seeker.targetPos = {10.0f, 0.0f};
    seeker.targetVel = {0.0f, 0.0f};
    seeker.targetAltitude = 2000.0f;
    seeker.rangeNM = 20.0f;
    seeker.maxLateralGs = 10.0f;

    // Decoy is slightly closer to the missile and appears first.
    RadarTrack decoy = radar_test::MakeLocalTrack(
        {9.95f, 0.0f}, ThreatClass::AIRCRAFT,
        {0.0f, 0.0f}, true, 1002u, 0.0f, false, 9000.0f);
    decoy.lifecycle = TrackLifecycle::CONFIRMED;
    decoy.lastMeasurementTimeSec = 1.0f;

    RadarTrack assigned = radar_test::MakeLocalTrack(
        {10.0f, 0.0f}, ThreatClass::MISSILE_INBOUND,
        {0.0f, 0.0f}, true, 1001u, 0.0f, false, 2000.0f);
    assigned.lifecycle = TrackLifecycle::CONFIRMED;
    assigned.lastMeasurementTimeSec = 1.0f;

    radar_test::SubmitTracks(
        registry.ctx().get<RadarDetectionData>(),
        missile,
        {decoy, assigned}
    );

    Systems::GuidanceSystem(registry, 1.0f / 60.0f);

    EXPECT_EQ(Seeker().targetGlobalTrackId, 1001u);
    EXPECT_NEAR(Seeker().targetPos.x, 10.0f, 1.0e-4f);
    EXPECT_NEAR(Seeker().targetAltitude, 2000.0f, 1.0e-3f);
}
