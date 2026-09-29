#include "Systems.h"

#include "Components.h"

#include "../core/Constants.h"
#include "../utils/MetricsLogger.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace {
    void CleanupEngagementLogs(entt::registry& registry) {
        auto shipView = registry.view<EngagementLog>();
        for (auto entity : shipView) {
            auto& log = shipView.get<EngagementLog>(entity);
            for (auto& eng : log.current) {
                if (eng.missileAlive) {
                    eng.missileAlive = registry.valid(eng.missileEntity);
                }
            }
            std::erase_if(log.current, [](const ActiveEngagement& e) {return !e.missileAlive;});
        }
    }

    std::string SelectWeapon(const Magazine& mag, const RadarTrack& threat) {
        bool needInterceptor = (threat.classification == ThreatClass::MISSILE_INBOUND ||
                                threat.classification == ThreatClass::AIRCRAFT);

        for (const auto& [weaponId, count] : mag.currentAmmo) {
            if (count <= 0) continue;

            const MissileStats& m = TacticalDatabase::GetMissile(weaponId);

            if (needInterceptor && m.isInterceptor) return weaponId;
            if (!needInterceptor && !m.isInterceptor) return weaponId;
        }

        // Fallback: Use anything available if no ideal weapon
        for (const auto& [weaponId, count] : mag.currentAmmo) {
            if (count > 0) return weaponId;
        }

        return ""; // No ammo at all
    }

    int CountActiveShotsAgainst(const EngagementLog& log, const RadarTrack& threat, float currentSimTime) {
        int count = 0;
        for (const auto& eng : log.current) {
            if (!eng.missileAlive) continue;
            float timeSinceFired = currentSimTime - eng.timeFiredSec;

            MathUtils::Vec2 predictedEngPos = MathUtils::Add(eng.targetPos,
                MathUtils::Scale(eng.targetVel, timeSinceFired));

            float dist = MathUtils::GetDistance(predictedEngPos, threat.pos);
            if (dist < DATALINK_ENGAGEMENT_CORRELATION_RADIUS_NM) count++;
        }
        return count;
    }

    int DetermineSalvoSize(const RadarTrack& threat) {
        switch (threat.classification) {
            case ThreatClass::MISSILE_INBOUND:
                // Inbound missiles get two shots immediately - too important to risk
                // Pk per shot ~0.7, two shots gives ~0.91 Pk
                if (threat.timeToImpactSec < 60.0f) return 2; // Danger close - ripple fire
                return 1; // Time to assess the first shot

            case ThreatClass::AIRCRAFT:
                return 1; // Time to assess the first shot

            case ThreatClass::SURFACE_SHIP:
                return 1; // Time to assess the first shot

            default:
                return 1;
        }
    }

    entt::entity LaunchMissile(
        entt::registry& registry,
        entt::entity shooter,
        const Transform2D& shooterTransform,
        const Kinematics& shooterKin,
        const IFF& shooterIFF,
        const RadarTrack& target,
        const MissileStats& mStats,
        const std::string& weaponId)
    {
        auto missile = registry.create();

        MathUtils::Vec2 initialHeading = {1.0f, 0.0f}; // Fallback
        MathUtils::Vec2 toTarget = MathUtils::Sub(target.pos, shooterTransform.pos);

        float distToTarget = MathUtils::Length(toTarget);
        if (distToTarget > 0.001f) initialHeading = MathUtils::Scale(toTarget, 1.0f / distToTarget);

        registry.emplace<SeekerHead>(missile,
            shooter,
            mStats.seekerType,
            mStats.seekerRangeNM,
            GuidancePhase::MIDCOURSE_DATALINK,
            target.pos,
            target.vel,
            target.altitude,
            mStats.isInterceptor,
            0.0, // Time since last correlation
            0.0, // Time since launch (seconds)
            mStats.maxLateralGs
            );
        registry.emplace<Transform2D>(missile,
            shooterTransform.pos, shooterTransform.altitude, shooterTransform.heading);

        float launchSpeedKnots = mStats.maxSpeedKnots * LAUNCH_SPEED_FRACTION;
        registry.emplace<Kinematics>(missile,
            shooterKin.velocity,  // velocity
            initialHeading,    // headingVector
            mStats.maxSpeedKnots, // maxSpeedKnots
            launchSpeedKnots,  // currentSpeedKnots
            mStats.maxSpeedKnots, // desiredSpeedKnots
            50.0f);               // accelerationRate

        registry.emplace<Aerodynamics>(missile,
            mStats.massKg, 1.0f / mStats.massKg, mStats.areaM2,
            mStats.thrustNewtons, mStats.maxFuelKg, mStats.burnRateKgSec,
            mStats.cruiseAltitudeMeters);

        float baseReliability = 0.9f; // Todo: Pull this from JSON file
        registry.emplace<Warhead>(missile,
            mStats.warheadYield, mStats.lethalRadiusNM,
            mStats.lethalRadiusNM * mStats.lethalRadiusNM,
            baseReliability, weaponId, shooter);

        registry.emplace<RadarSignature>(missile, mStats.rcs, mStats.rcsFourthRoot);
        registry.emplace<IFF>(missile, shooterIFF.isHostile);

        MetricsLogger::Log(Systems::GetSimTime(registry), "FIRE", (uint32_t)shooter,
            (int)target.classification, weaponId, distToTarget, "IN_FLIGHT");

        return missile;
    }

    void FireSalvo(
        entt::registry& registry,
        entt::entity entity,
        const Transform2D& transform,
        const Kinematics& kin,
        const IFF& iff,
        Magazine& mag,
        EngagementLog& log,
        RadarTrack& threat,
        const MissileStats& mStats,
        const std::string& selectedWeapon,
        float distToTarget,
        int salvoCount)
    {
        for (int shot = 0; shot < salvoCount; ++shot) {
            if (mag.currentAmmo[selectedWeapon] <= 0) break;

            auto missileEntity = LaunchMissile(registry, entity, transform,kin,
                iff, threat, mStats, selectedWeapon);

            if (VERBOSE_COMBAT_LOG) {
                std::cout << "[T=" << Systems::GetSimTime(registry) << "s] [FIRE] Entity " << (uint32_t)entity << " launched " << selectedWeapon
                                  << " at threat (class=" << (int)threat.classification
                                  << ", dist=" << distToTarget << "nm, TTI=" << threat.timeToImpactSec << "s)\n";
            }

            log.current.push_back({
                threat.pos,
                threat.vel,
                missileEntity,
                threat.classification,
                Systems::GetSimTime(registry),
                true});

            log.totalMissilesFired++;
            mag.currentAmmo[selectedWeapon]--;
        }
    }

    bool ProcessThreat(
        entt::registry& registry,
        entt::entity entity,
        Transform2D& transform,
        IFF& iff,
        Magazine& mag,
        EngagementLog& log,
        RadarTrack& threat)
    {
        if (threat.consistentObservationSec < 1.0f) return false;
        if (threat.ageSec > TRACK_ACTIONABLE_FRESHNESS_SEC) return false;

        // Selected the right weapon for this target
        std::string selectedWeapon = SelectWeapon(mag, threat);
        if (selectedWeapon.empty()) return false;

        const MissileStats& mStats = TacticalDatabase::GetMissile(selectedWeapon);
        float distToTarget = MathUtils::GetDistance(transform.pos, threat.pos);

        // Target must be in weapon range
        if (distToTarget > mStats.maxRangeNM) return false;

        // Don't dire at close range - too late for missiles to arm/guide
        if (distToTarget < MIN_ENGAGEMENT_RANGE_NM) return false;

        // Shoot-Look-Shoot: Are we ALREADY engaging this threat with a missile in flight?
        constexpr int MAX_SHOTS_PER_THREAT = 2;

        int activeShots = CountActiveShotsAgainst(log, threat, Systems::GetSimTime(registry));
        if (activeShots >= MAX_SHOTS_PER_THREAT) return false;

        // First shot is still in flight with time to spare
        if (activeShots == 1 && threat.timeToImpactSec > 30.0f) return false;

        // Only fire last missiles at incoming missiles (keep at least 1 in reserve per type)
        if (mag.currentAmmo.at(selectedWeapon) <= 1 &&
            threat.classification != ThreatClass::MISSILE_INBOUND) return false;

        int salvoCount = DetermineSalvoSize(threat);

        // Don't engage targets another ship is already handling
        // (Unless it's an incoming missile - then everyone who can shoot it, should)
        auto& sharedPicture = registry.ctx().get<SharedThreatPicture>();
        if (threat.classification != ThreatClass::MISSILE_INBOUND) {
            if (sharedPicture.IsAssigned(threat.pos)) return false;
        }

        // Claim the target before firing
        sharedPicture.Assign(threat.pos, entity);

        FireSalvo(registry, entity, transform, registry.get<Kinematics>(entity),
            iff, mag, log, threat, mStats, selectedWeapon, distToTarget, salvoCount);

        return true;
    }

    void ProcessCombatEntity(entt::registry& registry, entt::entity entity, float deltaTime,
        RadarDetectionData& detectionData, std::vector<RadarTrack> sortedThreats)
    {
        auto& transform = registry.get<Transform2D>(entity);
        auto& iff = registry.get<IFF>(entity);
        auto& mag = registry.get<Magazine>(entity);
        auto& brain = registry.get<AutonomousGuidance>(entity);
        auto& log = registry.get<EngagementLog>(entity);

        if (brain.currentState != TacticalState::STANDOFF &&
            brain.currentState != TacticalState::DEFEND &&
            brain.currentState != TacticalState::INTERCEPT) return;

        auto& tracks = detectionData.activeTracks[entity];
        if (tracks.empty()) return;

        // --- Sort tracks by threat score (Highest priority first) ---
        sortedThreats.clear();
        for (auto& track : tracks) sortedThreats.push_back(track);

        std::sort(sortedThreats.begin(), sortedThreats.end(),
            [](const RadarTrack& a, const RadarTrack& b) {
            return a.threatScore > b.threatScore;
    });

        for (RadarTrack threat : sortedThreats) {
            if (ProcessThreat(registry, entity, transform, iff, mag, log, threat)) {
                break;
            }
        }
    }
}

void Systems::CombatSystem(entt::registry& registry, float deltaTime) {
    registry.ctx().get<SharedThreatPicture>().Clear();

    auto& detectionData = registry.ctx().get<RadarDetectionData>();

    CleanupEngagementLogs(registry);

    auto combatView = registry.view<Transform2D, IFF, Magazine, AutonomousGuidance, EngagementLog>();

    std::vector<RadarTrack> sortedThreats;
    sortedThreats.reserve(16);

    for (auto entity : combatView) {
        ProcessCombatEntity(registry, entity, deltaTime, detectionData, sortedThreats);
    }
}