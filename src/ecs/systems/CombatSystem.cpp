#include "../Systems.h"
#include "../Components.h"
#include "../../core/Constants.h"
#include "../../utils/MetricsLogger.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace {
    void PruneDeadEngagements(entt::registry& registry) {
        for (auto [entity, log] : registry.view<EngagementLog>().each()) {

            // Update the alive status of each tracked missile
            for (auto& eng : log.current) {
                if (eng.missileAlive) {
                    eng.missileAlive = registry.valid(eng.missileEntity) &&
                                       !registry.all_of<DeadTag>(eng.missileEntity);
                }
            }

            // Remove dead engagements from the vector
            std::erase_if(log.current, [](const ActiveEngagement& e) {
                return !e.missileAlive;
            });
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

            MathUtils::Vec2 predictedThreatPos = MathUtils::ExtrapolatePosition(eng.targetPos, threat.vel, timeSinceFired);

            float dist = MathUtils::GetDistance(predictedThreatPos, threat.pos);
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
            case ThreatClass::SURFACE_SHIP:
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

        const float distToTarget = MathUtils::GetDistance(shooterTransform.pos, target.pos);

        MathUtils::Vec2 initialHeading = {1.0f, 0.0f}; // Fallback: due east if the target is on top of us
        if (distToTarget > 0.001f) {
            initialHeading = MathUtils::GetDirection(shooterTransform.pos, target.pos);
        }

        registry.emplace<SeekerHead>(missile,
            shooter,
            mStats.seekerType,
            mStats.seekerRangeNM,
            GuidancePhase::MIDCOURSE_DATALINK,
            target.pos,
            target.vel,
            target.altitude,
            mStats.isInterceptor,
            0.0f, // Time since last correlation
            0.0f, // Time since launch (seconds)
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

        auto& aero = registry.emplace<Aerodynamics>(missile,
            mStats.massKg, 1.0f / mStats.massKg, mStats.areaM2,
            mStats.thrustNewtons, mStats.maxFuelKg, mStats.burnRateKgSec,
            mStats.cruiseAltitudeMeters);
        aero.maxManeuverGs = mStats.maxLateralGs;

        // First altitude order, so the missile has one before guidance gets its first look at it
        aero.desiredAltitudeMeters = Systems::PlanMissileAltitude(
            registry.get<Transform2D>(missile), registry.get<Kinematics>(missile),
            aero, registry.get<SeekerHead>(missile));

        float baseReliability = 0.9f; // Todo: Pull this from JSON file
        registry.emplace<Warhead>(missile,
            mStats.warheadYield, mStats.lethalRadiusNM,
            mStats.lethalRadiusNM * mStats.lethalRadiusNM,
            baseReliability, weaponId, shooter);

        registry.emplace<RadarSignature>(missile, mStats.rcs, mStats.rcsFourthRoot);
        registry.emplace<IFF>(missile, shooterIFF.isHostile);

        Systems::RecordMissileFired(registry, shooter, missile,
            static_cast<int>(target.classification), weaponId, distToTarget);

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
        const RadarTrack& threat,
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

    EngagementEvaluationResult EvaluateEngagement(
        entt::registry& registry,
        const Transform2D& transform,
        const Magazine& mag,
        const EngagementLog& log,
        const RadarTrack& threat)
    {
        EngagementEvaluationResult result;

        if (threat.consistentObservationSec < MIN_OBSERVATION_TIME_SEC) {
            result.status = EngagementStatus::TrackUnconfirmed;
            return result;
        }
        if (threat.ageSec > TRACK_ACTIONABLE_FRESHNESS_SEC) {
            result.status = EngagementStatus::TrackStale;
            return result;
        }

        // --- Shoot-Look-Shoot Logic ---
        int activeShots = CountActiveShotsAgainst(log, threat, Systems::GetSimTime(registry));
        if (activeShots >= MAX_SHOTS_PER_THREAT ||
           (activeShots == 1 && threat.timeToImpactSec > MAX_TTI_FOR_SECOND_SHOT_SEC)) {
            result.status = EngagementStatus::MaxShotsInFlight;
            return result;
        }
        // --- Weapon & Range Selection ---
        std::string selectedWeapon = SelectWeapon(mag, threat);
        if (selectedWeapon.empty()) {
            result.status = EngagementStatus::NoCompatibleWeapon;
            return result;
        }

        // Assign the pointer once; we know it's valid past this point
        result.missileStats = &TacticalDatabase::GetMissile(selectedWeapon);
        result.distanceToTarget = MathUtils::GetDistance(transform.pos, threat.pos);

        if (result.distanceToTarget > result.missileStats->maxRangeNM) {
            result.status = EngagementStatus::OutOfRange;
            return result;
        }
        if (result.distanceToTarget < MIN_ENGAGEMENT_RANGE_NM) {
            result.status = EngagementStatus::BelowMinRange;
            return result;
        }

        int currentAmmo = mag.currentAmmo.at(result.missileStats->name);
        int maxAmmo = mag.maxAmmo.at(result.missileStats->name);
        int reserveThreshold = std::max(1, static_cast<int>(maxAmmo * MIN_AMMO_RESERVE_RATIO));

        if (currentAmmo <= reserveThreshold && threat.classification != ThreatClass::MISSILE_INBOUND) {
            result.status = EngagementStatus::AmmoReserveProtected;
            return result;
        }

        // --- Datalink check to see if we're already targeting this ---
        // Todo: Needs more nuance... a missile inbound in 20 minutes does not require multiple ships to fire at it
        auto& sharedPicture = registry.ctx().get<SharedThreatPicture>();
        if (threat.classification != ThreatClass::MISSILE_INBOUND && sharedPicture.IsAssigned(threat.pos)) {
            result.status = EngagementStatus::TargetAlreadyAssigned;
            return result;
        }

        result.status = EngagementStatus::ReadyToEngage;
        return result;
    }

    bool TryEngageThreat(
        entt::registry& registry,
        entt::entity entity,
        Transform2D& transform,
        IFF& iff,
        Magazine& mag,
        EngagementLog& log,
        const RadarTrack& threat)
    {
        EngagementEvaluationResult eval = EvaluateEngagement(registry, transform, mag, log, threat);

        if (eval.status != EngagementStatus::ReadyToEngage) return false;

        auto& sharedPicture = registry.ctx().get<SharedThreatPicture>();
        sharedPicture.Assign(threat.pos, entity);

        int salvoCount = DetermineSalvoSize(threat);
        FireSalvo(registry, entity, transform, registry.get<Kinematics>(entity),
                     iff, mag, log, threat,
                     *eval.missileStats, eval.missileStats->name, eval.distanceToTarget, salvoCount);

        return true;
    }

    bool SortThreatsByPriority(entt::entity entity, RadarDetectionData& detectionData, std::vector<RadarTrack>*& outTracks)
    {
        // --- Sort tracks by threat score (Highest priority first) ---

        // Look up the entity in the map safely without auto-inserting
        auto it = detectionData.activeTracks.find(entity);
        if (it == detectionData.activeTracks.end()) return false;

        // Grab a reference to the actual vector of tracks
        auto& tracks = it->second;
        if (tracks.empty()) return false;

        // Sort the real tracks in place
        std::sort(tracks.begin(), tracks.end(),
            [](const RadarTrack& a, const RadarTrack& b) {
            return a.threatScore > b.threatScore;
        });

        outTracks = &tracks;
        return true;
    }
}

void Systems::CombatSystem(entt::registry& registry) {
    registry.ctx().get<SharedThreatPicture>().Clear();
    auto& detectionData = registry.ctx().get<RadarDetectionData>();

    PruneDeadEngagements(registry);

    auto combatView = registry.view<Transform2D, IFF, Magazine, AutonomousGuidance, EngagementLog>();

    combatView.each([&](entt::entity entity,
                        Transform2D& transform,
                        IFF& iff,
                        Magazine& mag,
                        AutonomousGuidance& brain,
                        EngagementLog& log)
    {
        if (brain.currentState != TacticalState::STANDOFF &&
            brain.currentState != TacticalState::DEFEND &&
            brain.currentState != TacticalState::INTERCEPT) return;

        std::vector<RadarTrack>* tracks = nullptr;

        // Does the lookup, sorts in place, and bails out if empty/missing
        if (!SortThreatsByPriority(entity, detectionData, tracks)) return;

        // Loop through the sorted tracks at the pointer
        for (const RadarTrack& threat : *tracks) {
            if (TryEngageThreat(registry, entity, transform, iff, mag, log, threat)) {
                break; // We've fired at a inbound threat
            }
        }
    });
}