#pragma once
#include "Components.h"
#include "../core/Constants.h"
#include "../core/Physics.h"
#include "../utils/ThreatAnalysis.h"
#include "../utils/MetricsLogger.h"
#include <iostream>
#include <execution>
#include <random>

constexpr bool VERBOSE_COMBAT_LOG = true;

class Systems {
public:
    static float GetSimTime(entt::registry& registry) {
        return registry.ctx().contains<float>() ? registry.ctx().get<float>() : 0.0f;
    }

    // --- NAVIGATION SYSTEM  ---
    static void NavigationSystem(entt::registry& registry,float deltaTime);

    // --- MOVEMENT SYSTEM ---
    static void MovementSystem(entt::registry& registry, float deltaTime);

    // --- RADAR ---
    static void RadarSystem(entt::registry& registry, float deltaTime);

    // --- COMBAT SYSTEM ---
    static void CombatSystem(entt::registry& registry, float deltaTime) {
        registry.ctx().get<SharedThreatPicture>().Clear();
        auto& detectionData = registry.ctx().get<RadarDetectionData>();

        auto shipView = registry.view<EngagementLog>();
        for (auto entity : shipView) {
            auto& log = shipView.get<EngagementLog>(entity);
            for (auto& eng : log.current) {
                if (eng.missileAlive) {
                    eng.missileAlive = registry.valid(eng.missileEntity);
                }
            }
            std::erase_if(log.current, [](const ActiveEngagement& e) { return !e.missileAlive; });
        }

        // --- Engagement decisions ---
        auto combatView = registry.view<Transform2D, IFF, Magazine, AutonomousGuidance, EngagementLog>();
        std::vector<RadarTrack*> sortedThreats;
        sortedThreats.reserve(16);

        for (auto entity: combatView) {
            auto& transform = combatView.get<Transform2D>(entity);
            auto& iff = combatView.get<IFF>(entity);
            auto& mag = combatView.get<Magazine>(entity);
            auto& brain = combatView.get<AutonomousGuidance>(entity);
            auto& log = combatView.get<EngagementLog>(entity);

            if (brain.currentState != TacticalState::STANDOFF &&
                brain.currentState != TacticalState::DEFEND   &&
                brain.currentState != TacticalState::INTERCEPT) continue;

            if (mag.fireCooldown > 0.0f){
                mag.fireCooldown -= deltaTime;
                continue;
            }

            auto& tracks = detectionData.activeTracks[entity];
            if (tracks.empty()) continue;

            // --- Sort tracks by threat score (highest priority first) ---
            sortedThreats.clear();
            for (auto& track : tracks) sortedThreats.push_back(&track);
            std::sort(sortedThreats.begin(), sortedThreats.end(),
                [](const RadarTrack* a, const RadarTrack* b) {
                    return a->threatScore > b->threatScore;
                });

            // --- Evaluate each threat and decide ---
            for (RadarTrack* threat : sortedThreats) {
                if (threat->consistentObservationSec < 1.0f) continue; // require stable, multi-scan-confirmed data before firing
                if (threat->ageSec > TRACK_ACTIONABLE_FRESHNESS_SEC) continue;

                // DOCTRINE RULE 2: Select the right weapon for this target
                std::string selectedWeapon = SelectWeapon(mag, *threat);
                if (selectedWeapon.empty()) continue;

                const MissileStats& mStats = TacticalDatabase::GetMissile(selectedWeapon);
                float distToTarget = MathUtils::GetDistance(transform.pos, threat->pos);

                // DOCTRINE RULE 3: Target must be in weapon employment zone
                if (distToTarget > mStats.maxRangeNM) continue;

                // Don't fire at close range - too late for missile to arm/guide
                if (distToTarget < MIN_ENGAGEMENT_RANGE_NM) continue;

                // DOCTRINE RULE 4: Shoot-Look-Shoot: Are we ALREADY engaging this threat with a missile in flight?
                constexpr int MAX_SHOTS_PER_THREAT = 2;

                int activeShots = CountActiveShotsAgainst(log, *threat, GetSimTime(registry));
                if (activeShots >= MAX_SHOTS_PER_THREAT) {
                    continue; // Already saturated this target, move on to next threat
                }

                if (activeShots == 1 && threat->timeToImpactSec > 30.0f) {
                    continue; // First shot is still in flight with time to spare - let it work
                }

                // DOCTRINE RULE 5: Ammo conservation
                // Never fire last missile (keep at least 1 in reserve per type)
                if (mag.currentAmmo.at(selectedWeapon) <= 1 &&
                    threat->classification != ThreatClass::MISSILE_INBOUND){
                    continue; // Save last missile for incoming missiles
                }

                // DOCTRINE RULE 6: Determine salvo size
                int salvoCount = DetermineSalvoSize(*threat);

                // DOCTRINE RULE 7: Don't engage targets another ship is already handling
                // (Unless it's an incoming missile - then everyone who can shoot, does)
                auto& sharedPicture = registry.ctx().get<SharedThreatPicture>();
                if  (threat->classification != ThreatClass::MISSILE_INBOUND) {
                    if (sharedPicture.IsAssigned(threat->pos)) continue;
                }

                // Claim the target before firing
                sharedPicture.Assign(threat->pos, entity);

                // --- FIRE ---
                for (int shot = 0; shot < salvoCount; ++shot) {
                    if (mag.currentAmmo[selectedWeapon] <= 0) break;

                    auto missileEntity = LaunchMissile(registry, entity, transform,
                        registry.get<Kinematics>(entity), iff, *threat, mStats, selectedWeapon);
                    if (VERBOSE_COMBAT_LOG) {
                        std::cout << "[T=" << GetSimTime(registry) << "s] [FIRE] Entity " << (uint32_t)entity << " launched " << selectedWeapon
                                  << " at threat (class=" << (int)threat->classification
                                  << ", dist=" << distToTarget << "nm, TTI=" << threat->timeToImpactSec << "s)\n";
                    }

                    log.current.push_back({
                        threat->pos,
                        threat->vel,
                        missileEntity,
                        threat->classification,
                        GetSimTime(registry),
                        true
                    });
                    log.totalMissilesFired++;
                    mag.currentAmmo[selectedWeapon]--;
                }

                mag.fireCooldown = (threat->classification == ThreatClass::MISSILE_INBOUND)
                    ? 0.5f    // Fast response for incoming missiles
                    : 5.0f;   // Normal cooldown for surface targets

                break;
            }
        }
    }

    static void ProximityFuseSystem(entt::registry& registry) {
        auto missiles = registry.view<Transform2D, Warhead, IFF>();
        auto shipTargets = registry.view<Transform2D, Hull, IFF, Kinematics, RadarSignature>();

        for (auto missile : missiles) {
            if (registry.all_of<DeadTag>(missile)) continue;

            auto& mTrans = missiles.get<Transform2D>(missile);
            auto& warhead = missiles.get<Warhead>(missile);
            auto& mIFF = missiles.get<IFF>(missile);

            for (auto target: shipTargets) {
                auto& tTrans = shipTargets.get<Transform2D>(target);
                auto& tIFF = shipTargets.get<IFF>(target);
                auto& tKin = shipTargets.get<Kinematics>(target);
                auto& tSig = shipTargets.get<RadarSignature>(target);

                if (mIFF.isHostile == tIFF.isHostile) continue;

                // Fast-Fail Distance Check
                MathUtils::Vec2 diff = MathUtils::Sub(mTrans.pos, tTrans.pos);
                float distNmSq = MathUtils::LengthSq(diff);

                float altDiffMeters = std::abs(mTrans.altitude - tTrans.altitude);

                // DETONATION SEQUENCE
                if (distNmSq <= warhead.lethalRadiusNmSq && altDiffMeters <= FUSE_ALTITUDE_TOLERANCE_METERS) {

                    // --- DYNAMIC PK CALCULATION ---
                    float dynamicPk = warhead.baseReliability;

                    // Todo: More realistic calculation instead of broad assumptions
                    // Kinematic Evasion: Is the target moving fast?
                    if (tKin.currentSpeedKnots > 1400.0f) {
                        dynamicPk *= 0.25f;
                    }
                    else if (tKin.currentSpeedKnots > 750.0f) {
                        dynamicPk *= 0.4f; // Targets moving this fast significantly harder to hit
                    }
                    else if (tKin.currentSpeedKnots > 350.0f) {
                        dynamicPk *= 0.75f; // Fast targets are 25% harder to hit
                    } else if (tKin.currentSpeedKnots > 35.0f) {
                        dynamicPk *= 0.95f; // Fast ships are slightly harder to hit
                    }

                    // Signature Evasion: Is the target stealthy/small?
                    if (tSig.rcs < 1.0f) {
                        dynamicPk *= 0.7f; // Small RCS degrades seeker lock
                    }

                    // --- ROLL THE DICE ---

                    // Use a fallback RNG if the global one isn't injected yet, to prevent crashes
                    std::mt19937 fallbackRng(std::random_device{}());
                    auto& rng = registry.ctx().contains<std::mt19937>() ? registry.ctx().get<std::mt19937>() : fallbackRng;

                    std::uniform_real_distribution<float> distRoll(0.0f, 1.0f);
                    float roll = distRoll(rng);

                    float distNM = std::sqrt(distNmSq);

                    // --- DETERMINE OUTCOME ---
                    if (roll <= dynamicPk) {
                        // HIT
                        auto& hull = shipTargets.get<Hull>(target);
                        hull.currentHP -= warhead.yieldDamage;

                        MetricsLogger::Log(GetSimTime(registry), "IMPACT", (uint32_t)warhead.shooter, 0, warhead.weaponId, distNM, "HIT");

                        if (VERBOSE_COMBAT_LOG) {
                            std::cout << "[T=" << GetSimTime(registry) << "s] [IMPACT] Missile HIT entity " << (uint32_t)target <<"\n";
                        }

                        if (hull.currentHP <= 0.0f) {
                            registry.emplace_or_replace<DeadTag>(target);
                        }
                    } else {
                        // MISS
                        MetricsLogger::Log(GetSimTime(registry), "IMPACT", (uint32_t)warhead.shooter, 0, warhead.weaponId, distNM, "MISS");

                        if (VERBOSE_COMBAT_LOG) {
                            std::cout << "[T=" << GetSimTime(registry) << "s] [IMPACT] Missile MISSED entity " << (uint32_t)target << " (Roll: " << roll << ", Required: " << dynamicPk << ")\n";
                        }
                    }

                    registry.emplace_or_replace<DeadTag>(missile);
                    break;
                }
            }

            if (registry.all_of<DeadTag>(missile)) continue;

            for (auto otherMissile : missiles) {
                if (otherMissile == missile) continue;
                if (registry.all_of<DeadTag>(otherMissile)) continue;

                auto& oIFF = missiles.get<IFF>(otherMissile);
                if (mIFF.isHostile == oIFF.isHostile) continue;

                auto& oTrans = missiles.get<Transform2D>(otherMissile);
                float distNmSq = MathUtils::LengthSq(MathUtils::Sub(mTrans.pos, oTrans.pos));
                float altDiffMeters = std::abs(mTrans.altitude - oTrans.altitude);

                if (distNmSq <= warhead.lethalRadiusNmSq && altDiffMeters <= FUSE_ALTITUDE_TOLERANCE_METERS) {
                    auto& otherWarhead = missiles.get<Warhead>(otherMissile);

                    MetricsLogger::Log(GetSimTime(registry), "INTERCEPT", (uint32_t)warhead.shooter, 3, warhead.weaponId, std::sqrt(distNmSq), "KILLED_THREAT");
                    MetricsLogger::Log(GetSimTime(registry), "SHOT_DOWN", (uint32_t)otherWarhead.shooter, 0, otherWarhead.weaponId, std::sqrt(distNmSq), "INTERCEPTED");

                    registry.emplace_or_replace<DeadTag>(missile);
                    registry.emplace_or_replace<DeadTag>(otherMissile);
                    if (VERBOSE_COMBAT_LOG) {
                        std::cout << "[INTERCEPT] Missile " << (uint32_t)missile
                                  << " destroyed missile " << (uint32_t)otherMissile << "\n";
                    }
                    break;
                }

            }
        }
    }
private:

    static std::string SelectWeapon(const Magazine& mag, const RadarTrack& threat) {

        bool needInterceptor = (threat.classification == ThreatClass::MISSILE_INBOUND ||
                                threat.classification == ThreatClass::AIRCRAFT);

        for (const auto& [weaponId, count] : mag.currentAmmo) {
            if (count <= 0) continue;

            const MissileStats& m = TacticalDatabase::GetMissile(weaponId);

            if (needInterceptor && m.isInterceptor) return weaponId;
            if (!needInterceptor && !m.isInterceptor) return weaponId;
        }

        // Fallback: use anything available if no ideal weapon
        for (const auto& [weaponId, count] : mag.currentAmmo) {
            if (count > 0) return weaponId;
        }

        return ""; // No ammo at all
    }

    // Check if we already have a missile heading toward this threat position
    static int CountActiveShotsAgainst(const EngagementLog& log, const RadarTrack& threat, float currentSimTime) {
        int count = 0;
        for (const auto& eng : log.current) {
            if (!eng.missileAlive) continue;
            float timeSinceFired = currentSimTime - eng.timeFiredSec;
            MathUtils::Vec2 predictedEngPos = MathUtils::Add(eng.targetPos, MathUtils::Scale(eng.targetVel, timeSinceFired));
            float dist = MathUtils::GetDistance(predictedEngPos, threat.pos);
            if (dist < DATALINK_ENGAGEMENT_CORRELATION_RADIUS_NM) count++;
        }
        return count;
    }

    // How many missiles to fire at this threat?
    // Based on estimated probability of kill and threat importance
    static int DetermineSalvoSize(const RadarTrack& threat) {
        switch (threat.classification) {
            case ThreatClass::MISSILE_INBOUND:
                // Inbound missiles get two shots immediately - too important to risk
                // Pk per shot ~0.7, two shots gives ~0.91 Pk
                if (threat.timeToImpactSec < 60.0f) return 2; // Danger close - ripple fire
                return 1; // Time to assess first shot
            case ThreatClass::AIRCRAFT:
                return 1; // One shot, reassess
            case ThreatClass::SURFACE_SHIP:
                return 1; // Shoot-look-shoot
            default:
                return 1;
        }
    }

    // Extract the actual launch logic from CombatSystem into its own function
    static entt::entity LaunchMissile(
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

        MathUtils::Vec2 initialHeading = { 1.0f, 0.0f }; // fallback
        MathUtils::Vec2 toTarget = MathUtils::Sub(target.pos, shooterTransform.pos);
        float distToTarget = MathUtils::Length(toTarget);
        if (distToTarget > 0.001f) {
            initialHeading = MathUtils::Scale(toTarget, 1.0f / distToTarget);
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
            0.0f, // Time since launch (Seconds)
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

        float baseReliability = 0.90f; // Todo: Pull this from JSON file
        registry.emplace<Warhead>(missile,
            mStats.warheadYield, mStats.lethalRadiusNM,
            mStats.lethalRadiusNM * mStats.lethalRadiusNM,
            baseReliability, weaponId, shooter);

        registry.emplace<RadarSignature>(missile, mStats.rcs, mStats.rcsFourthRoot);
        registry.emplace<IFF>(missile, shooterIFF.isHostile);

        MetricsLogger::Log(GetSimTime(registry), "FIRE", (uint32_t)shooter, (int)target.classification, weaponId, distToTarget, "IN_FLIGHT");

        return missile;
    }
};