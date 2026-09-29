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
    static void CombatSystem(entt::registry& registry, float deltaTime) ;

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