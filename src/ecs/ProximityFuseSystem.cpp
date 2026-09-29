#include "Systems.h"

#include "Components.h"

#include "../core/Constants.h"
#include "../utils/MetricsLogger.h"

#include <cmath>
#include <iostream>
#include <random>

namespace {
    void ResolveShipDetonation(
        entt::registry& registry,
        entt::entity missile,
        entt::entity target,
        Warhead& warhead,
        Kinematics& tKin,
        RadarSignature& tSig,
        float distNmSq)
    {
        float dynamicPk = warhead.baseReliability;

        // Kinematic Evasion: Is the target moving fast?
        if (tKin.currentSpeedKnots > 1400.0f) {
            dynamicPk *= 0.25;
        } else if (tKin.currentSpeedKnots > 750.0f) {
            dynamicPk *= 0.4f;
        } else if (tKin.currentSpeedKnots > 350.0f) {
            dynamicPk *= 0.75f;
        } else if (tKin.currentSpeedKnots > 35.0f) {
            dynamicPk *= 0.95;
        }

        if (tSig.rcs < 1.0f) {
            dynamicPk *= 0.7f; // Small RCS degrades seeker lock
        }

        // Use a fallback RNG if the global one isn't injected yet, to prevent crashes
        std::mt19937 fallbackRng(std::random_device{}());
        auto& rng = registry.ctx().contains<std::mt19937>() ? registry.ctx().get<std::mt19937>() : fallbackRng;

        std::uniform_real_distribution<float> distRoll(0.0f, 1.0f);
        float roll = distRoll(rng);

        float distNM = std::sqrt(distNmSq);

        // -- DETRERMINE OUTCOME ---
        if (roll <= dynamicPk) {
            // HIT
            auto& hull = registry.get<Hull>(target);
            hull.currentHP -= warhead.yieldDamage;

            MetricsLogger::Log(Systems::GetSimTime(registry), "IMPACT", (uint32_t)warhead.shooter, 0, warhead.weaponId, distNM, "HIT");

            if (VERBOSE_COMBAT_LOG) {
                std::cout << "[T=" << Systems::GetSimTime(registry) << "s] [IMPACT] Missile HIT entity " << (uint32_t)target <<"\n";
            }

            if (hull.currentHP <= 0.0f) {
                registry.emplace_or_replace<DeadTag>(target);
            }
        } else {
            // MISS
            MetricsLogger::Log(Systems::GetSimTime(registry),
                "IMPACT", (uint32_t)warhead.shooter,
                0, warhead.weaponId, distNM, "MISS");

            if (VERBOSE_COMBAT_LOG) {
                std::cout << "[T=" << Systems::GetSimTime(registry) << "s] [IMPACT] Missile MISSED entity "
                << (uint32_t)target << " (Roll: " << roll << ", Required: " << dynamicPk << ")\n";
            }
        }

        registry.emplace_or_replace<DeadTag>(missile);
    }

    bool CheckShipTargets(
        entt::registry& registry,
        entt::entity missile,
        Transform2D& mTrans,
        Warhead& warhead,
        IFF& mIFF,
        const auto& shipTargets)
    {
        for (auto target : shipTargets) {
            auto& tTrans = shipTargets.template get<Transform2D>(target);
            auto& tIFF = shipTargets.template get<IFF>(target);
            auto& tKin = shipTargets.template get<Kinematics>(target);
            auto& tSig = shipTargets.template get<RadarSignature>(target);

            if (mIFF.isHostile == tIFF.isHostile) continue;

            // Fast-Fail Distance Check
            MathUtils::Vec2 diff = MathUtils::Sub(mTrans.pos, tTrans.pos);
            float distNmSq = MathUtils::LengthSq(diff);

            float altDiffMeters = std::abs(mTrans.altitude - tTrans.altitude);

            // DETONATION SEQUENCE
            if (distNmSq <= warhead.lethalRadiusNmSq && altDiffMeters <= FUSE_ALTITUDE_TOLERANCE_METERS) {
                ResolveShipDetonation(registry, missile, target, warhead, tKin, tSig, distNmSq);
                return true;
            }
        }
        return false;
    }

    bool CheckMissileTargets(
        entt::registry& registry,
        entt::entity missile,
        Transform2D& mTrans,
        Warhead& warhead,
        IFF& mIFF,
        const auto& missiles)
    {
        for (auto otherMissile : missiles) {
            if (otherMissile == missile) continue;
            if (registry.all_of<DeadTag>(otherMissile)) continue;

            auto& oIFF = missiles.template get<IFF>(otherMissile);
            if (mIFF.isHostile == oIFF.isHostile) continue;

            auto& oTrans = missiles.template get<Transform2D>(otherMissile);
            float distNmSq = MathUtils::LengthSq(MathUtils::Sub(mTrans.pos, oTrans.pos));
            float altDiffMeters = std::abs(mTrans.altitude - oTrans.altitude);

            if (distNmSq <= warhead.lethalRadiusNmSq && altDiffMeters <= FUSE_ALTITUDE_TOLERANCE_METERS) {
                auto& otherWarhead = missiles.template get<Warhead>(otherMissile);

                MetricsLogger::Log(Systems::GetSimTime(registry), "INTERCEPT", (uint32_t)warhead.shooter,
                    3, warhead.weaponId, std::sqrt(distNmSq), "KILLED_THREAT");
                MetricsLogger::Log(Systems::GetSimTime(registry), "SHOT_DOWN", (uint32_t)otherWarhead.shooter,
                    0, otherWarhead.weaponId, std::sqrt(distNmSq), "INTERCEPTED");

                registry.emplace_or_replace<DeadTag>(missile);
                registry.emplace_or_replace<DeadTag>(otherMissile);
                if (VERBOSE_COMBAT_LOG) {
                    std::cout << "[INTERCEPT] Missile " << (uint32_t)missile
                              << " destroyed missile " << (uint32_t)otherMissile << "\n";
                }

                return true;
            }
        }
        return false;
    }
}

void Systems::ProximityFuseSystem(entt::registry& registry) {
    auto missiles = registry.view<Transform2D, Warhead, IFF>();

    auto shipTargets = registry.view<Transform2D, Hull, IFF, Kinematics, RadarSignature>();

    for (auto missile : missiles) {
        if (registry.all_of<DeadTag>(missile)) continue;

        auto& mTrans = missiles.get<Transform2D>(missile);
        auto& warhead = missiles.get<Warhead>(missile);
        auto& mIFF = missiles.get<IFF>(missile);

        if (CheckMissileTargets(registry, missile, mTrans, warhead, mIFF, missiles)) continue;

        CheckShipTargets(registry, missile, mTrans, warhead, mIFF, shipTargets);



    }
}