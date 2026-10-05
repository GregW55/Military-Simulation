#pragma once

#include <entt.hpp>
#include "Events.h"
#include "components.h"

constexpr bool VERBOSE_COMBAT_LOG = true;

class Systems {
public:
    static float GetSimTime(entt::registry& registry);

    // --- NAVIGATION SYSTEM  ---
    static void NavigationSystem(entt::registry& registry, float deltaTime);

    static float PlanMissileAltitude(const Transform2D& trans, const Kinematics& kin,
        const Aerodynamics& aero, const SeekerHead& seeker);

    // --- MOVEMENT SYSTEM ---
    static void MovementSystem(entt::registry& registry, float deltaTime);

    // --- RADAR SYSTEM ---
    static void RadarSystem(entt::registry& registry, float deltaTime);

    // --- COMBAT SYSTEM ---
    static void CombatSystem(entt::registry& registry, float deltaTime);

    static void ProximityFuseSystem(entt::registry& registry);

    static void DestroyMissile(entt::registry& registry, entt::entity missile,
            MissileEventType reason, float rangeNM = 0.0f);

    // --- LOGGING SYSTEM ---
    static void LoggingSystem(entt::registry& registry);

    static void RecordMissileFired(entt::registry& registry, entt::entity shooter, entt::entity missile,
        int targetClass, const std::string& weaponId, float rangeNM);
};