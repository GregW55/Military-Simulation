#pragma once
#include "Components.h"
#include "../core/Constants.h"
#include "../core/Physics.h"
#include "../utils/ThreatAnalysis.h"
#include "../utils/MetricsLogger.h"
#include "Events.h"
#include <iostream>
#include <execution>
#include <random>

constexpr bool VERBOSE_COMBAT_LOG = true;

class Systems {
public:
    static float GetSimTime(entt::registry& registry);

    // --- NAVIGATION SYSTEM  ---
    static void NavigationSystem(entt::registry& registry,float deltaTime);

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