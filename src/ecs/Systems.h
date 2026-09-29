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
    static float GetSimTime(entt::registry& registry);

    // --- NAVIGATION SYSTEM  ---
    static void NavigationSystem(entt::registry& registry,float deltaTime);

    // --- MOVEMENT SYSTEM ---
    static void MovementSystem(entt::registry& registry, float deltaTime);

    // --- RADAR ---
    static void RadarSystem(entt::registry& registry, float deltaTime);

    // --- COMBAT SYSTEM ---
    static void CombatSystem(entt::registry& registry, float deltaTime);

    static void ProximityFuseSystem(entt::registry& registry);
};