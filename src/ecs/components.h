#pragma once
#include "../utils/MathUtils.h"
#include "../models/TacticalData.h"
#include"../external/entt.hpp"

enum class TacticalState {
    TRANSIT,
    PATROL,
    INTERCEPT,
    STANDOFF,
    DEFEND
};

enum class GuidancePhase {
    MIDCOURSE_DATALINK,
    TERMINAL_PITBULL,
    INERTIAL_BLIND
};

struct DeadTag {};

// Where is it in the world?
struct Transform2D {
    MathUtils::Vec2 pos;
    float altitude;
    float heading;
};

// How is it moving?
struct Kinematics {
    MathUtils::Vec2 velocity = {0.0f, 0.0f};
    MathUtils::Vec2 headingVector = {0.0f, 0.0f};
    float maxSpeedKnots = 0.0f;
    float currentSpeedKnots = 0.0f;
    float desiredSpeedKnots = 0.0f;
    float accelerationRate = 0.0f;
    bool isDead = false;
};

struct Aerodynamics {
    float dryMassKg = 0.0f;
    float inverseMass = 0.0f;
    float areaM2 = 0.0f;
    float thrustNewtons = 0.0f;
    float currentFuelKg = 0.0f;
    float burnRateKgSec = 0.0f;
    float cruiseAltitudeMeters = 0.0f;

    // --- Performance Optimization Cache ---
    float lastCachedAltitude = -999.0f;
    float cachedAirDensity = 0.0f;
    float cachedSpeedOfSound = 0.0f;
};

struct FlightData {
    float maxFuelSeconds = 0.0f;
    float currentFlightTime = 0.0f;
};

struct Warhead {
    float yieldDamage = 0.0f;
    float lethalRadiusNM = 0.0f;
    float lethalRadiusNmSq = 0.0f;
};

struct Hull {
    float currentHP = 100.0f;
};
// Who is it? (Identification Friend or Foe)
struct IFF {
    bool isHostile = true;
};

// Can it see?
struct RadarEmitter {
    float rangeNM = 0.0f;
    float rangeNmSq = 0.0f;
    float scanRateSec = 2.0f;
    float timeSinceLastScan = 0.0f;
};

// Does it have a radar signature?
struct RadarSignature {
    float rcs = 0.0f;  // Radar Cross-Section (m^2)
    float rcsFourthRoot = 0.0f;
};

// A pure physics return from a radar bounce
struct RadarTrack {
    MathUtils::Vec2 pos;
    MathUtils::Vec2 vel;
    float ageSec = 0.0f;
};

// Struct to hold successful radar pings
struct RadarDetectionData {
    // Map of [Observer ID] -> [List of Target Coordinates]
    std::unordered_map<uint32_t, std::vector<RadarTrack>> activeTracks;
};

struct SeekerHead {
    entt::entity datalinkSource { entt::null };
    SeekerType type;
    float rangeNM = 0.0f;
    GuidancePhase phase { GuidancePhase::MIDCOURSE_DATALINK };
    MathUtils::Vec2 targetPos = { 0.0f, 0.0f };
    MathUtils::Vec2 targetVel = {0.0f, 0.0f};
};

// What weapons are currently loaded, and can we fire
struct Magazine {
    std::unordered_map<std::string, int> currentAmmo;
    float fireCooldown = 0.0f;
};

struct AutonomousGuidance {
    TacticalState currentState = TacticalState::INTERCEPT;
    float desiredStandoffNM = 0.0f;
    std::vector<MathUtils::Vec2> waypoints;
    size_t currentWaypointIndex = 0;
};

