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

enum class ThreatClass {
    UNKNOWN,
    SURFACE_SHIP,       // Slow, large RCS, at sea level
    AIRCRAFT,           // Fast, airborne, medium RCS
    MISSILE_INBOUND,    // Very fast, tiny RCS, closing rapidly, low altitude
    FRIENDLY            // IFF confirmed friendly(future)
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

    // -- Logging & PK Variables ---
    float baseReliability = 0.90f;          // Mechanical chance the missile doesn't just break
    std::string weaponId = "";              // Remembering what type of missile it is
    entt::entity shooter { entt::null }; // Remembering who fired it
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
    float altitude = 0.0f;
    float ageSec = 0.0f;
    float speedKnots       = 0.0f;   // Derived from vel magnitude each update
    float closingSpeedKnots = 0.0f;  // Positive = closing on observer, negative = opening
    float timeToImpactSec  = -1.0f;  // -1 means not on intercept course
    ThreatClass classification = ThreatClass::UNKNOWN;
    float threatScore      = 0.0f;   // Higher = more urgent
    float consistentObservationSec = 0.0f;
};

struct SharedThreatPicture {
    std::unordered_map<uint32_t, entt::entity> engagementsAssignments;

    // Simple position hash for correlation (1 NM resolution)
    static uint32_t HashPos(MathUtils::Vec2 pos) {
        int ix = static_cast<int>(pos.x);
        int iy = static_cast<int>(pos.y);
        return static_cast<uint32_t>((ix * 73856093) ^ (iy * 19349663));
    }

    bool IsAssigned(MathUtils::Vec2 threatPos) const {
        uint32_t key = HashPos(threatPos);
        auto it = engagementsAssignments.find(key);
        if (it == engagementsAssignments.end()) return false;
        return true;
    }

    void Assign(MathUtils::Vec2 threatPos, entt::entity shooter) {
        engagementsAssignments[HashPos(threatPos)] = shooter;
    }

    void Clear() { engagementsAssignments.clear(); }
};
// Struct to hold successful radar pings
struct RadarDetectionData {
    // Map of [Observer ID] -> [List of Target Coordinates]
    std::unordered_map<entt::entity, std::vector<RadarTrack>> activeTracks;
};

struct SeekerHead {
    entt::entity datalinkSource { entt::null };
    SeekerType type;
    float rangeNM = 0.0f;
    GuidancePhase phase { GuidancePhase::MIDCOURSE_DATALINK };
    MathUtils::Vec2 targetPos = { 0.0f, 0.0f };
    MathUtils::Vec2 targetVel = {0.0f, 0.0f};
    float targetAltitude = 0.0f;
    bool isInterceptor = false; // Determines terminal-phase altitude behavior
    float timeSinceLastCorrelation = 0.0f;
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
    MathUtils::Vec2 cachedTargetPos = {0.0f, 0.0f};
    float targetingCooldown = 0.0f;
    bool hasTarget = false;
    bool pendingEngagement = false;
    float reactionTimer = 0.0f;
};

struct ActiveEngagement {
    MathUtils::Vec2 targetPos;       // Where we were aiming when we fired
    MathUtils::Vec2 targetVel;       // Estimated velocity at time of firing
    entt::entity missileEntity;      // The specific missile we fired
    ThreatClass targetClass;         // What we thought it was
    float timeFiredSec = 0.0f;       // Simulation time when fired
    bool missileAlive = true;        // Track if our shot is still flying
};

struct EngagementLog {
    std::vector<ActiveEngagement> current;
    int totalMissilesFired = 0;      // For Doctrine decisions (ammo awareness)
};