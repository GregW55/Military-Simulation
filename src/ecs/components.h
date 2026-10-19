#pragma once
#include "../utils/MathUtils.h"
#include "../models/TacticalData.h"

// ============================
// === ENUMS & STATUS CODES ===
// ============================

enum class EngagementStatus {
    ReadyToEngage,
    TrackUnconfirmed,
    TrackStale,
    NoCompatibleWeapon,
    WeaponOnCooldown,
    OutOfRange,
    BelowMinRange,
    MaxShotsInFlight,
    AmmoReserveProtected,
    TargetAlreadyAssigned
};

enum class TacticalState {
    TRANSIT, PATROL, INTERCEPT, STANDOFF,  DEFEND
};

enum class GuidancePhase {
    MIDCOURSE_DATALINK, TERMINAL_PITBULL, INERTIAL_BLIND
};

enum class ThreatClass {
    UNKNOWN,
    SURFACE_SHIP,       // Slow, large RCS, at sea level
    AIRCRAFT,           // Fast, airborne, medium RCS
    MISSILE_INBOUND,    // Very fast, tiny RCS, closing rapidly, low altitude
    FRIENDLY            // IFF confirmed friendly
};

// ==============================
// === QUERY & RESULT STRUCTS ===
// ==============================
struct DeadTag {};

// =====================================
// === PHYSICAL & SPATIAL COMPONENTS ===
// =====================================

struct Transform2D {
    MathUtils::Vec2 pos;
    float altitude;
    float heading;
};

struct Kinematics {
    MathUtils::Vec2 velocity = {0.0f, 0.0f};
    MathUtils::Vec2 headingVector = {0.0f, 0.0f};
    float maxSpeedKnots = 0.0f;
    float currentSpeedKnots = 0.0f;
    float desiredSpeedKnots = 0.0f;
    float accelerationRate = 0.0f;
};

struct Aerodynamics {
    float dryMassKg = 0.0f;
    float inverseMass = 0.0f;
    float areaM2 = 0.0f;
    float thrustNewtons = 0.0f;
    float currentFuelKg = 0.0f;
    float burnRateKgSec = 0.0f;
    float cruiseAltitudeMeters = 0.0f;
    float maxManeuverGs = 0.0f;
    float desiredAltitudeMeters = 0.0f;

    // --- Performance Optimization Cache ---
    float lastCachedAltitude = -999.0f;
    float cachedAirDensity = 0.0f;
    float cachedSpeedOfSound = 0.0f;
};

// =====================================
// === SENSORS & TRACKING COMPONENTS ===
// =====================================
struct MasterTrackTable {
    uint32_t nextSystemTrackId = 1000;

    // Global master list of fused tracks available to the entire task force datalink
    struct SystemTrack {
        uint32_t systemTrackId;
        MathUtils::Vec2 fusedPos;
        MathUtils::Vec2 fusedVel;
        float altitude;
        ThreatClass classification;
        float confidence;
        float ageSec;
    };

    std::unordered_map<uint32_t, SystemTrack> activeSystemTracks;

    uint32_t AllocateTrackId() {
        return nextSystemTrackId++;
    }

    void Clear() {
        activeSystemTracks.clear();
    }
};

struct RadarEmitter {
    float rangeNM = 0.0f;
    float rangeNmSq = 0.0f;
    float scanRateSec = 2.0f;
    float timeSinceLastScan = 0.0f;
};

struct RadarSignature {
    float rcs = 0.0f;  // Radar Cross-Section (m^2)
    float rcsFourthRoot = 0.0f;
};

struct RadarTrack {
    uint32_t systemTrackId = 0;  // Linked to the MasterTrackTable ID
    MathUtils::Vec2 pos;
    MathUtils::Vec2 vel;
    float altitude = 0.0f;
    float ageSec = 0.0f;
    bool hasVelocity = false;
    bool isMissing = false;

    float speedKnots = 0.0f;                                        // Derived from vel magnitude each update
    float closingSpeedKnots = 0.0f;                                 // Positive = closing on observer, negative = opening
    float timeToImpactSec = -1.0f;                                  // -1 means not on intercept course
    ThreatClass classification = ThreatClass::UNKNOWN;
    float threatScore = 0.0f;                                       // Higher = more urgent
    float consistentObservationSec = 0.0f;
};

struct SharedThreatPicture {
    // Maps System Track ID -> Shooter Entity assigned to engage it
    std::unordered_map<uint32_t, entt::entity> trackAssignments;

    bool IsAssigned(uint32_t systemTrackId) const {
        return trackAssignments.find(systemTrackId) != trackAssignments.end();
    }

    void Assign(uint32_t systemTrackId, entt::entity shooter) {
        trackAssignments[systemTrackId] = shooter;
    }

    void Clear() { trackAssignments.clear(); }
};

struct RadarDetectionData {
    // Persistent tracks per observer
    std::unordered_map<entt::entity, std::vector<RadarTrack>> activeTracks;

    // Transient pings generated this frame by SensorSystem, consumed by TrackingSystem
    std::unordered_map<entt::entity, std::vector<RadarTrack>> framePings;
};

// =============================================
// === COMBAT, WEAPONS & DOCTRINE COMPONENTS ===
// =============================================

struct Hull {
    float currentHP = 100.0f;
};

struct IFF {
    bool isHostile = true;
};

struct Warhead {
    float yieldDamage = 0.0f;
    float lethalRadiusNM = 0.0f;
    float lethalRadiusNmSq = 0.0f;
    float baseReliability = 0.90f;
    std::string weaponId;
    entt::entity shooter { entt::null };
};

struct Magazine {
    std::unordered_map<std::string, int> maxAmmo;
    std::unordered_map<std::string, int> currentAmmo;
    float fireCooldown = 0.0f;
};

struct ActiveEngagement {
    uint32_t systemTrackId = 0;
    entt::entity missileEntity = entt::null;
    float timeFiredSec = 0.0f;
    bool missileAlive = true;
};

struct EngagementLog {
    std::vector<ActiveEngagement> current;
    int totalMissilesFired = 0;
};

// ================================
// === GUIDANCE & AI COMPONENTS ===
// ================================

struct FlightData {
    float maxFuelSeconds = 0.0f;
    float currentFlightTime = 0.0f;
};

struct SeekerHead {
    entt::entity shooter { entt::null };
    SeekerType type;
    float rangeNM = 0.0f;
    GuidancePhase phase { GuidancePhase::MIDCOURSE_DATALINK };
    uint32_t targetSystemTrackId = 0;
    MathUtils::Vec2 targetPos = { 0.0f, 0.0f };
    MathUtils::Vec2 targetVel = {0.0f, 0.0f};
    float targetAltitude = 0.0f;
    bool isInterceptor = false;
    float timeSinceLastCorrelation = 0.0f;
    float timeSinceLaunchSec = 0.0f;
    float maxLateralGs = 0.0f;
    bool hasLiveTarget = true;
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