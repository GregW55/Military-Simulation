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
    FRIENDLY            // IFF confirmed friendly
};

enum class TrackLifecycle {
    TENTATIVE,          // Newly detected; not yet sufficiently confirmed
    CONFIRMED,          // Sufficient detections to establish a real track
    COASTING            // Previously confirmed, but currently not detected
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
    float verticalVelocityNMPerSec = 0.0f;
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
struct KalmanState3D {
    // State: x, y, z, vx, vy, vz
    // Position: NM
    // Velocity: NM/sec
    std::array<float, 6> x{};

    // 6x6 covariance matrix, row-major
    std::array<float, 36> P{};

    float timestampSec = 0.0f;
    uint32_t measurementCount = 0;
    bool initialized = false;
    float processNoiseAccelPSD = KALMAN_PSD_UNKNOWN;    // switched when the class is confirmed
};

struct RadarEmitter {
    // Radar configuration
    float maxRangeNM = 0.0f;                            // Maximum range the radar can report
    float maxRangeNmSq = 0.0f;                          // Maximum range the radar can report(squared)
    float scanRateSec = 2.0f;                           // Time between search updates

    // Scan timing
    float timeSinceLastScan = 0.0f;                     // Time since last scan

    // Measurement accuracy (standard deviations)
    float rangeErrorNM = 0.01f;
    float angleErrorRadians = 0.001f;
    float velocityErrorKnots = 5.0f;
};

struct RadarSignature {
    float rcsM2 = 0.0f;                                 // Radar Cross-Section (m^2)
    float rcsFourthRoot = 0.0f;
};

struct RadarMeasurement {
    float rangeNM;
    float azimuthRadians;
    float elevationRadians;
    float radialVelocityKnots;

    // Measurements standard deviations (1 sigma)
    float rangeSigmaNM = 0.0f;
    float azimuthSigmaRadians = 0.0f;
    float elevationSigmaRadians = 0.0f;
    float radialVelocitySigmaKnots = 0.0f;

    float timestampSec;
};

struct RadarTrack {
    // Global ID assigned by the tactical picture belonging to this observer's side. Zero means no global association
    uint32_t globalTrackId = NO_GLOBAL_TRACK;

    MathUtils::Vec2 latestPing{};           // Last observed position

    // Estimated state
    MathUtils::Vec2 pos{};
    MathUtils::Vec2 vel{};

    float altitude = 0.0f;
    float verticalVelocityNMPerSec = 0.0f;
    float ageSec = 0.0f;                    // Seconds since the last successful radar measurement
    float lastMeasurementTimeSec = 0.0f;    // Time of the most recent successful measurement
    bool hasVelocity = false;               // True after the filter has received at least two measurements
    bool isMissing = false;

    // Calculated kinematic/threat information.
    float speedKnots = 0.0f;
    float closingSpeedKnots = 0.0f;
    float timeToImpactSec = -1.0f;
    float threatScore = 0.0f;

    // Track existence is separate from object classification
    TrackLifecycle lifecycle = TrackLifecycle::TENTATIVE;

    // Five most recent completed radar scans
    std::array<uint8_t, 5> recentScanHits{};

    uint8_t scanHistoryCount = 0;
    uint8_t scanHistoryNext = 0;
    uint8_t missedScanCount = 0;

    // Object classification
    ThreatClass classification = ThreatClass::UNKNOWN;
    float consistentObservationSec = 0.0f;

    ThreatClass pendingClass = ThreatClass::UNKNOWN;
    float pendingSec = 0.0f;

    // Six-state EKF
    KalmanState3D kalman;
    float measurementSigmaNM = 0.0f;
};

struct SharedThreatPicture
{
    using AssignmentMap =
        std::unordered_map<uint32_t, entt::entity>;

    std::array<AssignmentMap, 2> trackAssignments;

    bool IsAssigned(
        bool isHostile,
        uint32_t globalTrackId) const
    {
        const size_t side =
            isHostile ? 1u : 0u;

        return trackAssignments[side].contains(
            globalTrackId
        );
    }

    void Assign(
        bool isHostile,
        uint32_t globalTrackId,
        entt::entity shooter)
    {
        const size_t side =
            isHostile ? 1u : 0u;

        trackAssignments[side][globalTrackId] =
            shooter;
    }

    void Clear()
    {
        for (auto& assignments : trackAssignments)
            assignments.clear();
    }
};

struct RadarDetectionData {
    // Persistent tracks per observer
    std::unordered_map<entt::entity, std::vector<RadarTrack>> activeTracks;

    // Transient pings generated this frame by SensorSystem, consumed by TrackingSystem
    std::unordered_map<entt::entity, std::vector<RadarMeasurement>> frameMeasurements;
};

struct MasterTrackTable
{
    struct GlobalTrack
    {
        RadarTrack track;
        float confidence = 1.0f;

        // Last measurement from each observer that was incorporated into this global track
        std::unordered_map<entt::entity, float> lastFusedMeasurementTime;
    };

    struct SideTable
    {
        std::unordered_map<uint32_t, GlobalTrack>
            activeGlobalTracks;

        // Each side starts its own ID sequence.
        uint32_t nextGlobalTrackId = 1001;

        uint32_t AllocateTrackId()
        {
            return nextGlobalTrackId++;
        }

        void Clear()
        {
            activeGlobalTracks.clear();
            nextGlobalTrackId = 1001;
        }
    };

    // Index 0 = IFF::isHostile false (Blue)
    // Index 1 = IFF::isHostile true  (Red)
    std::array<SideTable, 2> sides;

    static size_t GetSideIndex(bool isHostile)
    {
        return isHostile ? 1u : 0u;
    }

    SideTable& GetSide(bool isHostile)
    {
        return sides[GetSideIndex(isHostile)];
    }

    const SideTable& GetSide(bool isHostile) const
    {
        return sides[GetSideIndex(isHostile)];
    }

    void Clear()
    {
        for (auto& side : sides)
            side.Clear();
    }
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
    uint32_t targetGlobalTrackId = 0;
    SeekerType type = SeekerType::SEMI_ACTIVE;
    float rangeNM = 0.0f;
    GuidancePhase phase { GuidancePhase::MIDCOURSE_DATALINK };
    MathUtils::Vec2 targetPos = { .x = 0.0f, .y = 0.0f };
    MathUtils::Vec2 targetVel = {.x = 0.0f, .y = 0.0f};
    float targetAltitude = 0.0f;
    bool isInterceptor = false;
    float timeSinceLastCorrelation = 0.0f;
    float timeSinceLaunchSec = 0.0f;
    float maxLateralGs = 0.0f;
    bool hasLiveTarget = true;
    float targetVerticalVelocityNMPerSec = 0.0f;

    // Timestamp of the most recent target measurement accepted by this missiles guidance computer
    float lastTargetMeasurementTimeSec = -1.0f;
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