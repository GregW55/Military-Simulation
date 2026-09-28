#pragma once

// Engine
constexpr float fixedDelta = 1.0f;
constexpr int OFFSET_X = -580;
constexpr int OFFSET_Y = -470;
constexpr float RULER_THICKNESS = 3.0f;

// Conversions
constexpr float NM_PER_LAT_DEG = 60.0f;

constexpr float PX_PER_DEG_LON = 365.28f;  // Scale X
constexpr float PX_PER_DEG_LAT = -399.78f; // Scale Y

constexpr float MPS_PER_KNOT = 1852.0f / 3600.0f; // Approx 0.514444f
constexpr float KNOTS_PER_MPS = 3600.0f / 1852.0f; // Inverse: 1 Meter/Second = 1.94384... Knots

constexpr float MATH_EPSILON = 0.001f;
constexpr float MATH_PI = 3.14159265f;
constexpr float RAD_TO_DEG = 180.0f / MATH_PI;
constexpr float DEG_TO_RAD = MATH_PI / 180.0f;

// Track
constexpr float TRACK_ACQUISITION_GATE_NM = 1.25f;
constexpr float TRACK_MIN_CORRELATION_GATE_NM = 0.75f;
constexpr float TRACK_MAX_CORRELATION_GATE_NM = 0.75f;
// todo:  5 Seconds for now, Change to 10-30 seconds later, dont fire at the track until we physically detect it again
constexpr float TRACK_DELETION_TIMEOUT_SEC = 3.0f;   // track removed from activeTracks entirely after this long unrefreshed
constexpr float TRACK_ACTIONABLE_FRESHNESS_SEC = 2.0f; // track must be fresher than this to fire on / correlate a seeker lock onto
constexpr float MAX_PLAUSIBLE_VEL_CHANGE_KNOTS = 800.0f;

// Datalink
constexpr float DATALINK_TIMEOUT_SEC = 3.0f;
constexpr float DATALINK_ENGAGEMENT_CORRELATION_RADIUS_NM = 0.5f;

// Missile
constexpr float LAUNCH_SPEED_FRACTION = 0.15f; // leaves rail at ~15% of max speed, then accelerates under thrust
constexpr float SEEKER_FOV_DEGREES = 45.0f;
constexpr float SEEKER_IDENTITY_GATE_NM = 0.15f; // Anything lower may cause tracks to constantly be lost
constexpr float FUSE_ALTITUDE_TOLERANCE_METERS = 50.0f;
constexpr float MIN_TIME_BEFORE_SEEKER_ACTIVATION_SEC = 0.75f; // brief separation/boost phase
constexpr float DESCENT_SAFETY_MARGIN_SEC = 2.5f; // small buffer so it doesn't cut it razor-thin

// Ships
constexpr float MIN_ENGAGEMENT_RANGE_NM = 0.5f;

// Speed thresholds that separate contact/threat types
// Threat assessment used to determine what the entity is (It must be X thing, depending on how fast it achieves)
constexpr float MAX_SHIP_SPEED_KTS = 50.0f; // Maximum speed an entity could go before being reclassified as UNKNOWN
constexpr float MIN_MISSILE_SPEED_KTS = 250.0f; // Once an entity reaches this threshold its reclassified as MISSILE
constexpr float MIN_AIRCRAFT_SPEED_KTS = 150.0f; // Once an entity reaches this threshold its reclassified as AIRCRAFT

// Physics
// Standard atmospheric refraction constant for Radar Horizon in Nautical Miles
// accounts for radar waves bending slightly with Earth's curvature.
constexpr float EARTH_REFRACTION_FACTOR = 1.23f;

constexpr float DRAG_COEFF_SUBSONIC = 0.2f;
constexpr float DRAG_COEFF_TRANSONIC = 0.45f;
constexpr float DRAG_COEFF_SUPERSONIC = 0.3f;