#pragma once

// ==========================================
// 1. FUNDAMENTAL MATH & PHYSICS
// ==========================================
constexpr float MATH_PI = 3.14159265f;
constexpr float MATH_EPSILON = 0.001f;
constexpr float GRAVITY_MPS2 = 9.80665f;

// Atmospheric / Environmental
constexpr float EARTH_REFRACTION_FACTOR = 1.23f;                        // Standard refraction for Radar Horizon
constexpr float MAX_TRACKED_ACCEL_G = 45.0f;                            // Approx highest tactical acceleration in Gs

// Aerodynamic Drag Coefficients
constexpr float DRAG_COEFF_SUBSONIC = 0.2f;
constexpr float DRAG_COEFF_TRANSONIC = 0.45f;
constexpr float DRAG_COEFF_SUPERSONIC = 0.3f;


// ==========================================
// 2. UNIT CONVERSION FACTORS
// ==========================================
// Distance & Scale
constexpr float NM_TO_METERS = 1852.0f;
constexpr float METERS_TO_FT = 3.28084f;
constexpr float NM_PER_LAT_DEG = 60.0f;

// Speed & Time Rates
constexpr float SECONDS_PER_HOUR = 3600.0f;
constexpr float KNOTS_TO_NM_PER_SEC = 1.0f / SECONDS_PER_HOUR;          // Divide by 3600 (hrs to sec)
constexpr float NM_PER_SEC_TO_KNOTS = SECONDS_PER_HOUR;                 // Multiply by 3600 (sec to hrs)
constexpr float MPS_PER_KNOT = NM_TO_METERS / SECONDS_PER_HOUR;
constexpr float KNOTS_PER_MPS = SECONDS_PER_HOUR / NM_TO_METERS;

// Angles
constexpr float RAD_TO_DEG = 180.0f / MATH_PI;
constexpr float DEG_TO_RAD = MATH_PI / 180.0f;


// ==========================================
// 3. ENGINE, UI & MAP SCALING
// ==========================================
constexpr float targetFPS = 60.0f;
constexpr float fixedDelta = 1.0f / targetFPS;                          // One simulation step

constexpr int OFFSET_X = -580;
constexpr int OFFSET_Y = -470;
constexpr float RULER_THICKNESS = 3.0f;

// Screen/Map Pixels to Spatial Units
constexpr float PX_PER_DEG_LON = 365.28f;                               // Scale X
constexpr float PX_PER_DEG_LAT = -399.78f;                              // Scale Y
constexpr float PX_PER_NM = 399.78f / 60.0f;
constexpr float NM_PER_PX = 60.0f / 399.78f;
constexpr float PX_PER_KNOT_SEC = PX_PER_NM / SECONDS_PER_HOUR;


// ==========================================
// 4. TACTICAL TRACKS & CONTACTS
// ==========================================
constexpr float TRACK_MAX_ACCEL_NM_S2 = MAX_TRACKED_ACCEL_G * GRAVITY_MPS2 / NM_TO_METERS;
constexpr float TRACK_POSITION_TOLERANCE_NM = 0.02f;                    // Slop allowance for float precision

constexpr float MAX_PLAUSIBLE_SPEED_KNOTS = 4500.0f;
constexpr float TRACK_MAX_SPEED_NM_S = MAX_PLAUSIBLE_SPEED_KNOTS * KNOTS_TO_NM_PER_SEC;
constexpr float MAX_PLAUSIBLE_VEL_CHANGE_KNOTS = 800.0f;

// Timeouts & Freshness
constexpr float TRACK_DELETION_TIMEOUT_SEC = 5.0f;                      // Removed entirely after unrefreshed
constexpr float TRACK_ACTIONABLE_FRESHNESS_SEC = 2.0f;                  // Must be fresher than this to engage/lock


// ==========================================
// 5. DATALINK & NETWORKING
// ==========================================
constexpr float DATALINK_TIMEOUT_SEC = 3.0f;
constexpr float DATALINK_ENGAGEMENT_CORRELATION_RADIUS_NM = 0.5f;


// ==========================================
// 6. MISSILE GUIDANCE & DYNAMICS
// ==========================================
constexpr float LAUNCH_SPEED_FRACTION = 0.15f;                          // ~15% max speed off rail before thrust
constexpr float MIN_TIME_BEFORE_SEEKER_ACTIVATION_SEC = 0.75f;          // Brief separation/boost phase
constexpr float DESCENT_SAFETY_MARGIN_SEC = 2.5f;                       // Small buffer to avoid cutting it razor-thin

constexpr float SEEKER_FOV_DEGREES = 45.0f;
constexpr float SEEKER_IDENTITY_GATE_NM = 0.2f;
constexpr float FUSE_ALTITUDE_TOLERANCE_METERS = 50.0f;


// ==========================================
// 7. SHIPS & ENGAGEMENTS
// ==========================================
constexpr float MIN_ENGAGEMENT_RANGE_NM = 0.5f;


// ==========================================
// 8. THREAT CLASSIFICATION THRESHOLDS
// ==========================================
// Speed thresholds separating contact/threat types
constexpr float MAX_SHIP_SPEED_KTS = 50.0f;                             // Max speed before reclassifying to UNKNOWN
constexpr float MIN_AIRCRAFT_SPEED_KTS = 150.0f;                        // Threshold to reclassify as AIRCRAFT
constexpr float MIN_MISSILE_SPEED_KTS = 250.0f;                         // Threshold to reclassify as MISSILE