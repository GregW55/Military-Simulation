#pragma once

// Game Engine
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

constexpr float LOS_CHECK_STEP_SIZE_PX = 5.0f;

// Track
constexpr float TRACK_CORRELATION_GATE_PX = 15.0f;
constexpr float TRACK_STALE_TIMEOUT_SEC = 5.0f; // todo:  5 Seconds for now, Change to 10-30 seconds later, dont fire at the track until we physically detect it again
constexpr float TRACK_FILTER_ALPHA_MISSILES = 0.85f;
constexpr float TRACK_FILTER_ALPHA_SHIPS = 0.5f;

// Datalink
constexpr float DATALINK_TIMEOUT_SEC = 5.0f;
constexpr float DATALINK_MISSILE_CORRELATION_PX = 50.0f;
constexpr float DATALINK_ENGAGEMENT_CORRELATION_RADIUS_NM = 2.0f;

// Missile
constexpr float SEEKER_ACTIVATION_SPEED_KNOTS = 400.0f;
constexpr float SEEKER_ACTIVATION_DISTANCE = 15.0f;
constexpr float SEEKER_FOV_DEGREES = 45.0f;
constexpr float TERMINAL_DIVE_RANGE_PX = 200.0f;
constexpr float MISSILE_INTERCEPT_RANGE_PX = 15.0f;
constexpr float WAYPOINT_REACHED_DISTANCE_PX = 20.0f;
constexpr float FUSE_DISTANCE_INTERCEPTOR_PX = 5.0f;
constexpr float FUSE_DISTANCE_ANTISHIP_PX = 10.0f;

// Combat Doctrine
constexpr float RETREAT_THRESHOLD_KTS = -2.0f;

// Physics
// Standard atmospheric refraction constant for Radar Horizon in Nautical Miles
// accounts for radar waves bending slightly with Earth's curvature.
constexpr float EARTH_REFRACTION_FACTOR = 1.23f;

constexpr float DRAG_COEFF_SUBSONIC = 0.2f;
constexpr float DRAG_COEFF_TRANSONIC = 0.45f;
constexpr float DRAG_COEFF_SUPERSONIC = 0.3f;