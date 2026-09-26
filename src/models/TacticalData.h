#pragma once
#include <string>
#include <unordered_map>
#include "../external/entt.hpp"

enum class SeekerType {
    ACTIVE_RADAR,
    SEMI_ACTIVE,
    PASSIVE_IR
};

enum class AircraftMission {
    CAP,        // Combat Air Patrol - defend the group
    STRIKE,     // Attack surface targets
    SEAD,       // Suppress enemy air defenses
    RETURNING,  // Bingo fuel, heading home
    ON_DECK     // Not airborne
};

// --- TACTICAL STRUCTS ---
struct MissileStats {
    std::string name;
    float maxRangeNM;
    float maxSpeedKnots;
    SeekerType seekerType;
    float seekerRangeNM;
    bool isInterceptor;
    float rcs;
    float rcsFourthRoot;
    float cruiseAltitudeMeters;

    // Physics
    float accelerationRate;
    float massKg;
    float areaM2;
    float thrustNewtons;
    float maxFuelKg;
    float burnRateKgSec;
    float warheadYield;
    float lethalRadiusNM;
    float maxLateralGs;
};

struct ShipStats {
    std::string className;
    float radarRangeNM;
    float radarScanRateSec;
    float maxSpeedKnots;
    float radarMastHeight;
    float rcs;
    float rcsFourthRoot;
    std::unordered_map<std::string, int> loadout; // <Weapon_ID, Count>
};

struct FlightDeck {
    int maxAircraft = 75;
    std::vector<entt::entity> aircraftOnDeck;
    std::vector<entt::entity> aircraftAirborne;
    float launchCooldownSec = 0.0f;
    float recoveryWindowSec = 0.0f;
};

struct Aircraft {
    float fuelKg = 0.0f;
    float fuelBurnRateKgSec = 0.0f;
    float bingoFuelKg = 0.0f;   // Minimum fuel to return to carrier
    entt::entity assignedCarrier { entt::null };
    AircraftMission currentMission = AircraftMission::CAP;
};

// --- MASTER DATABASE ---
class TacticalDatabase {
public:
    // Memory pools holding every unit stat in the simulation
    inline static std::unordered_map<std::string, MissileStats> missiles;
    inline static std::unordered_map<std::string, ShipStats> ships;

    // Fast Lookup Functions
    static const MissileStats& GetMissile(const std::string& id) {
        return missiles.at(id);
    }
    static const ShipStats& GetShip(const std::string& id) {
        return ships.at(id);
    }
};