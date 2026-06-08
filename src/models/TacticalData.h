#pragma once
#include <string>
#include <unordered_map>

enum class SeekerType {
    ACTIVE_RADAR,
    SEMI_ACTIVE,
    PASSIVE_IR
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