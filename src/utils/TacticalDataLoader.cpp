#include "TacticalDataLoader.h"
#include "../models/TacticalData.h"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

void TacticalDataLoader::Load(const std::string &filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "CRITICAL ERROR: Could not open tactical data file: " << filepath << std::endl;
        return;
    }

    json j;
    try {
        file >> j;
    } catch (const json::parse_error& e) {
        std::cerr << "CRITICAL ERROR: units.json is malformed: " << e.what() << std::endl;
        return;
    }

    auto LoadFloat = [](const json& j, const std::string& key, float defaultVal, const std::string& unitName) {
        if (!j.contains(key)) {
            std::cerr << "[DATABASE WARNING] Unit '" << unitName
                      << " is missing '" << key
                      << "'. Defaulting to " << defaultVal << std::endl;
            return defaultVal;
        }
        return j[key].get<float>();
    };

    // Parse Missiles
    for (const auto& [key, val] : j["missiles"].items()) {
        MissileStats m;
        m.name = val.value("name", "MISSING_NAME");
        m.maxRangeNM = LoadFloat(val, "range_nm", 0.0f, key);
        m.maxSpeedKnots = LoadFloat(val, "speed_kts", 0.0f, key);

        std::string seekerStr = val.value("seeker_type", "PASSIVE");
        if (seekerStr == "ACTIVE") {
            m.seekerType = SeekerType::ACTIVE_RADAR;
        } else if (seekerStr == "SEMI_ACTIVE") {
            m.seekerType = SeekerType::SEMI_ACTIVE;
        } else {
            m.seekerType = SeekerType::PASSIVE_IR;
        }

        m.seekerRangeNM = LoadFloat(val, "seeker_range_nm", 0.0f, key);
        m.isInterceptor = val.value("is_interceptor", false); // Booleans can still use soft-loading
        m.rcs = LoadFloat(val, "rcs_sqm", 0.01f, key);
        m.rcsFourthRoot = std::pow(m.rcs, 0.25f);
        m.cruiseAltitudeMeters = LoadFloat(val, "cruise_altitude_m", 0.0f, key);

        // Physics
        m.accelerationRate = LoadFloat(val, "acceleration_kts_sec", 50.0f, key);
        m.massKg = LoadFloat(val, "mass_kg", 1500.0f, key);
        m.areaM2 = LoadFloat(val, "area_m2", 0.15f, key);
        m.thrustNewtons = LoadFloat(val, "thrust_newtons", 60000.0f, key);
        m.maxFuelKg = LoadFloat(val, "max_fuel_kg", 500.0f, key);
        m.burnRateKgSec = LoadFloat(val, "burn_rate_kg_sec", 25.0f, key);
        m.warheadYield = LoadFloat(val, "warhead_yield", 100.0f, key);
        m.lethalRadiusNM = LoadFloat(val, "lethal_radius_nm", 0.05f, key);

        // Save to the master database
        TacticalDatabase::missiles[key] = m;
    }

    // Parse Ships
    for (const auto& [key, val] : j["ships"].items()) {
        ShipStats s;
        s.className = val.value("name", "MISSING_NAME");
        s.radarRangeNM = val.value("radar_range_nm", 0.0f);
        s.radarScanRateSec = val.value("radar_scan_rate_sec", 2.0f);
        s.maxSpeedKnots = val.value("max_speed_kts", 0.0f);
        s.radarMastHeight = val.value("mast_height_m", 0.0f);
        s.rcs = val.value("rcs_sqm", 0.0f);
        s.rcsFourthRoot = std::pow(s.rcs, 0.25f);

        // Parse the dynamic loadout dictionary
        for (const auto& [weaponID, count] : val["loadout"].items()) {
            s.loadout[weaponID] = count;
        }

        // Save to the master database
        TacticalDatabase::ships[key] = s;
    }

}