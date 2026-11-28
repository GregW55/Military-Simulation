#include "ScenarioParser.h"
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

ScenarioData ScenarioParser::Load(const std::string& filepath) {
    ScenarioData data;

    // Open the file
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "CRITICAL ERROR: Could not open scenario file: " << filepath << std::endl;
        return data;
    }

    // Parse the text into a JSON object safely
    json j;
    try {
        file >> j;
    } catch (json::parse_error& e) {
        std::cerr << "CRITICAL ERROR: JSON syntax is broken. " << e.what() << std::endl;
        return data;
    }

    // The Dictionary (String to Enum Mapper)
    static const std::unordered_map<std::string, TacticalState> stateMap = {
        {"TRANSIT", TacticalState::TRANSIT},
        {"PATROL", TacticalState::PATROL},
        {"INTERCEPT", TacticalState::INTERCEPT},
        {"STANDOFF", TacticalState::STANDOFF},
        {"DEFEND", TacticalState::DEFEND}
    };

    // Map the root variables (with safe defaults)
    data.scenarioName = j.value("scenario_name", "UNNAMED_SCENARIO");
    

    // Safely Loop through the Spawner Groups
    if (j.contains("groups")) {
        for (const auto& groupJson : j["groups"]) {
            EntityGroupData group;
            group.name = groupJson.value("name", "Unknown_Group");
            group.unitType = groupJson.value("unit_type", "UNKNOWN_UNIT");
            group.count = groupJson.value("count", 1);
            group.centerLat = groupJson.value("center_lat", 0.0f);
            group.centerLon = groupJson.value("center_lon", 0.0f);
            group.formation = groupJson.value("formation", "grid");
            group.spacingNM = groupJson.value("spacing_nm", 0.5f);
            group.isHostile = groupJson.value("is_hostile", true);

            // --- Enum Conversion Logic ---
            std::string stateStr = groupJson.value("initial_state", "INTERCEPT");

            if (stateMap.find(stateStr) != stateMap.end()) {
                group.initialState = stateMap.at(stateStr);
            } else {
                std::cerr << "WARNING: Unknown state '" << stateStr << "'. Defaulting to INTERCEPT." << std::endl;
                group.initialState = TacticalState::INTERCEPT;
            }

            // If the JSON group has waypoints, read them and save them to the group struct
            if (groupJson.contains("waypoints")) {
                for (const auto& wp : groupJson["waypoints"]) {
                    float wLat = wp.value("lat", 0.0f);
                    float wLon = wp.value("lon", 0.0f);
                    group.waypointsGeo.push_back({wLat, wLon});
                }
            }
            data.groups.push_back(group);
        }
    } else {
        std::cerr << "WARNING: No unit groups found in scenario." << std::endl;
    }

    return data;
}