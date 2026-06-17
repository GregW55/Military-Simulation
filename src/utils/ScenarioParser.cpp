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

    // Safely Map the Heightmap
    if (j.contains("map_data") && j["map_data"].contains("heightmap")) {
        const auto& hm = j["map_data"]["heightmap"];
        data.heightMap.filepath = hm.value("filepath", "");
        data.heightMap.offsetX = hm.value("offset_x", 0.0f);
        data.heightMap.offsetY = hm.value("offset_y", 0.0f);
        data.heightMap.scale = hm.value("scale", 1.0f);
    } else {
        std::cerr << "WARNING: No heightmap found in scenario." << std::endl;
    }

    // Safely Loop through the Map Tiles
    if (j.contains("map_data") && j["map_data"].contains("visual_tiles")) {
        for (const auto& tileJson : j["map_data"]["visual_tiles"]) {
            MapTile tile;
            tile.id = tileJson.value("id", "UNKNOWN_TILE");
            tile.filepath = tileJson.value("filepath", "");
            tile.offsetX = tileJson.value("offset_x", 0.0f);
            tile.offsetY = tileJson.value("offset_y", 0.0f);
            tile.cropL = tileJson.value("crop_l", 0.0f);
            tile.cropR = tileJson.value("crop_r", 0.0f);
            tile.cropT = tileJson.value("crop_t", 0.0f);
            tile.cropB = tileJson.value("crop_b", 0.0f);

            data.visualTiles.push_back(tile);
        }
    }

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