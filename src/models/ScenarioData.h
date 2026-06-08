#pragma once
#include <string>
#include <vector>
#include "../ecs/components.h"

struct MapTile {
    std::string id;
    std::string filepath;
    float offsetX = 0.0f, offsetY = 0.0f;
    float cropL = 0.0f, cropR = 0.0f, cropT = 0.0f, cropB = 0.0f;
};

struct HeightMapData {
    std::string filepath;
    float offsetX = 0.0f, offsetY = 0.0f, scale = 1.0f; // Scale defaults to 1.0
};

struct EntityGroupData {
    std::string name;
    std::string unitType;
    int count = 0;
    float centerLat = 0.0f, centerLon = 0.0f;
    std::string formation;
    float spacingNM = 0.0f;
    bool isHostile = false;
    TacticalState initialState = TacticalState::INTERCEPT; // Default to hunting
    std::vector<MathUtils::Vec2> waypointsGeo;
};

struct ScenarioData {
    std::string scenarioName;
    HeightMapData heightMap = {};
    std::vector<MapTile> visualTiles;
    std::vector<EntityGroupData> groups;
};
