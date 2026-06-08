#pragma once
#include "raylib.h"
#include <unordered_map>
#include <string>
#include "../models/ScenarioData.h"

class MapRenderer {
public:
    // This runs automatically when the program closes(Destructor).
    ~MapRenderer() {
        UnloadAssets();
    }
    bool showElevation = false;

    // Pass the ScenarioData into the load and draw functions
    void LoadAssets(const ScenarioData& scenario);
    void UnloadAssets();
    void DrawLayer(const ScenarioData& scenario);

private:
    // -------------------------
    // --- Load All Textures ---
    // -------------------------
    std::unordered_map<std::string, Texture> tileTextures;

    // Height map
    Texture2D heightMapTexture = {};

    // --- Helper for cropping ---
    void DrawCropped(const Texture2D &tex, float x, float y, float cropL, float cropR, float cropT, float cropB);
};