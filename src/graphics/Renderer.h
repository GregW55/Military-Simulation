#pragma once
#include <string>
#include <vector>
#include "raylib.h"
#include "../models/ScenarioData.h"
#include "../core/MapProjection.h"

struct TerrainChunk {
    Model model{};
    Texture2D tex{};
};

class Renderer {
public:
    Renderer() = default;
    Renderer(const Renderer&) = delete;                 // would double-free GPU textures
    Renderer& operator=(const Renderer&) = delete;

    void Init(const ScenarioData& scenario);            // opens the window, loads the map textures
    void Shutdown();
    static bool ShouldClose() ;
    void ProcessInput();                                // zoom, pan, time compression, toggles
    void Draw(const entt::registry& registry, const ScenarioData& scenario);
    void BuildTerrain(const MapProjection& map, const std::string& dir);
    void UnloadTerrain();

    float TimeScale() const {return timeScale; }

private:
   // Pass the ScenarioData into the load and draw functions
    Camera3D camera{};
    Vector3 camTarget{0.0f, 0.0f, 0.0f};
    float camYaw = 0.0f, camPitch = 0.0f, camDist = 400.0f;
    std::vector<TerrainChunk> chunks;


    float timeScale = 1.0f;
    bool showRadarRings = false;
    // === 2D ===
    /* Camera2D camera = {};
     * std::unordered_map<std::string, Texture> tileTextures;
     * Texture2D heightMapTexture = {};
     * bool showElevation = false;
    */
};