#pragma once
#include <future>
#include <string>
#include <vector>
#include "raylib.h"
#include "../models/ScenarioData.h"
#include "../core/MapProjection.h"

struct TerrainChunk {
    Model     model{};
    Texture2D base{};          // level 0, always loaded
    Texture2D extra{};         // the current higher-detail texture (id 0 if none)
    int       level = 0;       // which level the model is showing now
    bool      avail[5] = {};   // which level files exist on disk (level 4 exists only over Taiwan)
    Vector3   lo{}, hi{};      // bounding box in render space, used for camera distance
    int pending = -1;          // Level being decoded on a worker thread right now, -1 is none
};

struct PendingLoad {
    int chunk = 0;
    int level = 0;
    std::future<Image> image;
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
    void UpdateChunkLods();

    float TimeScale() const {return timeScale; }

private:
   // Pass the ScenarioData into the load and draw functions
    Camera3D camera{};
    Vector3 camTarget{0.0f, 0.0f, 0.0f};
    float camYaw = 0.0f, camPitch = 0.0f, camDist = 400.0f;
    std::vector<TerrainChunk> chunks;
    std::string terrainDir;
    std::vector<PendingLoad> inFlight;


    float timeScale = 1.0f;
    bool showRadarRings = false;

    std::unordered_map<std::string, Model> shipModels;
    std::unordered_map<std::string, Model> missileModels;

    void LoadUnitModels();
    void UnloadUnitModels();

    // === 2D ===
    /* Camera2D camera = {};
     * std::unordered_map<std::string, Texture> tileTextures;
     * Texture2D heightMapTexture = {};
     * bool showElevation = false;
    */
};