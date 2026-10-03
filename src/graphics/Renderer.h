#pragma once
#include <string>
#include <unordered_map>
#include "raylib.h"
#include "../../external/entt.hpp"
#include "../models/ScenarioData.h"

class Renderer {
public:
    Renderer() = default;
    Renderer(const Renderer&) = delete;                 // would double-free GPU textures
    Renderer& operator=(const Renderer&) = delete;

    void Init(const ScenarioData& scenario);            // opens the window, loads the map textures
    void Shutdown();
    static bool ShouldClose() ;
    void ProcessInput();                                // zoom, pan, time compression, toggles
    void Draw(entt::registry& registry, const ScenarioData& scenario);

    float TimeScale() const {return timeScale; }

private:
   // Pass the ScenarioData into the load and draw functions
    void LoadAssets(const ScenarioData& scenario);
    void UnloadAssets();
    void DrawLayer(const ScenarioData& scenario);
    void DrawCropped(const Texture2D &tex, float x, float y, float cropL, float cropR, float cropT, float cropB);

    std::unordered_map<std::string, Texture> tileTextures;
    Texture2D heightMapTexture = {};
    bool showElevation = false;

    Camera2D camera = { 0 };
    float timeScale = 1.0f;
    bool showRadarRings = false;
};