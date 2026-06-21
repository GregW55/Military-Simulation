#pragma once
#include "raylib.h"
#include "../../external/entt.hpp"
#include "../models/ScenarioData.h"

class AegisEngine {
public:
    // The three public lifecycle methods
    bool Initialize(bool headless, int seed);
    void Run();
    void Shutdown();

private:
    // Internal loop phases
    void ProcessInput();
    void Update(float deltaTime);
    void Render();

    // Internal helper to keep the spawner logic organized
    void SpawnScenarioUnits();

    // The core state of the simulation
    bool isHeadless = false;
    int currentSeed = 0;
    entt::registry registry;
    Camera2D camera = { 0 };
    ScenarioData currentScenario;
    float timeScale = 1.0f;
    bool showRadarRings = false;
};
