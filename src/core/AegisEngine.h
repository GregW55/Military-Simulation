#pragma once
#include "raylib.h"
#include "../../external/entt.hpp"
#include "../models/ScenarioData.h"
#include "../graphics/Renderer.h"

class AegisEngine {
public:
    // The three public lifecycle methods
    bool Initialize(bool headless, int seed);
    void Run();
    void Shutdown();

private:
    // Internal loop phases
    void Update(float deltaTime);

    // Internal helper to keep the spawner logic organized
    void SpawnScenarioUnits();

    // The core state of the simulation
    bool isHeadless = false;
    int currentSeed = 0;
    entt::registry registry;
    ScenarioData scenario;
    Renderer renderer;
};
