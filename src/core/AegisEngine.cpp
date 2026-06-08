#include "AegisEngine.h"
#include <iostream>
#include "GeoEngine.h"
#include "../ecs/components.h"
#include "../ecs/Systems.h"
#include "../graphics/MapRenderer.h"
#include "../models/TacticalData.h"
#include "../utils/TacticalDataLoader.h"
#include "../utils/ScenarioParser.h"
#include "../utils/MathUtils.h"

bool AegisEngine::Initialize() {
    InitWindow(1920, 1080, "AEGIS - TAIWAN THEATER");
    SetTargetFPS(60);

    camera.zoom = 1.0f;
    camera.offset = {1920.0f / 2, 1080.0f / 2};

    std::cout << "Loading tactical database..." << std::endl;
    TacticalDataLoader::Load("../data/units.json");

    std::cout << "Loading scenario from JSON..." << std::endl;
    currentScenario = ScenarioParser::Load("../scenarios/taiwan_strait.json");
    if (currentScenario.scenarioName.empty()) {
        std::cerr << "CRITICAL: Failed to load scenario file." << std::endl;
        return false;
    }

    registry.ctx().emplace<MapProjection>();
    registry.ctx().emplace<MapRenderer>();
    registry.ctx().emplace<RadarDetectionData>();

    registry.ctx().get<MapProjection>().LoadheightMap(currentScenario.heightMap.filepath);
    registry.ctx().get<MapRenderer>().LoadAssets(currentScenario);

    SpawnScenarioUnits();

    return true;
}

void AegisEngine::SpawnScenarioUnits() {
    auto& map = registry.ctx().get<MapProjection>();

    for (const auto& group : currentScenario.groups) {
        if (group.formation == "grid") {
            const ShipStats& stats = TacticalDatabase::GetShip(group.unitType);

            // --- Calculate Optimal Standoff based on longest-range weapon ---
            float maxWeaponRangeNM = 0.0f;
            for (const auto& [weaponId, count] : stats.loadout) {
                const MissileStats& mStats = TacticalDatabase::GetMissile(weaponId);
                if (mStats.maxRangeNM > maxWeaponRangeNM) {
                    maxWeaponRangeNM = mStats.maxRangeNM;
                }
            }
            float optimalStandoff = maxWeaponRangeNM * 0.95f;

            // ===============================================
            // === THE SPHERICAL TO FLAT-EARTH TRANSLATION ===
            // ===============================================

            // Translate the Origin: Lat/Lon -> Cartesian NM
            MathUtils::Vec2 centerNM = map.GeoToNM(group.centerLat, group.centerLon);

            // Translate all Waypoints: Lat/Lon -> Cartesian NM
            std::vector<MathUtils::Vec2> waypointsNM;
            for (auto wpGeo : group.waypointsGeo) {
                waypointsNM.push_back(map.GeoToNM(wpGeo.x, wpGeo.y));
            }

            int cols = MathUtils::CalculateGridColumns(group.count);

            // ============================================
            // === THE ECS SPAWNER (Pure Reality Units) ===
            // ============================================
            for (int i = 0; i < group.count; ++i) {
                int row = i / cols;
                int col = i % cols;

                float startX = centerNM.x + (col * group.spacingNM);
                float startY = centerNM.y + (row * group.spacingNM);

                auto entity = registry.create();
                float randomStartTimer = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * stats.radarScanRateSec;

                registry.emplace<Transform2D>(entity,
                    MathUtils::Vec2{startX, startY},              // Location (Cartesian NM)
                    stats.radarMastHeight,                        // Altitude (Meters)
                    0.0f                                          // Heading (Degrees)
                );

                registry.emplace<Kinematics>(entity,
                     MathUtils::Vec2{0.0f, 0.0f},           // Starting velocity (NM/sec)
                     MathUtils::Vec2{0.0f, 0.0f},           // Starting heading vector
                     stats.maxSpeedKnots,                         // Max Speed (Knots)
                     0.0f,                                        // Current Speed (Knots)
                     0.0f,                                        // Desired Speed (Knots)
                     5.0f                                         // Acceleration Rate (Knots/sec)
                 );

                registry.emplace<AutonomousGuidance>(entity,
                    group.initialState,                           // Initial State
                    optimalStandoff,                           // Standoff Range (NM)
                    waypointsNM,                              // Translated Flat Waypoints (NM)
                    static_cast<size_t>(0)                        // Current Waypoint Index
                    );

                registry.emplace<Magazine>(entity,
                    stats.loadout,                                // Weapons / Ammo Count
                    0.0f                                          // Weapon Firing Cooldown
                );

                registry.emplace<RadarEmitter>(entity,
                    stats.radarRangeNM,                           // Range (Nautical Miles)
                    stats.radarRangeNM * stats.radarRangeNM,      // Physics Cache (NM Squared)
                    stats.radarScanRateSec,                       // Scan Rate(Seconds)
                    randomStartTimer                             // Time Since Last Scan (Randomized when initialized)
                );

                registry.emplace<RadarSignature>(entity,
                    stats.rcs,                                    // Radar Cross-Section (sqm)
                    stats.rcsFourthRoot                           // RCS Fourth Root
                    );

                registry.emplace<IFF>(entity,
                    group.isHostile                              // Friendly / Hostile
                );
                registry.emplace<Hull>(entity, 100.0f);
            }
            std::cout << "Spawned " << group.count << " " << stats.className << "s." << std::endl;
        }
    }
}

void AegisEngine::ProcessInput() {
    // Time Compression
    if (IsKeyPressed(KEY_UP)) timeScale *= 10.0f;
    if (IsKeyPressed(KEY_DOWN)) timeScale /= 10.0f;
    if (timeScale < 1.0f) timeScale = 1.0f;

    // Toggle Radar Rings
    if (IsKeyPressed(KEY_R)) showRadarRings = !showRadarRings;

    // Camera Zoom
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
        camera.offset = GetMousePosition();
        camera.target = mouseWorldPos;
        camera.zoom += wheel * 0.1f;
        if (camera.zoom < 0.25f) camera.zoom = 0.25f;
    }

    // Camera Pan
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 delta = GetMouseDelta();
        float scale = -1.0f / camera.zoom;
        delta.x *= scale;
        delta.y *= scale;
        camera.target.x += delta.x;
        camera.target.y += delta.y;
    }
}

void AegisEngine::Update(float deltaTime) {
    Systems::NavigationSystem(registry, deltaTime);
    Systems::CombatSystem(registry, deltaTime);
    Systems::MovementSystem(registry, deltaTime);
    Systems::RadarSystem(registry, deltaTime);
    Systems::ProximityFuseSystem(registry);

    auto deadEntities = registry.view<DeadTag>();
    registry.destroy(deadEntities.begin(), deadEntities.end());
}

void AegisEngine::Render() {
    BeginDrawing();
    ClearBackground({ 10, 20, 30, 255 }); // Ocean Blue

    BeginMode2D(camera);

    // Draw Map
    registry.ctx().get<MapRenderer>().DrawLayer(currentScenario);

    // Draw Entities & Hover UI
    auto renderView = registry.view<Transform2D, IFF>();
    Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), camera);
    MathUtils::Vec2 mPos = { mouseWorld.x, mouseWorld.y };

    auto& map = registry.ctx().get<MapProjection>();

    for (auto entity : renderView) {
        auto& transform = renderView.get<Transform2D>(entity);
        auto& iff = renderView.get<IFF>(entity);

        // TRANSLATE WORLD SPACE (NM) TO SCREEN SPACE (PIXELS)
        MathUtils::Vec2 screenPos = map.WorldToScreen(transform.pos);

        if (registry.all_of<Warhead>(entity)) {
            DrawCircleV({screenPos.x, screenPos.y}, 10000.0f, ORANGE);
            DrawCircleLines(screenPos.x, screenPos.y, 6.0f, RED); // Box them so they're obvious
        } else {
            Color c = iff.isHostile ? RED : BLUE;
            DrawCircleV({screenPos.x, screenPos.y}, 5.0f, c);
        }

        bool isHovered = MathUtils::GetDistance(mPos, screenPos) < 10.0f;

        // Hover UI
        auto* kin = registry.try_get<Kinematics>(entity);
        if (isHovered && kin && !registry.all_of<Warhead>(entity)) {
            const char* tooltip = TextFormat("Alt: %.1fm\nSpd: %.1f kts", transform.altitude, kin->currentSpeedKnots);
            DrawText(tooltip, screenPos.x + 10, screenPos.y - 20, 20, GREEN);
        }

        auto* radar = registry.try_get<RadarEmitter>(entity);
        if (radar) {
            float radarRadiusPx = MathUtils::NmToPixels(radar->rangeNM);
            if (showRadarRings || isHovered) {
                Color c = iff.isHostile ? RED : BLUE;
                DrawCircleLines(screenPos.x, screenPos.y, radarRadiusPx, Fade(c, 0.5f));
            }
        }
    }

    auto& detectionData = registry.ctx().get<RadarDetectionData>();

    for (const auto& [observerId, targetList] : detectionData.activeTracks) {
        auto observerEntity = static_cast<entt::entity>(observerId);

        if (registry.valid(observerEntity)) {
            auto& obsTransform = registry.get<Transform2D>(observerEntity);
            MathUtils::Vec2 obsPx = map.WorldToScreen(obsTransform.pos);

            for (const auto& targetTrack : targetList) {
                MathUtils::Vec2 tgtPx = map.WorldToScreen(targetTrack.pos);

                DrawLine(
                    static_cast<int>(obsPx.x), static_cast<int>(obsPx.y),
                    static_cast<int>(tgtPx.x), static_cast<int>(tgtPx.y),
                    Fade(YELLOW, 0.15f)
                );
            }
        }
    }

    EndMode2D();

    // Draw Static Screen UI
    DrawFPS(10, 10);
    DrawText(currentScenario.scenarioName.c_str(), 10, 40, 20, LIGHTGRAY);

    const char* timeText = TextFormat("TIME COMPRESSION: %.0fx", timeScale);
    DrawText(timeText, 10, 70, 20, YELLOW);

    EndDrawing();
}

void AegisEngine::Run() {
    while (!WindowShouldClose()) {
        ProcessInput();

        // Calculate physics time step based on our timeScale
        float dt = GetFrameTime() * timeScale;

        Update(dt);
        Render();
    }
}

void AegisEngine::Shutdown() {
    // Unload map assets before destroying the window
    registry.ctx().get<MapRenderer>().UnloadAssets();
    CloseWindow();
}
