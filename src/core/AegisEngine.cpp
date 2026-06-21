#include "AegisEngine.h"
#include <random>

#include "GeoEngine.h"
#include "../ecs/components.h"
#include "../ecs/Systems.h"
#include "../graphics/MapRenderer.h"
#include "../models/TacticalData.h"
#include "../utils/TacticalDataLoader.h"
#include "../utils/ScenarioParser.h"
#include "../utils/MathUtils.h"

bool AegisEngine::Initialize(const bool headless, int seed) {
    isHeadless = headless;
    currentSeed = seed;

    TacticalDataLoader::Load("../data/units.json");

    // currentScenario = ScenarioParser::Load("../scenarios/taiwan_strait.json");
    currentScenario = ScenarioParser::Load("../scenarios/test_1v1.json");
    if (currentScenario.scenarioName.empty()) {
        return false;
    }

    registry.ctx().emplace<std::mt19937>(seed); // Seed Randomization
    registry.ctx().emplace<float>(0.0f);           // Sim Time In Seconds

    registry.ctx().emplace<MapProjection>();
    registry.ctx().emplace<MapRenderer>();
    registry.ctx().emplace<RadarDetectionData>();
    registry.ctx().emplace<SharedThreatPicture>();

    registry.ctx().get<MapProjection>().LoadheightMap(currentScenario.heightMap.filepath);

    if (!headless) {
        InitWindow(1920, 1080, "AEGIS - TAIWAN THEATER");
        SetTargetFPS(60);

        camera.zoom = 1.0f;
        camera.offset = {1920.0f / 2, 1080.0f / 2};

        registry.ctx().get<MapRenderer>().LoadAssets(currentScenario);
    }

    SpawnScenarioUnits();

    return true;
}

void AegisEngine::SpawnScenarioUnits() {
    auto& map = registry.ctx().get<MapProjection>();
    auto& rng = registry.ctx().get<std::mt19937>();

    for (const auto& group : currentScenario.groups) {
        if (group.formation == "grid") {
            const ShipStats& stats = TacticalDatabase::GetShip(group.unitType);

            float maxWeaponRangeNM = 0.0f;
            for (const auto &weaponId: stats.loadout | std::views::keys) {
                const MissileStats& mStats = TacticalDatabase::GetMissile(weaponId);
                if (mStats.maxRangeNM > maxWeaponRangeNM) {
                    maxWeaponRangeNM = mStats.maxRangeNM;
                }
            }

            float mastHeight = stats.radarMastHeight > 1.0f ? stats.radarMastHeight : 30.0f;

            float weaponLimitedStandoff = maxWeaponRangeNM * 0.95f;

            float ownHorizonNM = MathUtils::GetRadarHorizonNM(mastHeight);
            float assumedEnemyHorizonNM = MathUtils::GetRadarHorizonNM(mastHeight);
            float maxAchievableRangeNM = std::min(stats.radarRangeNM, ownHorizonNM + assumedEnemyHorizonNM);

            float radarLimitedStandoff = maxAchievableRangeNM * 0.90f;

            float optimalStandoff = std::min(weaponLimitedStandoff, radarLimitedStandoff);
            if (optimalStandoff < 10.0f) optimalStandoff = 10.0f;

            MathUtils::Vec2 centerNM = map.GeoToNM(group.centerLat, group.centerLon);

            std::vector<MathUtils::Vec2> waypointsNM;
            for (auto wpGeo : group.waypointsGeo) {
                waypointsNM.push_back(map.GeoToNM(wpGeo.x, wpGeo.y));
            }

            int cols = MathUtils::CalculateGridColumns(group.count);

            for (int i = 0; i < group.count; ++i) {
                int row = i / cols;
                int col = i % cols;

                float startX = centerNM.x + (col * group.spacingNM);
                float startY = centerNM.y + (row * group.spacingNM);

                auto entity = registry.create();
                std::uniform_real_distribution<float> dist(0.0f, stats.radarScanRateSec);
                float randomStartTimer = dist(rng);

                float maxSpeed = stats.maxSpeedKnots > 1.0f ? stats.maxSpeedKnots : 30.0f;

                registry.emplace<Transform2D>(entity,
                    MathUtils::Vec2{startX, startY},              // Location (Cartesian NM)
                    mastHeight,                                   // Altitude (Meters)
                    0.0f                                          // Heading (Degrees)
                );

                registry.emplace<Kinematics>(entity,
                     MathUtils::Vec2{0.0f, 0.0f},           // Starting velocity (NM/sec)
                     MathUtils::Vec2{0.0f, 0.0f},           // Starting heading vector
                     maxSpeed,                         // Max Speed (Knots)
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
                registry.emplace<EngagementLog>(entity);
                registry.emplace<Hull>(entity, 100.0f);
            }
        }
    }
}

void AegisEngine::ProcessInput() {
    if (IsKeyPressed(KEY_UP)) timeScale *= 10.0f;
    if (IsKeyPressed(KEY_DOWN)) timeScale /= 10.0f;
    if (timeScale < 1.0f) timeScale = 1.0f;
    if (timeScale > 10000.0f) timeScale = 10000.0f;

    if (IsKeyPressed(KEY_R)) showRadarRings = !showRadarRings;

    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
        camera.offset = GetMousePosition();
        camera.target = mouseWorldPos;
        camera.zoom += wheel * 0.1f;
        if (camera.zoom < 0.25f) camera.zoom = 0.25f;
    }

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
    ClearBackground({ 10, 20, 30, 255 });

    BeginMode2D(camera);

    registry.ctx().get<MapRenderer>().DrawLayer(currentScenario);

    auto renderView = registry.view<Transform2D, IFF>();
    Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), camera);
    MathUtils::Vec2 mPos = { mouseWorld.x, mouseWorld.y };

    auto& map = registry.ctx().get<MapProjection>();

    for (auto entity : renderView) {
        if (!registry.all_of<Warhead>(entity)) {
            auto& transform = renderView.get<Transform2D>(entity);
            auto& iff = renderView.get<IFF>(entity);
            MathUtils::Vec2 screenPos = map.WorldToScreen(transform.pos);

            Color c = iff.isHostile ? RED : BLUE;
            DrawCircleV({screenPos.x, screenPos.y}, 5.0f, c);

            bool isHovered = MathUtils::GetDistance(mPos, screenPos) < 10.0f;
            auto* kin = registry.try_get<Kinematics>(entity);
            if (isHovered && kin) {
                const char* tooltip = TextFormat("Alt: %.1fm\nSpd: %.1f kts", transform.altitude, kin->currentSpeedKnots);
                DrawText(tooltip, screenPos.x + 10, screenPos.y - 20, 20, GREEN);
            }

            if (auto* radar = registry.try_get<RadarEmitter>(entity)) {
                float radarRadiusPx = MathUtils::NmToPixels(radar->rangeNM);
                if (showRadarRings || isHovered) {
                    DrawCircleLines(screenPos.x, screenPos.y, radarRadiusPx, Fade(c, 0.5f));
                }
            }
        }
    }

    for (auto entity : renderView) {
        if (registry.all_of<Warhead>(entity)) {
            auto& transform = renderView.get<Transform2D>(entity);
            auto& kin = registry.get<Kinematics>(entity);
            MathUtils::Vec2 screenPos = map.WorldToScreen(transform.pos);

            // Draw a flame trail behind the missile
            MathUtils::Vec2 tailPos = MathUtils::Sub(transform.pos, MathUtils::Scale(kin.headingVector, 0.5f));
            MathUtils::Vec2 tailScreen = map.WorldToScreen(tailPos);
            DrawLineEx({screenPos.x, screenPos.y}, {tailScreen.x, tailScreen.y}, 2.0f, ORANGE);

            // Draw the warhead
            DrawCircleV({screenPos.x, screenPos.y}, 3.0f, YELLOW);
            DrawCircleLines(screenPos.x, screenPos.y, 5.0f, RED);
        }
    }

    auto& detectionData = registry.ctx().get<RadarDetectionData>();
    for (const auto& [observerEntity, targetList] : detectionData.activeTracks) {

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

    DrawFPS(10, 10);
    DrawText(currentScenario.scenarioName.c_str(), 10, 40, 20, LIGHTGRAY);
    const char* timeText = TextFormat("TIME COMPRESSION: %.0fx", timeScale);
    if (timeScale > 1000.0f) {
        DrawText("WARNING: PHYSICS MAY BE UNSTABLE", 10, 95, 18, RED);
    }
    DrawText(timeText, 10, 70, 20, YELLOW);

    EndDrawing();
}

void AegisEngine::Run() {
    auto& simTime = registry.ctx().get<float>();
    if (isHeadless) {
        while (simTime < 7200.0f) {
            constexpr float fixedDelta = 1.0f;
            Update(fixedDelta);
            simTime += fixedDelta;
        }
    } else {
        while (!WindowShouldClose()) {
            ProcessInput();
            float dt = GetFrameTime() * timeScale;
            simTime += dt;
            Update(dt);
            Render();
        }
    }
}

void AegisEngine::Shutdown() {
    registry.ctx().get<MapRenderer>().UnloadAssets();
    CloseWindow();
}