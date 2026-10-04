#include "AegisEngine.h"
#include <random>

#include "Constants.h"
#include "GeoEngine.h"
#include "../ecs/components.h"
#include "../ecs/Systems.h"
#include "../models/TacticalData.h"
#include "../utils/TacticalDataLoader.h"
#include "../utils/ScenarioParser.h"
#include "../utils/MathUtils.h"
#include "../utils/MetricsLogger.h"
#include <vector>
#include <string>

namespace {
    // Reads a heightmap image into one byte per pixel (the red channel).
    // This is the only place terrain data touches raylib.
    bool LoadHeightMapPixels(const std::string& path, std::vector<uint8_t>& elevation, int& width, int& height) {
        Image image = LoadImage(path.c_str());
        if (image.width <= 0 || image.height <= 0) return false;

        Color* pixels = LoadImageColors(image);
        width = image.width;
        height = image.height;

        elevation.resize(static_cast<size_t>(width) * height);
        for (size_t i = 0; i < elevation.size(); ++i) elevation[i] = pixels[i].r;

        UnloadImageColors(pixels);
        UnloadImage(image);
        return true;
    }
}

bool AegisEngine::Initialize(const bool headless, int seed) {
    isHeadless = headless;
    currentSeed = seed;

    TacticalDataLoader::Load("../data/units.json");

    // scenario = ScenarioParser::Load("../scenarios/taiwan_strait.json");
    scenario = ScenarioParser::Load("../scenarios/test_1v1.json");

    if (scenario.scenarioName.empty()) return false;

    registry.ctx().emplace<std::mt19937>(seed); // Seed Randomization
    registry.ctx().emplace<float>(0.0f);           // Sim Time In Seconds

    registry.ctx().emplace<SimEvents>();
    registry.ctx().emplace<MapProjection>();
    registry.ctx().emplace<RadarDetectionData>();
    registry.ctx().emplace<SharedThreatPicture>();

    // Same offsets and scale the renderer draws the heightmap with, so what you see and what the radar uses can't drift apart.
    std::vector<uint8_t> elevation;
    int hmWidth = 0, hmHeight = 0;
    if (LoadHeightMapPixels(scenario.heightMap.filepath, elevation, hmWidth, hmHeight)) {
        registry.ctx().get<MapProjection>().SetHeightMap(std::move(elevation), hmWidth, hmHeight,
            scenario.heightMap.offsetX + OFFSET_X,
            scenario.heightMap.offsetY + OFFSET_Y,
            scenario.heightMap.scale);
    }

    if (!headless) renderer.Init(scenario);

    SpawnScenarioUnits();
    MetricsLogger::Initialize("metrics.csv", seed);

    return true;
}

void AegisEngine::SpawnScenarioUnits() {
    auto& map = registry.ctx().get<MapProjection>();
    auto& rng = registry.ctx().get<std::mt19937>();

    for (const auto& group : scenario.groups) {
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

void AegisEngine::Update(float deltaTime) {
    Systems::MovementSystem(registry, deltaTime);
    Systems::RadarSystem(registry, deltaTime);
    Systems::NavigationSystem(registry, deltaTime);
    Systems::CombatSystem(registry, deltaTime);
    Systems::ProximityFuseSystem(registry);
    Systems::LoggingSystem(registry);

    auto deadEntities = registry.view<DeadTag>();
    registry.destroy(deadEntities.begin(), deadEntities.end());
}

void AegisEngine::Run() {
    auto& simTime = registry.ctx().get<float>();

    if (isHeadless) {
        while (simTime < 7200.0f) {
            Update(fixedDelta);
            simTime += fixedDelta;

            // --- EARLY EXIT LOGIC ---
            int activeShips = 0;
            int activeMissiles = 0;

            for (auto entity : registry.view<Hull>()) {
                if (!registry.all_of<DeadTag>(entity)) activeShips++;
            }

            for (auto entity : registry.view<Warhead>()) {
                if (!registry.all_of<DeadTag>(entity)) activeMissiles++;
            }

            // If 1 or 0 ships are left, and no missiles are in the air, the battle is over
            if (activeShips <= 1 && activeMissiles == 0) {
                std::cout << "Combat concluded early at " <<simTime << " seconds." << std::endl;
                break;
            }
        }
    } else {
        while (!Renderer::ShouldClose()) {
            constexpr double FRAME_BUDGET_SEC = 0.010;  // spend at most -10ms per frame simulating; the rest is for drawing
            float accumulator = 0.0f;                   // simulated time we still owe the simulation

            renderer.ProcessInput();
            accumulator += GetFrameTime() * renderer.TimeScale();

            const double frameStart = GetTime();        // raylib: seconds since the window opened
            while (accumulator >= fixedDelta && (GetTime() - frameStart) < FRAME_BUDGET_SEC) {
                Update(fixedDelta);
                simTime += fixedDelta;
                accumulator -= fixedDelta;
            }
            if (accumulator >= fixedDelta) accumulator = 0.0f;

            renderer.Draw(registry, scenario);
        }
    }
}

void AegisEngine::Shutdown() {
    MetricsLogger::Shutdown();                  // closes the CSV
    if (!isHeadless) renderer.Shutdown();       // unloads textures and closes the window
}