#include "../core/Constants.h"
#include "../core/GeoEngine.h"
#include "../ecs/components.h"
#include "../utils/MathUtils.h"
#include <algorithm>
#include <limits>
#include <string>
#include <vector>
#include "Renderer.h"

#include <iostream>
// #include <cstdio> // For TraceLog

void Renderer::Init(const ScenarioData& scenario) {
    InitWindow(1920, 1080, "AEGIS - TAIWAN THEATER");
    SetTargetFPS(60);

    camera.zoom = 1.0f;
    camera.offset = {1920.0f / 2, 1080.0f / 2};

    LoadAssets(scenario);
}

void Renderer::Shutdown() {
    UnloadAssets();
    CloseWindow();
}

bool Renderer::ShouldClose() { return WindowShouldClose(); }

void Renderer::ProcessInput() {
    if (IsKeyPressed(KEY_UP)) timeScale *= 2.0f;
    if (IsKeyPressed(KEY_DOWN)) timeScale /= 2.0f;
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

void Renderer::Draw(const entt::registry& registry, const ScenarioData& scenario) {
    BeginDrawing();
    ClearBackground({ 10, 20, 30, 255 });

    BeginMode2D(camera);

    DrawLayer(scenario);

    auto renderView = registry.view<Transform2D, IFF>();
    Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), camera);
    MathUtils::Vec2 mPos = { mouseWorld.x, mouseWorld.y };

    auto& map = registry.ctx().get<MapProjection>();

    for (auto entity : renderView) {
        if (!registry.all_of<Warhead>(entity)) {
            auto& transform = renderView.get<Transform2D>(entity);
            auto& iff = renderView.get<IFF>(entity);
            auto* kin = registry.try_get<Kinematics>(entity);
            MathUtils::Vec2 screenPos = map.WorldToScreen(transform.pos);

            Color c = iff.isHostile ? RED : BLUE;
            DrawCircleV({screenPos.x, screenPos.y}, 5.0f, c);

            bool isHovered = MathUtils::GetDistance(mPos, screenPos) < 2.0f;

            if (isHovered && kin) {
                std::string tooltip = TextFormat("Alt: %.1fm\nSpd: %.1f kts", transform.altitude, kin->currentSpeedKnots);

                if (auto* mag = registry.try_get<Magazine>(entity)) {
                    std::vector<std::pair<std::string, int>> sortedAmmo(mag->currentAmmo.begin(), mag->currentAmmo.end());
                    std::sort(sortedAmmo.begin(), sortedAmmo.end());

                    tooltip += "\nAmmo:";
                    for (const auto& [weaponId, count] : sortedAmmo) {
                        tooltip += TextFormat("\n  %s: %d", weaponId.c_str(), count);
                    }
                }

                DrawText(tooltip.c_str(), screenPos.x + 10, screenPos.y - 20, 20, GREEN);
            }

            if (auto* radar = registry.try_get<RadarEmitter>(entity)) {
                if (showRadarRings || isHovered) {
                    auto sigTargets = registry.view<Transform2D, RadarSignature, IFF>();
                    entt::entity nearest = entt::null;
                    float nearestDistSq = std::numeric_limits<float>::max();

                    for (auto target : sigTargets) {
                        if (sigTargets.get<IFF>(target).isHostile == iff.isHostile) continue;
                        float d = MathUtils::LengthSq(MathUtils::Sub(transform.pos, sigTargets.get<Transform2D>(target).pos));
                        if (d < nearestDistSq) {
                            nearestDistSq = d;
                            nearest = target;
                        }
                    }

                    if (registry.valid(nearest)) {
                        auto& tgtTransform = sigTargets.get<Transform2D>(nearest);
                        auto& tgtSig = sigTargets.get<RadarSignature>(nearest);

                        float obsHorizon = MathUtils::GetRadarHorizonNM(transform.altitude);
                        float tgtHorizon = MathUtils::GetRadarHorizonNM(tgtTransform.altitude);
                        float effectiveRangeNM = std::min(radar->rangeNM * tgtSig.rcsFourthRoot, obsHorizon + tgtHorizon);
                        float effectiveRadiusPx = MathUtils::NmToPixels(effectiveRangeNM);

                        DrawCircleLines(screenPos.x, screenPos.y, effectiveRadiusPx, Fade(GREEN, 0.5f));

                        const char* rangeText = TextFormat("Eff. range vs nearest contact: %.1f nm", effectiveRangeNM);
                        DrawText(rangeText, screenPos.x + 10, screenPos.y - 40, 16, GREEN);
                    }
                }
            }
        }

        else if (registry.all_of<Warhead>(entity)) {
            auto& transform = renderView.get<Transform2D>(entity);
            auto& iff = renderView.get<IFF>(entity);
            auto* kin = registry.try_get<Kinematics>(entity);
            MathUtils::Vec2 screenPos = map.WorldToScreen(transform.pos);

            Color c = iff.isHostile ? RED : BLUE;

            // Draw a flame trail behind the missile
            MathUtils::Vec2 tailPos = MathUtils::Sub(transform.pos, MathUtils::Scale(kin->headingVector, 0.5f));
            MathUtils::Vec2 tailScreen = map.WorldToScreen(tailPos);
            DrawLineEx({screenPos.x, screenPos.y}, {tailScreen.x, tailScreen.y}, 2.0f, ORANGE);

            // Draw the warhead
            DrawCircleV({screenPos.x, screenPos.y}, 3.0f, c);
            DrawCircleLines(screenPos.x, screenPos.y, 5.0f, Fade(c, 0.6f));

            // --- HOVER & TARGET HIGHLIGHT LOGIC ---
            bool isHovered = MathUtils::GetDistance(mPos, screenPos) < 4.0f;
            if (isHovered && kin) {
                const char* tooltip = TextFormat("MISSILE\nAlt: %.1fm\nSpd: %.1f kts", transform.altitude, kin->currentSpeedKnots);
                DrawText(tooltip, static_cast<int>(screenPos.x + 10), static_cast<int>(screenPos.y - 20), 20, PURPLE);

                // Pull target position directly from SeekerHead
                if (auto* seeker = registry.try_get<SeekerHead>(entity)) {
                    MathUtils::Vec2 targetScreenPos = map.WorldToScreen(seeker->targetPos);

                    // Draw a purple rectangle box around the target destination/position
                    DrawRectangleLines(
                        static_cast<int>(targetScreenPos.x - 12),
                        static_cast<int>(targetScreenPos.y - 12),
                        24.0f, 24.0f, PURPLE
                    );

                    // Draw connecting line from missile to its target point
                    DrawLineEx(
                        {screenPos.x, screenPos.y},
                        {targetScreenPos.x, targetScreenPos.y},
                        1.5f, Fade(PURPLE, 0.7f)
                    );
                }
            }
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
    DrawText(scenario.scenarioName.c_str(), 10, 40, 20, LIGHTGRAY);
    const char* timeText = TextFormat("TIME COMPRESSION: %.0fx", timeScale);
    if (timeScale > 1000.0f) {
        DrawText("WARNING: PHYSICS MAY BE UNSTABLE", 10, 95, 18, RED);
    }
    DrawText(timeText, 10, 70, 20, YELLOW);

    EndDrawing();
}

void Renderer::LoadAssets(const ScenarioData &scenario) {
    // Loop through the JSON data and load every tile dynamically
    for (const auto& tile : scenario.visualTiles) {
        // .c_str() converts the c++ string to a C-string for Raylib
        tileTextures[tile.id] = LoadTexture(tile.filepath.c_str());
    }

    // Load Height Map from the JSON data
    Image heightMapImage = LoadImage(scenario.heightMap.filepath.c_str());
    ImageColorReplace(&heightMapImage, BLACK, BLANK);
    heightMapTexture = LoadTextureFromImage(heightMapImage);
    UnloadImage(heightMapImage);
}

void Renderer::UnloadAssets() {
    for (auto& pair : tileTextures) {
        UnloadTexture(pair.second);
    }
    tileTextures.clear();

    // Only unload if the texture was actually loaded (id != 0)
    if (heightMapTexture.id != 0) {
        UnloadTexture(heightMapTexture);
        heightMapTexture = {};  // Zero it out so a second call is safe
    }
}

void Renderer::DrawCropped(const Texture2D &tex, float x, float y,
                                float cropL, float cropR, float cropT, float cropB) {
    const Rectangle srcRect = {
        cropL,
        cropT,
        static_cast<float>(tex.width) - cropL - cropR,
        static_cast<float>(tex.height) - cropT - cropB
    };

    const Vector2 position = {
        x + cropL + OFFSET_X,
        y + cropT + OFFSET_Y
    };

    DrawTextureRec(tex, srcRect, position, WHITE);
}

void Renderer::DrawLayer(const ScenarioData &scenario) {
    // Draw every tile in the scenario dynamically
    for (const auto& tile : scenario.visualTiles) {
        Texture2D tex = tileTextures[tile.id]; // Look up the texture by its ID
        DrawCropped(tex, tile.offsetX, tile.offsetY, tile.cropL, tile.cropR, tile.cropT, tile.cropB);
    }

    // Height Map Logic
    if (IsKeyPressed(KEY_H)) {
        showElevation = !showElevation;
    }

    if (showElevation) {
        DrawTextureEx(heightMapTexture,
                       {scenario.heightMap.offsetX + OFFSET_X, scenario.heightMap.offsetY + OFFSET_Y},
                       0.0f,
                       scenario.heightMap.scale,
                       Fade(WHITE, 0.3f));
        DrawText("ELEVATION VIEW", 20, 50, 20, Fade(WHITE, 0.5f));
    }
}
