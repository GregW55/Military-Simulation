#include "../core/Constants.h"
#include "../core/MapProjection.h"
#include "../ecs/components.h"
#include "../utils/MathUtils.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include "Renderer.h"

#include "raymath.h"
#include "rlgl.h"

namespace {
    constexpr float kMetersPerNM = 1852.0f;
    constexpr float kVertExag    = 4.0f;     // display only: raises terrain AND entity altitudes together
    constexpr int   kChunks      = 16;       // must match make_terrain.py
    constexpr int   kChunkCells  = 128;

    // Render space: x = east, y = up, z = -north, unit = NM. +y is north in the sim (GeoToNM: y = dLat * 60).
    Vector3 ToRender(MathUtils::Vec2 nm, float altMeters) {
        return { nm.x, altMeters / kMetersPerNM * kVertExag, -nm.y };
    }
}

void Renderer::Init(const ScenarioData& scenario) {
    (void)scenario;
    InitWindow(1920, 1080, "AEGIS - TAIWAN THEATER");
    SetTargetFPS(targetFPS);

    // Default far plane (~1000) is too short to see an 840 NM box. If this line does not compile, see the CMake note.
    rlSetClipPlanes(1.0, 4000.0);

    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}

void Renderer::Shutdown() {
    UnloadTerrain();
    CloseWindow();
}

bool Renderer::ShouldClose() { return WindowShouldClose(); }

void Renderer::BuildTerrain(const MapProjection& map, const std::string& dir) {
    const int W = map.GridW(), H = map.GridH();
    if (W != kChunks * kChunkCells + 1 || H != W) {
        TraceLog(LOG_ERROR, "terrain grid is %dx%d but the renderer expects %d (rerun make_terrain.py?)",
                 W, H, kChunks * kChunkCells + 1);
        return;
    }

    const int vn = kChunkCells + 1;
    chunks.assign(static_cast<size_t>(kChunks) * kChunks, TerrainChunk{});

    for (int cy = 0; cy < kChunks; ++cy)
    for (int cx = 0; cx < kChunks; ++cx) {
        const int k = cy * kChunks + cx;          // same numbering as the tile files (row 0 = north)

        Mesh m = {};
        m.vertexCount   = vn * vn;
        m.triangleCount = kChunkCells * kChunkCells * 2;
        m.vertices  = static_cast<float*>(MemAlloc(m.vertexCount * 3 * sizeof(float)));
        m.texcoords = static_cast<float*>(MemAlloc(m.vertexCount * 2 * sizeof(float)));
        m.indices   = static_cast<unsigned short*>(MemAlloc(m.triangleCount * 3 * sizeof(unsigned short)));
        // No normals: lighting is baked into the pictures.

        for (int j = 0; j < vn; ++j)
        for (int i = 0; i < vn; ++i) {
            const int gi = cx * kChunkCells + i, gj = cy * kChunkCells + j;
            const Vector3 p = ToRender(map.GridToNM(gi, gj), map.GridHeight(gi, gj));
            const int v = j * vn + i;
            m.vertices[v * 3 + 0] = p.x;
            m.vertices[v * 3 + 1] = p.y;
            m.vertices[v * 3 + 2] = p.z;
            // 0..1 across THIS chunk, because each picture covers exactly one chunk
            m.texcoords[v * 2 + 0] = static_cast<float>(i) / kChunkCells;
            m.texcoords[v * 2 + 1] = static_cast<float>(j) / kChunkCells;
        }

        int t = 0;
        for (int j = 0; j < kChunkCells; ++j)
        for (int i = 0; i < kChunkCells; ++i) {
            const unsigned short tl = static_cast<unsigned short>(j * vn + i);
            const unsigned short tr = tl + 1, bl = tl + vn, br = bl + 1;
            m.indices[t++] = tl; m.indices[t++] = bl; m.indices[t++] = tr;   // counter-clockwise seen from above
            m.indices[t++] = tr; m.indices[t++] = bl; m.indices[t++] = br;
        }

        UploadMesh(&m, false);
        TerrainChunk& c = chunks[k];
        c.model = LoadModelFromMesh(m);

        const char* path = TextFormat("%s/tiles/L0_%03d.jpg", dir.c_str(), k);

        c.tex = LoadTexture(path);

        if (!IsTextureValid(c.tex))
        {
            TraceLog(LOG_ERROR, "Failed to load terrain texture: %s", path);
            continue;
        }

        GenTextureMipmaps(&c.tex);
        SetTextureFilter(c.tex, TEXTURE_FILTER_TRILINEAR);
        SetTextureWrap(c.tex, TEXTURE_WRAP_CLAMP);

        c.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = c.tex;
    }

    // Start the camera over the middle of the grid, looking at sea level
    camTarget = ToRender(map.GridToNM(W / 2, H / 2), 0.0f);
}

void Renderer::UnloadTerrain() {
    for (auto& c : chunks) {
        c.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = Texture2D{};   // the model must not own the texture
        UnloadModel(c.model);
        if (c.tex.id != 0) UnloadTexture(c.tex);
    }
    chunks.clear();
}

void Renderer::ProcessInput() {
    if (IsKeyPressed(KEY_UP)) timeScale *= 2.0f;
    if (IsKeyPressed(KEY_DOWN)) timeScale /= 2.0f;
    if (timeScale < 1.0f) timeScale = 1.0f;
    if (timeScale > 10000.0f) timeScale = 10000.0f;

    if (IsKeyPressed(KEY_R)) showRadarRings = !showRadarRings;

    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) camDist = std::clamp(camDist * (1.0f - wheel * 0.1f), 3.0f, 1500.0f);

    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {            // orbit
        const Vector2 d = GetMouseDelta();
        camYaw  -= d.x * 0.005f;
        camPitch = std::clamp(camPitch + d.y * 0.005f, 0.05f, 1.55f);
    }

    // Pan on the ground plane with WASD, scaled by zoom so it feels the same at any distance
    const float speed = camDist * 0.8f * GetFrameTime();
    const Vector3 fwd   = {-sinf(camYaw), 0.0f, -cosf(camYaw)};
    const Vector3 right = { cosf(camYaw), 0.0f, -sinf(camYaw)};
    if (IsKeyDown(KEY_W)) { camTarget.x += fwd.x * speed;   camTarget.z += fwd.z * speed; }
    if (IsKeyDown(KEY_S)) { camTarget.x -= fwd.x * speed;   camTarget.z -= fwd.z * speed; }
    if (IsKeyDown(KEY_D)) { camTarget.x += right.x * speed; camTarget.z += right.z * speed; }
    if (IsKeyDown(KEY_A)) { camTarget.x -= right.x * speed; camTarget.z -= right.z * speed; }

    camera.target = camTarget;
    camera.position = { camTarget.x + camDist * cosf(camPitch) * sinf(camYaw),
                        camTarget.y + camDist * sinf(camPitch),
                        camTarget.z + camDist * cosf(camPitch) * cosf(camYaw) };
}

void Renderer::Draw(const entt::registry& registry, const ScenarioData& scenario) {
    BeginDrawing();
    ClearBackground({ 10, 20, 30, 255 });

    const auto& map = registry.ctx().get<MapProjection>();
    const Vector2 mouse = GetMousePosition();
    const float markerR = camDist * 0.004f;                  // markers stay readable at any zoom

    struct Label { Vector2 at; std::string text; Color color; };
    std::vector<Label> labels;                               // text can't be drawn in 3D mode, so collect it

    BeginMode3D(camera);

    for (const auto& c : chunks) DrawModel(c.model, {0.0f, 0.0f, 0.0f}, 1.0f, WHITE);

    auto renderView = registry.view<Transform2D, IFF>();
    for (auto entity : renderView) {
        const auto& tf  = renderView.get<Transform2D>(entity);
        const auto& iff = renderView.get<IFF>(entity);
        const auto* kin = registry.try_get<Kinematics>(entity);
        const Color col = iff.isHostile ? RED : BLUE;

        const Vector3 pos    = ToRender(tf.pos, tf.altitude);
        const Vector3 ground = ToRender(tf.pos, map.GetElevation(tf.pos));
        const Vector2 scr    = GetWorldToScreen(pos, camera);
        const bool hovered   = Vector2Distance(scr, mouse) < 12.0f;

        DrawLine3D(pos, ground, Fade(col, 0.35f));           // altitude stalk: height above the terrain

        if (!registry.all_of<Warhead>(entity)) {
            DrawSphere(pos, markerR, col);

            if (hovered && kin) {
                std::string tip = TextFormat("Alt: %.0fm\nSpd: %.1f kts", tf.altitude, kin->currentSpeedKnots);
                if (const auto* mag = registry.try_get<Magazine>(entity)) {
                    std::vector<std::pair<std::string, int>> ammo(mag->currentAmmo.begin(), mag->currentAmmo.end());
                    std::sort(ammo.begin(), ammo.end());
                    tip += "\nAmmo:";
                    for (const auto& [weaponId, count] : ammo) tip += TextFormat("\n  %s: %d", weaponId.c_str(), count);
                }
                labels.push_back({{scr.x + 10.0f, scr.y - 20.0f}, tip, GREEN});
            }

            if (const auto* radar = registry.try_get<RadarEmitter>(entity)) {
                if (showRadarRings || hovered) {
                    auto sigTargets = registry.view<Transform2D, RadarSignature, IFF>();
                    entt::entity nearest = entt::null;
                    float nearestDistSq = std::numeric_limits<float>::max();
                    for (auto target : sigTargets) {
                        if (sigTargets.get<IFF>(target).isHostile == iff.isHostile) continue;
                        const float d = MathUtils::GetDistanceSq(tf.pos, sigTargets.get<Transform2D>(target).pos);
                        if (d < nearestDistSq) { nearestDistSq = d; nearest = target; }
                    }
                    if (registry.valid(nearest)) {
                        const auto& tgtTf  = sigTargets.get<Transform2D>(nearest);
                        const auto& tgtSig = sigTargets.get<RadarSignature>(nearest);
                        const float effectiveRangeNM = std::min(
                            radar->rangeNM * tgtSig.rcsFourthRoot,
                            MathUtils::GetRadarHorizonNM(tf.altitude) + MathUtils::GetRadarHorizonNM(tgtTf.altitude));

                        DrawCircle3D(pos, effectiveRangeNM, {1.0f, 0.0f, 0.0f}, 90.0f, Fade(GREEN, 0.6f));  // radius is already NM
                        labels.push_back({{scr.x + 10.0f, scr.y - 40.0f},
                            TextFormat("Eff. range vs nearest contact: %.1f nm", effectiveRangeNM), GREEN});
                    }
                }
            }
        } else {
            DrawSphere(pos, markerR * 0.6f, col);
            if (kin) {
                const MathUtils::Vec2 tailNM = MathUtils::Sub(tf.pos, MathUtils::Scale(kin->headingVector, 0.5f));
                DrawLine3D(pos, ToRender(tailNM, tf.altitude), ORANGE);   // flame trail
            }

            if (hovered && kin) {
                labels.push_back({{scr.x + 10.0f, scr.y - 20.0f},
                    TextFormat("MISSILE\nAlt: %.0fm\nSpd: %.1f kts", tf.altitude, kin->currentSpeedKnots), PURPLE});
                if (const auto* seeker = registry.try_get<SeekerHead>(entity)) {
                    const Vector3 tgt = ToRender(seeker->targetPos, map.GetElevation(seeker->targetPos));
                    DrawLine3D(pos, tgt, Fade(PURPLE, 0.7f));
                    DrawSphereWires(tgt, markerR * 2.0f, 6, 6, PURPLE);
                }
            }
        }
    }

    // PLACEHOLDER: reads RadarTrack.pos/altitude. The tracker rewrite will replace this with truth / measurement / estimate.
    const auto& detectionData = registry.ctx().get<RadarDetectionData>();
    for (const auto& [observerEntity, targetList] : detectionData.activeTracks) {
        if (!registry.valid(observerEntity)) continue;
        const auto& obsTf = registry.get<Transform2D>(observerEntity);
        for (const auto& trk : targetList)
            DrawLine3D(ToRender(obsTf.pos, obsTf.altitude), ToRender(trk.pos, trk.altitude), Fade(YELLOW, 0.15f));
    }

    EndMode3D();

    for (const auto& l : labels)
        DrawText(l.text.c_str(), static_cast<int>(l.at.x), static_cast<int>(l.at.y), 20, l.color);

    DrawFPS(10, 10);
    DrawText(scenario.scenarioName.c_str(), 10, 40, 20, LIGHTGRAY);
    DrawText(TextFormat("TIME COMPRESSION: %.0fx", timeScale), 10, 70, 20, YELLOW);
    if (timeScale > 1000.0f) DrawText("WARNING: PHYSICS MAY BE UNSTABLE", 10, 95, 18, RED);

    EndDrawing();
}