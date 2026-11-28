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
    constexpr int   kLevels      = 5;
    // Level loaded for EVERY chunk at startup and kept resident. Memory for all 256 chunks:
    // 0 = 90 MB, 1 = 0.4 GB, 2 = 1.4 GB, 3 = 5.7 GB. Levels above it load on demand near the camera.
    constexpr int   kBaseLevel   = 2;
    static_assert(kBaseLevel >= 0 && kBaseLevel <= 3, "level 4 exists only over Taiwan");

    // Level L is used when the camera is closer than kLevelDistNM[L] (NM to the chunk's bounding box).
    constexpr float kLevelDistNM[kLevels] = { 1e9f, 140.0f, 70.0f, 35.0f, 18.0f };


    int LevelForDistance(float d) {
        int lvl = 0;                                        // 0 means "the resident base level"
        for (int L = kBaseLevel + 1; L < kLevels; ++L)
            if (d < kLevelDistNM[L]) lvl = L;
        return lvl;
    }

    float DistToBox(Vector3 p, Vector3 lo, Vector3 hi) {
        const float dx = std::max({lo.x - p.x, 0.0f, p.x - hi.x});
        const float dy = std::max({lo.y - p.y, 0.0f, p.y - hi.y});
        const float dz = std::max({lo.z - p.z, 0.0f, p.z - hi.z});
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    Texture2D LoadChunkTexture(const char* path) {
        Texture2D texture = LoadTexture(path);
        if (texture.id == 0) return texture;
        GenTextureMipmaps(&texture);
        SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
        SetTextureFilter(texture, TEXTURE_FILTER_ANISOTROPIC_8X);
        SetTextureWrap(texture, TEXTURE_WRAP_CLAMP);
        return texture;
    }

    Image DecodeTile(std::string path) {
        return LoadImage(path.c_str());
    }

    // Main thread only(OpenGL): Turns a decoded image into a texture then frees the image
    Texture2D UploadTile(Image& img) {
        Texture2D t = LoadTextureFromImage(img);
        UnloadImage(img);
        if (t.id == 0) return t;
        GenTextureMipmaps(&t);
        SetTextureFilter(t, TEXTURE_FILTER_TRILINEAR);
        SetTextureFilter(t, TEXTURE_FILTER_ANISOTROPIC_8X);
        SetTextureWrap(t, TEXTURE_WRAP_CLAMP);
        return t;
    }

    // Render space: x = east, y = up, z = -north, unit = NM. +y is north in the sim (GeoToNM: y = dLat * 60).
    Vector3 ToRender(MathUtils::Vec2 nm, float altMeters) {
        return { nm.x, altMeters / kMetersPerNM * kVertExag, -nm.y };
    }
}

void Renderer::Init(const ScenarioData& scenario) {
    (void)scenario;
    InitWindow(1920, 1080, "AEGIS - TAIWAN THEATER");
    SetTargetFPS(targetFPS);
    LoadUnitModels();

    // Default far plane (~1000) is too short to see an 840 NM box. If this line does not compile, see the CMake note.
    rlSetClipPlanes(1.0, 4000.0);

    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}

void Renderer::Shutdown() {
    UnloadTerrain();
    UnloadUnitModels();
    CloseWindow();
}

void Renderer::LoadUnitModels() {
    shipModels["ARLEIGH_BURKE"] = LoadModel("../assets/models/ships/arleigh_burke.glb");
    shipModels["TYPE_055"] = LoadModel("../assets/models/ships/type_055.glb");
    missileModels["SM_6"] = LoadModel("../assets/models/missiles/heatseeker_air-air_missile.glb");
    missileModels["SM_2"] = LoadModel("../assets/models/missiles/heatseeker_air-air_missile.glb");
    missileModels["TOMAHAWK"] = LoadModel("../assets/models/missiles/air-to-ground_missile.glb");
    missileModels["YJ_18"] = LoadModel("../assets/models/missiles/air-to-ground_missile.glb");
}

void Renderer::UnloadUnitModels() {
    for (auto& [id, model] : shipModels)
        UnloadModel(model);

    for (auto& [id, model] : missileModels)
        UnloadModel(model);
}

bool Renderer::ShouldClose() { return WindowShouldClose(); }

void Renderer::BuildTerrain(const MapProjection& map, const std::string& dir) {
    terrainDir = dir;
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

        Vector3 lo = { FLT_MAX,  FLT_MAX,  FLT_MAX};
        Vector3 hi = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

        for (int j = 0; j < vn; ++j)
        for (int i = 0; i < vn; ++i) {
            const int gi = cx * kChunkCells + i, gj = cy * kChunkCells + j;
            const Vector3 p = ToRender(map.GridToNM(gi, gj), map.GridHeight(gi, gj));
            const int v = j * vn + i;
            m.vertices[v * 3 + 0] = p.x;
            m.vertices[v * 3 + 1] = p.y;
            m.vertices[v * 3 + 2] = p.z;
            m.texcoords[v * 2 + 0] = static_cast<float>(i) / kChunkCells;
            m.texcoords[v * 2 + 1] = static_cast<float>(j) / kChunkCells;
            lo = Vector3Min(lo, p);
            hi = Vector3Max(hi, p);
        }

        int t = 0;
        for (int j = 0; j < kChunkCells; ++j)
        for (int i = 0; i < kChunkCells; ++i) {
            const unsigned short tl = static_cast<unsigned short>(j * vn + i);
            const unsigned short tr = tl + 1, bl = tl + vn, br = bl + 1;
            m.indices[t++] = tl; m.indices[t++] = bl; m.indices[t++] = tr;
            m.indices[t++] = tr; m.indices[t++] = bl; m.indices[t++] = br;
        }

        UploadMesh(&m, false);
        TerrainChunk& c = chunks[k];
        c.model = LoadModelFromMesh(m);
        c.lo = lo;
        c.hi = hi;

        for (int L = 0; L < kLevels; ++L)
            c.avail[L] = FileExists(TextFormat("%s/tiles/L%d_%03d.jpg", dir.c_str(), L, k));

        c.base = LoadChunkTexture(TextFormat("%s/tiles/L%d_%03d.jpg", dir.c_str(), kBaseLevel, k));
        if (c.base.id == 0) TraceLog(LOG_ERROR, "missing terrain texture for chunk %d", k);
        c.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = c.base;
    }

    camTarget = ToRender(map.GridToNM(W / 2, H / 2), 0.0f);
}

void Renderer::UpdateChunkLods() {
    if (chunks.empty()) return;

    // 1) Finish at most ONE decoded image per frame. This upload is the only GPU work on the main thread.
    for (size_t i = 0; i < inFlight.size(); ++i) {
        if (inFlight[i].image.wait_for(std::chrono::seconds(0)) != std::future_status::ready) continue;

        PendingLoad p = std::move(inFlight[i]);
        inFlight.erase(inFlight.begin() + static_cast<std::ptrdiff_t>(i));

        TerrainChunk& c = chunks[p.chunk];
        c.pending = -1;
        Image img = p.image.get();
        if (img.data == nullptr) { c.avail[p.level] = false; break; }        // file failed to decode: stop asking for it

        const float d = DistToBox(camera.position, c.lo, c.hi);
        if (LevelForDistance(d) < p.level) { UnloadImage(img); break; }      // camera moved away meanwhile: drop it

        const Texture2D old = c.extra;
        c.extra = UploadTile(img);
        c.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = c.extra;   // point the model at the new one first
        c.level = p.level;
        if (old.id) UnloadTexture(old);                                      // then free the old one
        break;
    }

    // 2) Decide levels. Downgrades to the base are free. Upgrades are requested from a worker thread.
    constexpr size_t kMaxInFlight = 3;
    int reqChunk = -1, reqLevel = 0;
    float reqDist = FLT_MAX;

    for (int k = 0; k < static_cast<int>(chunks.size()); ++k) {
        TerrainChunk& c = chunks[k];
        const float d = DistToBox(camera.position, c.lo, c.hi);

        int want = LevelForDistance(d);
        // Hysteresis: keep the higher level until the camera is ~15% past its boundary
        if (want < c.level) want = std::min(c.level, std::max(want, LevelForDistance(d / 1.15f)));
        while (want > 0 && !c.avail[want]) --want;
        if (want == c.level) continue;

        if (want == 0) {                                  // back to the always-loaded base: no decoding needed
            c.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = c.base;
            c.level = 0;
            if (c.extra.id) { UnloadTexture(c.extra); c.extra = Texture2D{}; }
            continue;
        }
        if (c.pending != -1) continue;                    // one outstanding request per chunk
        if (d < reqDist) { reqChunk = k; reqLevel = want; reqDist = d; }   // nearest chunk first
    }

    if (reqChunk >= 0 && inFlight.size() < kMaxInFlight) {
        chunks[reqChunk].pending = reqLevel;
        const std::string path = TextFormat("%s/tiles/L%d_%03d.jpg", terrainDir.c_str(), reqLevel, reqChunk);
        PendingLoad p;
        p.chunk = reqChunk;
        p.level = reqLevel;
        p.image = std::async(std::launch::async, DecodeTile, path);
        inFlight.push_back(std::move(p));
    }
}

void Renderer::UnloadTerrain() {
    for (auto& p : inFlight) {                       // let running decodes finish, then free their pixels
        Image img = p.image.get();
        if (img.data) UnloadImage(img);
    }
    inFlight.clear();

    for (auto& c : chunks) {
        c.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = Texture2D{};   // the model must not own the texture
        UnloadModel(c.model);
        if (c.extra.id) UnloadTexture(c.extra);
        if (c.base.id)  UnloadTexture(c.base);
    }
    chunks.clear();
}

void Renderer::ProcessInput() {
    if (IsKeyPressed(KEY_UP)) timeScale *= 2.0f;
    if (IsKeyPressed(KEY_DOWN)) timeScale /= 2.0f;
    if (timeScale < 0.1f) timeScale = 0.1f;
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

void Renderer::Draw(const entt::registry& registry, const ScenarioData& scenario)
{
    UpdateChunkLods();
    BeginDrawing();
    ClearBackground({ 10, 20, 30, 255 });

    const auto& map = registry.ctx().get<MapProjection>();
    const Vector2 mouse = GetMousePosition();
    const float markerR = camDist * 0.004f;

    struct Label { Vector2 at; std::string text; Color color; };
    std::vector<Label> labels;

    BeginMode3D(camera);

    for (const auto& c : chunks)
        DrawModel(c.model, { 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);

    auto renderView = registry.view<Transform2D, IFF>();

    for (auto entity : renderView)
    {
        const auto& tf  = renderView.get<Transform2D>(entity);
        const auto& iff = renderView.get<IFF>(entity);
        const auto* kin = registry.try_get<Kinematics>(entity);
        const Color col = iff.isHostile ? RED : BLUE;

        const Vector3 pos    = ToRender(tf.pos, tf.altitude);
        const Vector3 ground = ToRender(tf.pos, map.GetElevation(tf.pos));
        const Vector2 scr    = GetWorldToScreen(pos, camera);
        const bool hovered   = Vector2Distance(scr, mouse) < 12.0f;

        DrawLine3D(pos, ground, Fade(col, 0.35f));

        // =========================================================
        // SHIPS / AIRCRAFT
        // =========================================================
        if (!registry.all_of<Warhead>(entity))
        {
            bool modelDrawn = false;

            if (const auto* visual = registry.try_get<VisualIdentity>(entity))
            {
                const auto modelIt = shipModels.find(visual->modelId);

                if (modelIt != shipModels.end())
                {
                    const float heading = kin
                        ? atan2f(kin->headingVector.x, -kin->headingVector.y) * RAD2DEG
                        : 0.0f;

                    const float scale = visual->modelScale * 0.01f;

                    DrawModelEx(
                        modelIt->second,
                        pos,
                        { 0.0f, 1.0f, 0.0f },
                        heading,
                        { scale, scale * kVertExag, scale },
                        WHITE
                    );

                    modelDrawn = true;
                }
            }

            // Keep the original marker as a fallback if no model is available.
            if (!modelDrawn)
                DrawSphere(pos, markerR, col);

            if (hovered && kin)
            {
                std::string tip = TextFormat(
                    "Alt: %.0fm\nSpd: %.1f kts",
                    tf.altitude,
                    kin->currentSpeedKnots
                );

                if (const auto* mag = registry.try_get<Magazine>(entity))
                {
                    std::vector<std::pair<std::string, int>> ammo(
                        mag->currentAmmo.begin(),
                        mag->currentAmmo.end()
                    );

                    std::sort(ammo.begin(), ammo.end());

                    tip += "\nAmmo:";

                    for (const auto& [weaponId, count] : ammo)
                        tip += TextFormat("\n  %s: %d", weaponId.c_str(), count);
                }

                labels.push_back({
                    { scr.x + 10.0f, scr.y - 20.0f },
                    tip,
                    GREEN
                });
            }

            if (const auto* radar = registry.try_get<RadarEmitter>(entity))
            {
                if (showRadarRings || hovered)
                {
                    auto sigTargets = registry.view<Transform2D, RadarSignature, IFF>();
                    entt::entity nearest = entt::null;
                    float nearestDistSq = std::numeric_limits<float>::max();

                    for (auto target : sigTargets)
                    {
                        if (sigTargets.get<IFF>(target).isHostile == iff.isHostile)
                            continue;

                        const float d = MathUtils::GetDistanceSq(
                            tf.pos,
                            sigTargets.get<Transform2D>(target).pos
                        );

                        if (d < nearestDistSq)
                        {
                            nearestDistSq = d;
                            nearest = target;
                        }
                    }

                    if (registry.valid(nearest))
                    {
                        const auto& tgtTf  = sigTargets.get<Transform2D>(nearest);
                        const auto& tgtSig = sigTargets.get<RadarSignature>(nearest);

                        const float effectiveRangeNM = std::min(
                            radar->maxRangeNM * tgtSig.rcsFourthRoot,
                            MathUtils::GetRadarHorizonNM(tf.altitude) +
                            MathUtils::GetRadarHorizonNM(tgtTf.altitude)
                        );

                        DrawCircle3D(
                            pos,
                            effectiveRangeNM,
                            { 1.0f, 0.0f, 0.0f },
                            90.0f,
                            Fade(GREEN, 0.6f)
                        );

                        labels.push_back({
                            { scr.x + 10.0f, scr.y - 40.0f },
                            TextFormat(
                                "Eff. range vs nearest contact: %.1f nm",
                                effectiveRangeNM
                            ),
                            GREEN
                        });
                    }
                }
            }
        }

        // =========================================================
        // MISSILES / WARHEADS
        // =========================================================
        else
        {
            bool modelDrawn = false;

            if (const auto* warhead = registry.try_get<Warhead>(entity))
            {
                const auto modelIt = missileModels.find(warhead->weaponId);

                if (modelIt != missileModels.end())
                {
                    Vector3 modelForward;
                    Vector3 direction;
                    const float horizontalSpeed = kin->currentSpeedKnots * KNOTS_TO_NM_PER_SEC;

                    if (modelIt->first == "SM_6" || modelIt->first == "SM_2") {
                        modelForward = {0.0f, -1.0f, 0.0f };

                        direction = Vector3Normalize({
                           -kin->headingVector.x * horizontalSpeed,
                           -kin->verticalVelocityNMPerSec,
                           kin->headingVector.y * horizontalSpeed
                        });
                    }
                    else {
                        modelForward = {-1.0f, 0.0f, 0.0f };

                        direction = Vector3Normalize({
                            -kin->headingVector.x * horizontalSpeed,
                            -kin->verticalVelocityNMPerSec,
                            -kin->headingVector.y * horizontalSpeed
                        });
                    }

                    const float scale = 0.075f;

                    const Vector3 rotationAxis = Vector3CrossProduct(modelForward, direction);

                    const float dot = Vector3DotProduct(modelForward, direction);
                    const float angle = acosf(Clamp(dot, -1.0f, 1.0f)) * RAD2DEG;

                    DrawModelEx(
                        modelIt->second,
                        pos,
                        Vector3Normalize(rotationAxis),
                        angle,
                        { scale, scale, scale },
                        WHITE
                    );

                    modelDrawn = true;
                }
            }

            // Keep the original marker as a fallback if no model is available.
            if (!modelDrawn)
                DrawSphere(pos, markerR * 0.6f, col);

            if (kin)
            {
                const MathUtils::Vec2 tailNM = MathUtils::Sub(
                    tf.pos,
                    MathUtils::Scale(kin->headingVector, 0.5f)
                );

                DrawLine3D(pos, ToRender(tailNM, tf.altitude), ORANGE);
            }

            if (hovered && kin)
            {
                labels.push_back({
                    { scr.x + 10.0f, scr.y - 20.0f },
                    TextFormat(
                        "MISSILE\nAlt: %.0fm\nSpd: %.1f kts",
                        tf.altitude,
                        kin->currentSpeedKnots
                    ),
                    PURPLE
                });

                if (const auto* seeker = registry.try_get<SeekerHead>(entity))
                {
                    const Vector3 tgt = ToRender(
                        seeker->targetPos,
                        seeker->targetAltitude
                    );

                    DrawLine3D(pos, tgt, Fade(PURPLE, 0.7f));
                    DrawSphereWires(tgt, markerR * 2.0f, 6, 6, PURPLE);
                }
            }
        }
    }

    EndMode3D();

    for (const auto& l : labels)
    {
        DrawText(
            l.text.c_str(),
            static_cast<int>(l.at.x),
            static_cast<int>(l.at.y),
            20,
            l.color
        );
    }

    DrawFPS(10, 10);
    DrawText(scenario.scenarioName.c_str(), 10, 40, 20, LIGHTGRAY);
    DrawText(TextFormat("TIME COMPRESSION: %.0fx", timeScale), 10, 70, 20, YELLOW);

    if (timeScale > 1000.0f)
        DrawText("WARNING: PHYSICS MAY BE UNSTABLE", 10, 95, 18, RED);

    EndDrawing();
}