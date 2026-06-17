#include "MapRenderer.h"
#include "../core/Constants.h"
#include <iostream>
// #include <cstdio> // For TraceLog

void MapRenderer::LoadAssets(const ScenarioData &scenario) {
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

void MapRenderer::UnloadAssets() {
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

void MapRenderer::DrawCropped(const Texture2D &tex, float x, float y,
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

void MapRenderer::DrawLayer(const ScenarioData &scenario) {
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
