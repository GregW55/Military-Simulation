#pragma once
#include <cstdint>
#include <vector>

#include "../utils/MathUtils.h"

class MapProjection {
public:
    // Coordinate conversions (all read-only)
    MathUtils::Vec2 GeoToPixel(float lat, float lon) const;
    MathUtils::Vec2 GeoToNM(float lat, float lon) const;
    MathUtils::Vec2 PixelToGeo(MathUtils::Vec2 pixel) const;
    MathUtils::Vec2 WorldToScreen(MathUtils::Vec2 posNM) const;     // NM -> map pixels (renderer only)
    float GetDistanceNM(MathUtils::Vec2 pixelA, MathUtils::Vec2 pixelB) const;

    // elevation = one byte per pixel (0-255, mapped to 0-4000 m), already loaded by the caller.
    // artOffset / artScale = where the heightmap image sits on the map artwork. Pass the same
    // numbers the renderer draws it with (the scenario's heightMap offset + OFFSET_X/Y, and scale).
    void SetHeightMap(std::vector<uint8_t> elevation, int width, int height,
                    float artOffsetX, float artOffsetY, float artScale);

    float GetElevation(MathUtils::Vec2 posNM) const;    // Returns meters
    bool HasLineOfSight(MathUtils::Vec2 startNM, MathUtils::Vec2 endNM, float startAlt, float endAlt) const;

private:
    std::vector<uint8_t> elevationRaw;
    int mapWidth = 0;
    int mapHeight = 0;

    // Worked out once in SetHeightMap:
    float pxPerNmX = 0.0f, pxPerNmY = 0.0f;             // map pixels per nautical mile
    float imgScaleX = 1.0f, imgOffsetX = 0.0f;          // NM - > heightmap image: img = nm * scale + offset
    float imgScaleY = 1.0f, imgOffsetY = 0.0f;

    const MathUtils::Vec2 originPixel = {-528, -930};
    const MathUtils::Vec2 originGeo = {119.31394626181884, 26.072769978890854}; // X=Lon: Y=Lat
};