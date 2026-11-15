#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "../utils/MathUtils.h"

class MapProjection {
public:
    // Coordinate conversions (all read-only)
    MathUtils::Vec2 GeoToPixel(float lat, float lon) const;
    MathUtils::Vec2 GeoToNM(float lat, float lon) const;
    MathUtils::Vec2 PixelToGeo(MathUtils::Vec2 pixel) const;
    MathUtils::Vec2 NMToGeo(MathUtils::Vec2 nm) const;              // inverse of GeoToNM, returns {lon, lat}
    MathUtils::Vec2 WorldToScreen(MathUtils::Vec2 posNM) const;     // NM -> map pixels (2D renderer only)
    float GetDistanceNM(MathUtils::Vec2 pixelA, MathUtils::Vec2 pixelB) const;

    // Reads terrain.bin. Returns false (and leaves no terrain loaded) on any problem.
    bool LoadTerrain(const std::string& path);

    float GetElevation(MathUtils::Vec2 posNM) const;    // Returns meters, 0 outside the grid
    bool HasLineOfSight(MathUtils::Vec2 startNM, MathUtils::Vec2 endNM, float startAlt, float endAlt) const;

    // For the 3D renderer (row 0 is the north edge, columns go west to east)
    int GridW() const { return gridW; }
    int GridH() const { return gridH; }
    float GridHeight(int i, int j) const { return heights[static_cast<size_t>(j) * gridW + i]; }
    MathUtils::Vec2 GridToNM(int i, int j) const;       // i = column, j = row

private:
    std::vector<float> heights;
    int gridW = 0, gridH = 0;
    float latMin = 0.0f, latMax = 0.0f, lonMin = 0.0f, lonMax = 0.0f;

    const MathUtils::Vec2 originPixel = {-528, -930};
    const MathUtils::Vec2 originGeo = {119.31394626181884, 26.072769978890854}; // X=Lon: Y=Lat
};