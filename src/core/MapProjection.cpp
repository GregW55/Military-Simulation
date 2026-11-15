#include "MapProjection.h"
#include "Constants.h"
#include <cmath>
#include <fstream>
#include <iostream>

bool MapProjection::LoadTerrain(const std::string& path) {
    heights.clear();
    gridW = gridH = 0;

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "LoadTerrain: cannot open " << path << std::endl;
        return false;
    }
    const std::streamoff fileSize = file.tellg();
    file.seekg(0);

    // Layout: int32 width, int32 height, float32 latMin, latMax, lonMin, lonMax, then width*height float32 heights
    int32_t w = 0, h = 0;
    float b[4] = {};
    file.read(reinterpret_cast<char*>(&w), 4);
    file.read(reinterpret_cast<char*>(&h), 4);
    file.read(reinterpret_cast<char*>(b), 16);
    if (!file || w < 2 || h < 2 || !(b[1] > b[0]) || !(b[3] > b[2])) {
        std::cerr << "LoadTerrain: bad header in " << path << std::endl;
        return false;
    }
    const std::streamoff expected = 24 + static_cast<std::streamoff>(w) * h * 4;
    if (fileSize != expected) {
        std::cerr << "LoadTerrain: " << path << " is " << fileSize << " bytes, a " << w << "x" << h
                  << " grid needs " << expected << std::endl;
        return false;
    }

    std::vector<float> data(static_cast<size_t>(w) * h);
    file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size() * sizeof(float)));
    if (!file) return false;

    heights = std::move(data);              // commit only after everything checked out
    gridW = w; gridH = h;
    latMin = b[0]; latMax = b[1]; lonMin = b[2]; lonMax = b[3];
    return true;
}

// Calculate x,y
MathUtils::Vec2 MapProjection::GeoToPixel(float lat, float lon) const {
    const float deltaLon = lon - originGeo.x;
    const float deltaLat = lat - originGeo.y;
    return {
        originPixel.x + (deltaLon * PX_PER_DEG_LON),
        originPixel.y + (deltaLat * PX_PER_DEG_LAT)
    };
}

MathUtils::Vec2 MapProjection::GeoToNM(float lat, float lon) const {
    // How far are from the center of the world in degrees?
    const float deltaLon = lon - originGeo.x;
    const float deltaLat = lat - originGeo.y;

    // 1 Degree of Latitude is always exactly 60 Nautical Miles
    float yNM = deltaLat * NM_PER_LAT_DEG;

    // 1 Degree of Longitude shrinks as you get closer to the poles
    float localLatRad = originGeo.y * DEG_TO_RAD;
    float xNM = deltaLon * NM_PER_LAT_DEG * std::cos(localLatRad);

    // Return the pure Cartesian coordinates
    return {xNM, yNM};
}

// Calculate Longitude/Latitude
MathUtils::Vec2 MapProjection::PixelToGeo(MathUtils::Vec2 pixel) const {
    const float deltaX = pixel.x - originPixel.x;
    const float deltaY = pixel.y - originPixel.y;
    return {
        originGeo.x + (deltaX / PX_PER_DEG_LON),
        originGeo.y + (deltaY / PX_PER_DEG_LAT)
    };
}

// Exact inverse of GeoToNM (same constants, same origin). Returns {lon, lat}.
MathUtils::Vec2 MapProjection::NMToGeo(MathUtils::Vec2 nm) const {
    const float cosLat = std::cos(originGeo.y * DEG_TO_RAD);
    return { originGeo.x + nm.x / (NM_PER_LAT_DEG * cosLat),    // lon
             originGeo.y + nm.y / NM_PER_LAT_DEG };             // lat
}

MathUtils::Vec2 MapProjection::WorldToScreen(MathUtils::Vec2 posNM) const {
    float deltaLat = posNM.y / NM_PER_LAT_DEG;
    float localLatRad = originGeo.y * DEG_TO_RAD;
    float deltaLon = posNM.x / (NM_PER_LAT_DEG * std::cos(localLatRad));

    float lat = originGeo.y + deltaLat;
    float lon = originGeo.x + deltaLon;

    return GeoToPixel(lat, lon);
}

// todo: Add true spherical distance calculation using Haversine Formula
float MapProjection::GetDistanceNM(MathUtils::Vec2 pixelA, MathUtils::Vec2 pixelB) const {
    MathUtils::Vec2 geoA = PixelToGeo(pixelA);
    MathUtils::Vec2 geoB = PixelToGeo(pixelB);

    float lonDiff = std::abs(geoA.x - geoB.x);
    float latDiff = std::abs(geoA.y - geoB.y);

    // 1 deg lat = 60 NM
    float distLatNM = latDiff * NM_PER_LAT_DEG;

    //1 deg Lon shrinks as you go North (cosine correction)

    float avgLatRad = ((geoA.y + geoB.y) / 2.0f) * DEG_TO_RAD;
    float distLonNM = lonDiff * NM_PER_LAT_DEG * std::cos(avgLatRad);

    return std::sqrt(distLatNM * distLatNM + distLonNM * distLonNM);
}

// Every grid vertex goes through GeoToNM, so terrain sits exactly where entities do.
MathUtils::Vec2 MapProjection::GridToNM(int i, int j) const {
    const float lon = lonMin + (lonMax - lonMin) * static_cast<float>(i) / static_cast<float>(gridW - 1);
    const float lat = latMax - (latMax - latMin) * static_cast<float>(j) / static_cast<float>(gridH - 1);  // row 0 = north
    return GeoToNM(lat, lon);
}

float MapProjection::GetElevation(MathUtils::Vec2 p) const {
    if (heights.empty()) return 0.0f;
    const MathUtils::Vec2 g = NMToGeo(p);                       // g.x = lon, g.y = lat
    const float fx = (g.x - lonMin) / (lonMax - lonMin) * static_cast<float>(gridW - 1);
    const float fy = (latMax - g.y) / (latMax - latMin) * static_cast<float>(gridH - 1);
    if (fx < 0.0f || fy < 0.0f || fx > static_cast<float>(gridW - 1) || fy > static_cast<float>(gridH - 1)) return 0.0f;

    const int x0 = std::min(static_cast<int>(fx), gridW - 2);
    const int y0 = std::min(static_cast<int>(fy), gridH - 2);
    const float tx = fx - static_cast<float>(x0), ty = fy - static_cast<float>(y0);
    const float top = GridHeight(x0, y0)     * (1.0f - tx) + GridHeight(x0 + 1, y0)     * tx;
    const float bot = GridHeight(x0, y0 + 1) * (1.0f - tx) + GridHeight(x0 + 1, y0 + 1) * tx;
    return top * (1.0f - ty) + bot * ty;
}

bool MapProjection::HasLineOfSight(MathUtils::Vec2 startNM, MathUtils::Vec2 endNM, float startAlt, float endAlt) const {
    if (startAlt > 4000.0f && endAlt > 4000.0f) {return true;}   // both above the tallest terrain (~3,950 m)
    if (heights.empty()) return true;                            // was elevationRaw.empty(): no terrain loaded, nothing blocks

    float dx = endNM.x - startNM.x;
    float dy = endNM.y - startNM.y;
    float distNM = std::sqrt(dx * dx + dy * dy);

    // Sample about once per terrain cell. Cell size in NM follows from the grid's latitude span.
    const float cellNM = (latMax - latMin) * NM_PER_LAT_DEG / static_cast<float>(gridH - 1);   // ~0.41 NM
    int steps = static_cast<int>(distNM / std::max(cellNM, 0.05f));
    if (steps < 2) return true;

    float invSteps = 1.0f / (float)steps;
    float stepDeltaX = dx * invSteps;
    float stepDeltaY = dy * invSteps;
    float stepDeltaAlt = (endAlt - startAlt) * invSteps;

    float currentX = startNM.x;
    float currentY = startNM.y;
    float currentAlt = startAlt;

    for (int i = 1; i < steps; i++) {
        currentX += stepDeltaX;
        currentY += stepDeltaY;
        currentAlt += stepDeltaAlt;

        if (GetElevation({currentX, currentY}) > currentAlt) {
            return false;
        }
    }
    return true;
}