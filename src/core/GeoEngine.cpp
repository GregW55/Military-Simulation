#include "GeoEngine.h"
#include "Constants.h"
#include "raylib.h" // ONLY for internal textures loading
#include <cmath>

// Calculate x,y
MathUtils::Vec2 MapProjection::GeoToPixel(const float lat, const float lon) {
    const float deltaLon = lon - originGeo.x;
    const float deltaLat = lat - originGeo.y;
    return {
        originPixel.x + (deltaLon * PX_PER_DEG_LON),
        originPixel.y + (deltaLat * PX_PER_DEG_LAT)
    };
}

MathUtils::Vec2 MapProjection::GeoToNM(const float lat, const float lon) {
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
MathUtils::Vec2 MapProjection::PixelToGeo(const MathUtils::Vec2 pixel) {
    const float deltaX = pixel.x - originPixel.x;
    const float deltaY = pixel.y - originPixel.y;
    return {
        originGeo.x + (deltaX / PX_PER_DEG_LON),
        originGeo.y + (deltaY / PX_PER_DEG_LAT)
    };
}

MathUtils::Vec2 MapProjection::WorldToScreen(const MathUtils::Vec2 posNM) {
    float deltaLat = posNM.y / NM_PER_LAT_DEG;
    float localLatRad = originGeo.y * DEG_TO_RAD;
    float deltaLon = posNM.x / (NM_PER_LAT_DEG * std::cos(localLatRad));

    float lat = originGeo.y + deltaLat;
    float lon = originGeo.x + deltaLon;

    return GeoToPixel(lat, lon);
}
// todo: Add true spherical distance calculation using Haversine Formula
float MapProjection::GetDistanceNM(const MathUtils::Vec2 pixelA, const MathUtils::Vec2 pixelB) {
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

// Load raw height data into CPU memory
void MapProjection::LoadheightMap(const std::string& filepath) {
    Image heightMapImage = LoadImage(filepath.c_str());

    // Store dimensions for safety checks without needing the Image object later
    mapWidth = heightMapImage.width;
    mapHeight = heightMapImage.height;

    // Load colors and cast to void* to hide the Raylib 'Color' type in the header
    heightPixelsRaw = (void*)LoadImageColors(heightMapImage);

    // Unload the Image container from CPU RAM as we now have the raw pixel array
    UnloadImage(heightMapImage);
}

float MapProjection::GetElevation(MathUtils::Vec2 pixelPos) {
    constexpr float ELEVATION_MULTIPLIER = 4000.0F / 255.0f;
    if (!heightPixelsRaw) return 0.0f;

    float imgX = (pixelPos.x - heightMapPos.x - OFFSET_X) / heightMapScale;
    float imgY = (pixelPos.y - heightMapPos.y - OFFSET_Y) / heightMapScale;

    // Boundary check using stored metadata
    if (imgX < 0 || imgX >= mapWidth || imgY < 0 || imgY >= mapHeight) return 0.0f;

    // Cast the raw pointer back to a Color pointer internally
    Color* pixels = (Color*)heightPixelsRaw;
    int index = (int)imgY * mapWidth + (int)imgX;

    if (index < 0 || index >= mapWidth * mapHeight) return 0.0f;

    Color c = pixels[index];

   // Map 0-255 Red channel to 0-4000 meters elevation
    return c.r *ELEVATION_MULTIPLIER;
}

bool MapProjection::HasLineOfSight(MathUtils::Vec2 startNM, MathUtils::Vec2 endNM, float startAlt, float endAlt) {
    if (startAlt > 4000.0f && endAlt > 4000.0f) {return true;}

    // Translate Reality into Map Space
    MathUtils::Vec2 startPx = WorldToScreen(startNM);
    MathUtils::Vec2 endPx = WorldToScreen(endNM);

    float dx = endPx.x - startPx.x;
    float dy = endPx.y - startPx.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    int steps = (int)(dist / LOS_CHECK_STEP_SIZE_PX);
    if (steps < 2) return true;

    float invSteps = 1.0f / (float)steps;
    float stepDeltaX = dx * invSteps;
    float stepDeltaY = dy * invSteps;
    float stepDeltaAlt = (endAlt - startAlt) * invSteps;

    float currentX = startPx.x;
    float currentY = startPx.y;
    float currentAlt = startAlt;

    for (int i = 1; i < steps; i++) {
        currentX += stepDeltaX;
        currentY += stepDeltaY;
        currentAlt += stepDeltaAlt;

        if (GetElevation({currentX, currentY})> currentAlt) {
            return false;
        }
    }
    return true;
}