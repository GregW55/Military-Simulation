#include "GeoEngine.h"
#include "Constants.h"
#include <cmath>
#include <utility>

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

// Load raw height data into CPU memory
void MapProjection::SetHeightMap(std::vector<uint8_t> elevation, int width, int height,
                        float artOffsetX, float artOffsetY, float artScale) {
    elevationRaw = std::move(elevation);
    mapWidth = width;
    mapHeight = height;

    // NM -> map pixels is a straight line (see GeoToPixel / WorldToScreen), so the whole chain
    // NM -> map pixels -> heightmap image folds into one scale and one offset per axis.
    pxPerNmX = PX_PER_DEG_LON / (NM_PER_LAT_DEG * std::cos(originGeo.y * DEG_TO_RAD));
    pxPerNmY = PX_PER_DEG_LAT / NM_PER_LAT_DEG;

    imgScaleX = pxPerNmX / artScale;
    imgOffsetX = (originPixel.x - artOffsetX) / artScale;
    imgScaleY = pxPerNmY / artScale;
    imgOffsetY = (originPixel.y - artOffsetY) / artScale;
}

float MapProjection::GetElevation(MathUtils::Vec2 posNM) const {
    constexpr float ELEVATION_MULTIPLIER = 4000.0f / 255.0f;
    if (elevationRaw.empty()) return 0.0f;

    float imgX = posNM.x * imgScaleX + imgOffsetX;
    float imgY = posNM.y * imgScaleY + imgOffsetY;

    // Off the edge of the heightmap: treat as sea level
    if (imgX < 0 || imgX >= mapWidth || imgY < 0 || imgY >= mapHeight) return 0.0f;

   // Map 0-255 to 0-4000 meters elevation
    return elevationRaw[(int)imgY * mapWidth + (int)imgX] * ELEVATION_MULTIPLIER;
}

bool MapProjection::HasLineOfSight(MathUtils::Vec2 startNM, MathUtils::Vec2 endNM, float startAlt, float endAlt) const {
    if (startAlt > 4000.0f && endAlt > 4000.0f) {return true;}
    if (elevationRaw.empty()) return true;

    float dx = endNM.x - startNM.x;
    float dy = endNM.y - startNM.y;

    // Sample the path every 5 map pixels
    constexpr float LOS_CHECK_STEP_SIZE_PX = 5.0f;
    float dxPx = dx * pxPerNmX;
    float dyPx = dy * pxPerNmY;
    float distPx = std::sqrt(dxPx * dxPx + dyPx * dyPx);

    int steps = (int)(distPx / LOS_CHECK_STEP_SIZE_PX);
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

        if (GetElevation({currentX, currentY})> currentAlt) {
            return false;
        }
    }
    return true;
}