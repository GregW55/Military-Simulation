#pragma once
#include <string>

#include "../utils/MathUtils.h"

class MapProjection {
public:
    MapProjection() = default;
    ~MapProjection();

    MathUtils::Vec2 GeoToPixel(float lat, float lon);
    MathUtils::Vec2 GeoToNM(float lat, float lon);
    MathUtils::Vec2 PixelToGeo(MathUtils::Vec2 pixel);
    MathUtils::Vec2 WorldToScreen(MathUtils::Vec2 posNM);
    float GetDistanceNM(MathUtils::Vec2 pixelA, MathUtils::Vec2 pixelB);

    // Height Map
    float GetElevation(MathUtils::Vec2 pixelPos); // Returns meters
    void LoadheightMap(const std::string& filepath);;

    // Raycast check
    bool HasLineOfSight(MathUtils::Vec2 startPx, MathUtils::Vec2 endPx, float startAlt, float endAlt);

private:
    void* heightPixelsRaw = nullptr;
    int mapWidth = 0;
    int mapHeight = 0;

    const MathUtils::Vec2 originPixel = {-528, -930};
    const MathUtils::Vec2 originGeo = {119.31394626181884, 26.072769978890854}; // X=Lon: Y=Lat
    const MathUtils::Vec2 heightMapPos = {-2039,-396};
    const float heightMapScale = 0.84f;
};