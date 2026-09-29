#include "Systems.h"

float Systems::GetSimTime(entt::registry& registry)
{
    return registry.ctx().contains<float>()
        ? registry.ctx().get<float>()
        : 0.0f;
}