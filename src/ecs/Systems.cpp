#include "Systems.h"
#include "components.h"

float Systems::GetSimTime(entt::registry& registry)
{
    return registry.ctx().contains<float>() ? registry.ctx().get<float>() : 0.0f;
}

void Systems::RecordMissileFired(
    entt::registry& registry,
    entt::entity shooter,
    entt::entity missile,
    int targetClass,
    const std::string& weaponId,
    float rangeNM)
{
    registry.ctx().get<SimEvents>().missileEvents.push_back(MissileEvent {
        .time = GetSimTime(registry),
        .type = MissileEventType::FIRED,
        .shooter = shooter,
        .missile = missile,
        .weaponId = weaponId,
        .targetClass = targetClass,
        .rangeNM = rangeNM
    });
}

void Systems::DestroyMissile(
    entt::registry& registry,
    entt::entity missile,
    MissileEventType reason,
    float rangeNM)
{
    if (registry.all_of<DeadTag>(missile)) return;

    registry.emplace<DeadTag>(missile);

    entt::entity shooter { entt::null };
    std::string weaponId;
    if (const auto* warhead = registry.try_get<Warhead>(missile)) {
        shooter = warhead->shooter;
        weaponId = warhead->weaponId;
    }

    registry.ctx().get<SimEvents>().missileEvents.push_back(MissileEvent {
        .time = GetSimTime(registry),
        .type = reason,
        .shooter = shooter,
        .missile = missile,
        .weaponId = weaponId,
        .rangeNM = rangeNM
    });
}