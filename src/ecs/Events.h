#pragma once
#include <string>
#include <vector>
#include "../external/entt.hpp"

enum class MissileEventType {
    FIRED,
    OUT_OF_FUEL,
    HIT_WATER,
    SELF_DESTRUCT_NO_TARGET,
    SELF_DESTRUCT_INERTIAL_LOST,
    HIT_SHIP,
    MISSED_SHIP,
    KILLED_THREAT,
    SHOT_DOWN
};

struct MissileEvent {
    float time = 0.0f;
    MissileEventType type = MissileEventType::FIRED;
    entt::entity shooter { entt::null };
    entt::entity missile { entt::null };
    std::string weaponId;
    int targetClass = 0;
    float rangeNM = 0.0f;
};

struct SimEvents {
    std::vector<MissileEvent> missileEvents;
};