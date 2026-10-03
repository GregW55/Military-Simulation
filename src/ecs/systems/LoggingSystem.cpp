#include "../Components.h"
#include "../Events.h"
#include "../Systems.h"
#include "../../utils/MetricsLogger.h"

namespace {
    struct CsvLabels {
        const char* event;
        const char* outcome;
        int targetClass;
    };

    CsvLabels LabelsFor(const MissileEvent& e) {
        switch (e.type) {
            case MissileEventType::FIRED:
                return {"FIRE", "IN_FLIGHT", e.targetClass};
            case MissileEventType::OUT_OF_FUEL:
                return {"CRASH", "OUT_OF_FUEL", 0};
            case MissileEventType::HIT_WATER:
                return {"CRASH", "HIT_WATER", 0};
            case MissileEventType::SELF_DESTRUCT_NO_TARGET:
                return {"CRASH", "SELF_DESTRUCT_NO_TARGET", 0};
            case MissileEventType::SELF_DESTRUCT_INERTIAL_LOST:
                return {"CRASH", "SELF_DESTRUCT_INERTIAL_LOST", 0};
            case MissileEventType::HIT_SHIP:
                return {"IMPACT", "HIT", 0};
            case MissileEventType::MISSED_SHIP:
                return {"IMPACT", "MISS", 0};
            case MissileEventType::KILLED_THREAT:
                return {"INTERCEPT", "KILLED_THREAT", static_cast<int>(ThreatClass::MISSILE_INBOUND)};
            case MissileEventType::SHOT_DOWN:
                return {"SHOT_DOWN", "INTERCEPTED", 0};
        }
        return {"UNKNOWN", "UNKNOWN", 0};
    }
}

void Systems::LoggingSystem(entt::registry& registry)
{
    auto& events = registry.ctx().get<SimEvents>().missileEvents;

    for (const auto& e : events) {
        const CsvLabels labels = LabelsFor(e);
        MetricsLogger::Log(e.time, labels.event, static_cast<uint32_t>(e.shooter),
            labels.targetClass, e.weaponId, e.rangeNM, labels.outcome);
    }

    events.clear();
}