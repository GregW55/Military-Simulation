#include "../Systems.h"

#include "../Components.h"

#include "../../core/Constants.h"
#include "../../core/Physics.h"
#include "../../utils/MetricsLogger.h"

#include <cmath>
#include <execution>

namespace
{
    void UpdateAdvancedFlightPhysics(entt::registry& registry, entt::entity entity,
            Transform2D& transform, Kinematics& kin, float deltaTime) {
        auto* aero = registry.try_get<Aerodynamics>(entity);

        if (!aero) return;

        float targetAlt = aero->desiredAltitudeMeters;
        float altDiff = targetAlt - transform.altitude;

        float speedMps = kin.currentSpeedKnots * MPS_PER_KNOT;

        float maxLateralAccelMps2 = aero->maxManeuverGs * GRAVITY_MPS2;

        float maxVerticalSpeedMps = (maxLateralAccelMps2 > 0.0f) ? std::sqrt(2.0f * maxLateralAccelMps2 *
            std::abs(altDiff)): 0.0f;

        maxVerticalSpeedMps = std::min(maxVerticalSpeedMps, speedMps);

        float verticalSpeed = maxVerticalSpeedMps * deltaTime;

        if (transform.altitude < targetAlt) {
            transform.altitude = std::min(transform.altitude + verticalSpeed, targetAlt);
        }
        else if (transform.altitude > targetAlt) {
            transform.altitude = std::max(transform.altitude - verticalSpeed, targetAlt);
        }

        if (aero->currentFuelKg > 0.0f) {
            aero->currentFuelKg -= aero->burnRateKgSec * deltaTime;

            if (aero->currentFuelKg < 0.0f) aero->currentFuelKg = 0.0f;

            aero->inverseMass = 1.0f / (aero->dryMassKg + aero->currentFuelKg);
        }

        if (std::abs(transform.altitude - aero->lastCachedAltitude) > 50.0f) {
            aero->cachedAirDensity = Physics::GetAirDensity(transform.altitude);
            aero->cachedSpeedOfSound = Physics::GetSpeedOfSound(transform.altitude);
            aero->lastCachedAltitude = transform.altitude;
        }

        float machNumber = speedMps / aero->cachedSpeedOfSound;

        float dragCoeff = DRAG_COEFF_SUBSONIC;

        if (machNumber > 0.8f && machNumber < 1.2f) {
            dragCoeff = DRAG_COEFF_TRANSONIC;
        }
        else if (machNumber >= 1.2f) {
            dragCoeff = DRAG_COEFF_SUPERSONIC;
        }

        float dragForce = 0.5f * aero->cachedAirDensity * (speedMps * speedMps) * dragCoeff * aero->areaM2;

        if (kin.currentSpeedKnots < kin.desiredSpeedKnots) {
            float accelMps = (aero->currentFuelKg > 0.0f) ? (aero->thrustNewtons * aero->inverseMass) : 0.0f;

            kin.currentSpeedKnots += (accelMps * KNOTS_PER_MPS) * deltaTime;
        }

        float decelMps = dragForce * aero->inverseMass;

        kin.currentSpeedKnots -= (decelMps * KNOTS_PER_MPS) * deltaTime;
    }


    void UpdateStandardHydrodynamics(Kinematics& kin, float deltaTime)
    {
        if (kin.currentSpeedKnots < kin.desiredSpeedKnots) {
            kin.currentSpeedKnots += kin.accelerationRate * deltaTime;

            if (kin.currentSpeedKnots > kin.desiredSpeedKnots) kin.currentSpeedKnots = kin.desiredSpeedKnots;
        }
        else if (kin.currentSpeedKnots > kin.desiredSpeedKnots) {
            kin.currentSpeedKnots -= kin.accelerationRate * deltaTime;

            if (kin.currentSpeedKnots < kin.desiredSpeedKnots) kin.currentSpeedKnots = kin.desiredSpeedKnots;
        }
    }


    void UpdateVelocityAndPosition(Transform2D& transform, Kinematics& kin, float deltaTime) {
        if (kin.currentSpeedKnots < 0.0f) kin.currentSpeedKnots = 0.0f;

        if (kin.currentSpeedKnots > 0.01f) {
            if (MathUtils::LengthSq(kin.headingVector) < 0.0001f) {
                kin.headingVector = {1.0f, 0.0f};
            }

            kin.velocity = MathUtils::GetVelocityNmPerSec(kin.headingVector, kin.currentSpeedKnots);
        }
        else {
            kin.velocity = {0.0f, 0.0f};
        }

        transform.pos = MathUtils::ExtrapolatePosition(transform.pos, kin.velocity, deltaTime);
    }


    void UpdateMovementForEntity(entt::registry& registry, entt::entity entity, float deltaTime) {
        auto& transform = registry.get<Transform2D>(entity);

        auto& kin = registry.get<Kinematics>(entity);

        if (registry.try_get<Aerodynamics>(entity)) UpdateAdvancedFlightPhysics(registry, entity, transform,kin, deltaTime);
        else UpdateStandardHydrodynamics(kin, deltaTime);

        UpdateVelocityAndPosition(transform,kin,deltaTime);
    }


    void CheckMissileCrashes(entt::registry& registry) {
        auto view = registry.view<Transform2D, Kinematics, Aerodynamics, Warhead>();
        for (auto entity : view) {
            auto& transform = view.get<Transform2D>(entity);
            auto& kin = view.get<Kinematics>(entity);
            auto& aero = view.get<Aerodynamics>(entity);

            // Todo: Calculate if the missile can reach its target based on its altitude/speed/current drag etc instead of hard coded 200.0f speed gate
            bool isStalled = (aero.currentFuelKg <= 0.0f && kin.currentSpeedKnots < 50.0f);
            if (isStalled) Systems::DestroyMissile(registry, entity, MissileEventType::OUT_OF_FUEL);
            else if (transform.altitude < 0.0f) Systems::DestroyMissile(registry, entity, MissileEventType::HIT_WATER);
        }
    }
}


void Systems::MovementSystem(entt::registry& registry, float deltaTime) {
    auto view = registry.view<Transform2D, Kinematics>();

    std::for_each(std::execution::par_unseq, view.begin(), view.end(), [&registry, deltaTime](auto entity) {
            UpdateMovementForEntity(registry, entity, deltaTime);
        }
    );

    CheckMissileCrashes(registry);
}