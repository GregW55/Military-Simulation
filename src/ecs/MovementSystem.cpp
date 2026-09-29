#include "Systems.h"

#include "Components.h"

#include "../core/Constants.h"
#include "../core/Physics.h"
#include "../utils/MetricsLogger.h"

#include <cmath>
#include <execution>

namespace
{
    void UpdateAdvancedFlightPhysics(entt::registry& registry, entt::entity entity,
            Transform2D& transform, Kinematics& kin, float deltaTime) {
        auto* aero = registry.try_get<Aerodynamics>(entity);

        if (!aero) return;

        float targetAlt = transform.altitude;

        if (auto* seeker = registry.try_get<SeekerHead>(entity)) {
            if (seeker->hasLiveTarget) {
                targetAlt = aero->cruiseAltitudeMeters < 100.0f ? 10000.0f : aero->cruiseAltitudeMeters;

                MathUtils::Vec2 toTarget =MathUtils::Sub( seeker->targetPos, transform.pos);

                float distToTargetNM = MathUtils::Length(toTarget);

                if (distToTargetNM > 0.001f) {
                    MathUtils::Vec2 toTargetDir = MathUtils::Scale(toTarget,1.0f / distToTargetNM);

                    float missileClosingSpeedNmSec = MathUtils::KnotsToNmPerSec(kin.currentSpeedKnots) *
                        MathUtils::Dot(kin.headingVector,toTargetDir);

                    MathUtils::Vec2 targetToMissileDir = MathUtils::Scale(toTarget, -1.0f / distToTargetNM);

                    float targetClosingSpeedNmSec =MathUtils::Dot(seeker->targetVel, targetToMissileDir);

                    float combinedClosingSpeedNmSec = missileClosingSpeedNmSec + targetClosingSpeedNmSec;

                    if (combinedClosingSpeedNmSec > 0.0001f) {
                        float timeToImpactSec = distToTargetNM / combinedClosingSpeedNmSec;

                        float altDiffMeters = std::abs(seeker->targetAltitude - transform.altitude);

                        float maxLateralAccelMps2 = seeker->maxLateralGs * Physics::GRAVITY;

                        float timeToDescendSec = 0.0f;

                        if (maxLateralAccelMps2 > 0.0f) {
                            float peakVerticalSpeedMps = std::sqrt(2.0f * maxLateralAccelMps2 * altDiffMeters);

                            peakVerticalSpeedMps = std::min(peakVerticalSpeedMps, kin.currentSpeedKnots *MPS_PER_KNOT);

                            timeToDescendSec = peakVerticalSpeedMps / maxLateralAccelMps2;
                        }

                        if (timeToImpactSec <= (timeToDescendSec + DESCENT_SAFETY_MARGIN_SEC)) {
                            targetAlt = seeker->targetAltitude;
                        }
                    }
                }
            }
        }

        float altDiff = targetAlt - transform.altitude;

        float speedMps = kin.currentSpeedKnots * MPS_PER_KNOT;

        float maxLateralAccelMps2 = 0.0f;

        if (auto* seeker = registry.try_get<SeekerHead>(entity)) {
            maxLateralAccelMps2 = seeker->maxLateralGs * Physics::GRAVITY;
        }

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

        bool isStalled = (aero->currentFuelKg <= 0.0f && kin.currentSpeedKnots < 200.0f);

        if (transform.altitude < 0.0f || isStalled) {
            if (!kin.isDead) {
                if (auto* warhead = registry.try_get<Warhead>(entity)) {
                    std::string reason = isStalled ? "OUT_OF_FUEL" : "HIT_WATER";

                    MetricsLogger::Log(Systems::GetSimTime(registry),"CRASH", (uint32_t)warhead->shooter, 0,
                        warhead->weaponId, 0.0f, reason);
                }
                kin.isDead = true;
            }
        }
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
            float currentSpeedNmps = MathUtils::KnotsToNmPerSec(kin.currentSpeedKnots);

            if (MathUtils::LengthSq(kin.headingVector) < 0.0001f) {
                kin.headingVector = {1.0f, 0.0f};
            }

            kin.velocity = MathUtils::Scale(kin.headingVector, currentSpeedNmps);
        }
        else {
            kin.velocity = {0.0f, 0.0f};
        }

        transform.pos = MathUtils::Add(transform.pos, MathUtils::Scale(kin.velocity, deltaTime));
    }


    void UpdateMovementForEntity(entt::registry& registry, entt::entity entity, float deltaTime) {
        auto& transform = registry.get<Transform2D>(entity);

        auto& kin = registry.get<Kinematics>(entity);

        if (registry.try_get<Aerodynamics>(entity)) UpdateAdvancedFlightPhysics(registry, entity, transform,kin, deltaTime);
        else UpdateStandardHydrodynamics(kin, deltaTime);

        UpdateVelocityAndPosition(transform,kin,deltaTime);
    }


    void MarkDeadEntities(entt::registry& registry, const auto& view) {
        for (auto entity : view) {
            if (registry.get<Kinematics>(entity).isDead) {
                registry.emplace_or_replace<DeadTag>(entity);
            }
        }
    }
}


void Systems::MovementSystem(entt::registry& registry, float deltaTime) {
    auto view = registry.view<Transform2D, Kinematics>();

    std::for_each(std::execution::par_unseq, view.begin(), view.end(), [&registry, deltaTime](auto entity) {
            UpdateMovementForEntity(registry, entity, deltaTime);
        }
    );

    MarkDeadEntities(registry, view);
}