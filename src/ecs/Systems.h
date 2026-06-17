#pragma once
#include "Components.h"
#include "../core/Constants.h"
#include "../core/Physics.h"
#include <iostream>
#include <execution>

class Systems {
public:
    // --- NAVIGATION SYSTEM (The Captain) ---
    static void NavigationSystem(entt::registry& registry, float deltaTime) {
        auto view = registry.view<Transform2D, Kinematics>();

        for (auto entity : view) {
            auto& trans = view.get<Transform2D>(entity);
            auto& kin = view.get<Kinematics>(entity);

            // ========================
            // === MISSILE GUIDANCE ===
            // ========================
            if (auto* seeker = registry.try_get<SeekerHead>(entity)) {

                if (seeker->phase == GuidancePhase::MIDCOURSE_DATALINK) {
                    if (registry.valid(seeker->datalinkSource)) {
                        auto& shipTracks = registry.ctx().get<RadarDetectionData>().activeTracks[seeker->datalinkSource];

                        float closestDist = 999999.0f;
                        for (const auto& track : shipTracks) {
                            float d = MathUtils::GetDistance(seeker->targetPos, track.pos);
                            if (d < DATALINK_ENGAGEMENT_CORRELATION_RADIUS_NM) {
                                if (d < closestDist) {
                                    closestDist = d;
                                    seeker->targetPos = track.pos;
                                    seeker->targetVel = track.vel;
                                }
                            }
                        }
                    } else {
                        seeker->phase = GuidancePhase::INERTIAL_BLIND;
                    }

                    MathUtils::Vec2 interceptPos = MathUtils::PredictIntercept(
                        trans.pos, kin.currentSpeedKnots, seeker->targetPos, seeker->targetVel
                    );

                    MathUtils::Vec2 toIntercept = MathUtils::Sub(interceptPos, trans.pos);
                    float distToIntercept = MathUtils::Length(toIntercept);

                    if (distToIntercept > 0.001f) {
                        kin.headingVector = MathUtils::Scale(toIntercept, 1.0f / distToIntercept);
                    }

                    if (seeker->type == SeekerType::ACTIVE_RADAR && distToIntercept < seeker->rangeNM) {
                        seeker->phase = GuidancePhase::TERMINAL_PITBULL;
                        registry.emplace_or_replace<RadarEmitter>(entity,
                            seeker->rangeNM, seeker->rangeNM * seeker->rangeNM, 0.25f, 0.0f
                        );
                    }
                }
                else if (seeker->phase == GuidancePhase::TERMINAL_PITBULL) {
                    auto& myTracks = registry.ctx().get<RadarDetectionData>().activeTracks[entity];

                    if (!myTracks.empty()) {
                        float closestDist = 999999.0f;
                        for (const auto& track : myTracks) {
                            float d = MathUtils::GetDistance(trans.pos, track.pos);
                            if (d < closestDist) {
                                closestDist = d;
                                seeker->targetPos = track.pos;
                                seeker->targetVel = track.vel;
                            }
                        }

                        float turnRateDeg = MathUtils::CalculateProNavTurnRate(
                            trans.pos, kin.velocity, seeker->targetPos, seeker->targetVel, 4.0f
                        );

                        float currentAngle = std::atan2(kin.headingVector.y, kin.headingVector.x) * RAD_TO_DEG;
                        currentAngle += (turnRateDeg * deltaTime);

                        kin.headingVector.x = std::cos(currentAngle * DEG_TO_RAD);
                        kin.headingVector.y = std::sin(currentAngle * DEG_TO_RAD);
                    }
                }
                else if (seeker->phase == GuidancePhase::INERTIAL_BLIND) {
                    MathUtils::Vec2 toIntercept = MathUtils::Sub(seeker->targetPos, trans.pos);
                    float dist = MathUtils::Length(toIntercept);
                    if (dist > 0.001f) {
                        kin.headingVector = MathUtils::Scale(toIntercept, 1.0f / dist);
                    }
                }
                continue;
            }

            // =====================
            // === SHIP GUIDANCE ===
            // =====================
            if (auto* brain = registry.try_get<AutonomousGuidance>(entity)) {
                auto* myIFF = registry.try_get<IFF>(entity);
                if (!myIFF) continue;

                // Only re-scan for targets every 2 seconds, not every frame
                brain->targetingCooldown -= deltaTime;

                if (brain->targetingCooldown <= 0.0f) {
                    brain->targetingCooldown = 2.0f;  // Recalculate every 2 seconds
                    brain->hasTarget = false;

                    float closestDistSq = std::numeric_limits<float>::max();
                    auto allShips = registry.view<Transform2D, IFF, Hull>();

                    for (auto other : allShips) {
                        if (entity == other) continue;
                        auto& otherIFF = registry.get<IFF>(other);
                        if (myIFF->isHostile != otherIFF.isHostile) {
                            auto& otherTrans = registry.get<Transform2D>(other);
                            float distSq = MathUtils::LengthSq(MathUtils::Sub(trans.pos, otherTrans.pos));
                            if (distSq < closestDistSq) {
                                closestDistSq = distSq;
                                brain->cachedTargetPos = otherTrans.pos;
                                brain->hasTarget = true;
                            }
                        }
                    }
                }

                // Use the cached target instead of re-scanning
                float closestDist = MathUtils::GetDistance(trans.pos, brain->cachedTargetPos);
                MathUtils::Vec2 toTarget = MathUtils::Sub(brain->cachedTargetPos, trans.pos);
                bool foundTarget = brain->hasTarget;

                // --- Combat Interrupt ---
                auto* myRadar = registry.try_get<RadarEmitter>(entity);
                float detectionRange = myRadar ? myRadar->rangeNM : 50.0f;

                if (foundTarget && closestDist < detectionRange) {
                    if (brain->currentState == TacticalState::PATROL || brain->currentState == TacticalState::TRANSIT) {
                        brain->currentState = TacticalState::INTERCEPT;
                    }
                }

                // --- Tactical State Machine ---
                switch (brain->currentState) {
                    case TacticalState::DEFEND: {
                        if (foundTarget) {
                            if (closestDist < brain->desiredStandoffNM) {
                                if (closestDist > MATH_EPSILON) {
                                    MathUtils::Vec2 awayDirection = MathUtils::Scale(toTarget, -1.0f / closestDist);
                                    kin.headingVector = awayDirection;
                                }
                                kin.desiredSpeedKnots = kin.maxSpeedKnots;
                            } else {
                                kin.desiredSpeedKnots = 0.0f;
                            }
                        } else {
                            brain->currentState = TacticalState::PATROL;
                        }
                        break;
                    }
                    case TacticalState::INTERCEPT: {
                        if (foundTarget) {
                            if (closestDist > brain->desiredStandoffNM + 2.0f) {
                                if (closestDist > MATH_EPSILON) {
                                    kin.headingVector = MathUtils::Scale(toTarget, 1.0f / closestDist);
                                }
                                kin.desiredSpeedKnots = kin.maxSpeedKnots;
                            } else {
                                brain->currentState = TacticalState::STANDOFF;
                                kin.desiredSpeedKnots = 0.0f;
                            }
                        } else {
                            brain->currentState = TacticalState::PATROL;
                        }
                        break;
                    }
                    case TacticalState::STANDOFF: {
                        if (foundTarget) {
                            if (closestDist > brain->desiredStandoffNM + 5.0f) {
                                kin.headingVector = MathUtils::Scale(toTarget, 1.0f / closestDist);
                                kin.desiredSpeedKnots = kin.maxSpeedKnots;
                            } else if (closestDist < brain->desiredStandoffNM - 5.0f) {
                                kin.headingVector = MathUtils::Scale(toTarget, -1.0f / closestDist);
                                kin.desiredSpeedKnots = kin.maxSpeedKnots;
                            } else {
                                kin.desiredSpeedKnots = 0.0f; // Perfect firing spot
                            }
                        } else {
                            brain->currentState = TacticalState::PATROL;
                        }
                        break;
                    }
                    case TacticalState::PATROL: {
                        if (!brain->waypoints.empty()) {
                            MathUtils::Vec2 targetWp = brain->waypoints[brain->currentWaypointIndex];
                            float distToWp = MathUtils::GetDistance(trans.pos, targetWp);

                            if (distToWp < 0.5f) {
                                brain->currentWaypointIndex = (brain->currentWaypointIndex + 1) % brain->waypoints.size();
                                targetWp = brain->waypoints[brain->currentWaypointIndex];
                                distToWp = MathUtils::GetDistance(trans.pos, targetWp);
                            }

                            if (distToWp > MATH_EPSILON) {
                                kin.headingVector = MathUtils::Scale(MathUtils::Sub(targetWp, trans.pos), 1.0f / distToWp);
                            }
                            kin.desiredSpeedKnots = kin.maxSpeedKnots;
                        } else {
                            kin.desiredSpeedKnots = 0.0f;
                        }
                        break;
                    }
                    case TacticalState::TRANSIT: {
                        if (!brain->waypoints.empty()) {
                            MathUtils::Vec2 destination = brain->waypoints.back(); // Head for final waypoint
                            float distToDestination = MathUtils::GetDistance(trans.pos, destination);

                            if (distToDestination > 0.5f) {
                                // Still traveling - point toward destination and go full speed
                                kin.headingVector = MathUtils::Scale(
                                    MathUtils::Sub(destination, trans.pos),
                                    1.0f / distToDestination
                                );
                                kin.desiredSpeedKnots = kin.maxSpeedKnots;
                            } else {
                                // Arrived - hand off to patrol behavior
                                brain->currentState = TacticalState::PATROL;
                                brain->currentWaypointIndex = 0;
                            }
                        } else {
                            // No waypoints assigned, just stop and patrol in place
                            brain->currentState = TacticalState::PATROL;
                        }
                        break;
                    }
                    default:
                        kin.desiredSpeedKnots = 0.0f;
                        break;
                }
            }
        }
    }

    // --- MOVEMENT SYSTEM (The Engine Room) ---
    static void MovementSystem(entt::registry& registry, float deltaTime) {
        auto view = registry.view<Transform2D, Kinematics>();

        // --- MULTITHREADED PHYSICS LOOP ---
        std::for_each(std::execution::par_unseq, view.begin(), view.end(), [&registry, deltaTime](auto entity) {
            auto& transform = registry.get<Transform2D>(entity);
            auto& kin = registry.get<Kinematics>(entity);

            // ===============================
            // === ADVANCED FLIGHT PHYSICS ===
            // ===============================
            if (auto* aero = registry.try_get<Aerodynamics>(entity)) {
                // If JSON failed to load cruise altitude, force it to 10,000m
                float targetAlt = aero->cruiseAltitudeMeters < 100.0f ? 10000.0f : aero->cruiseAltitudeMeters;

                if (auto* seeker = registry.try_get<SeekerHead>(entity)) {
                    float distToTarget = MathUtils::GetDistance(transform.pos, seeker->targetPos);
                    if (distToTarget < 5.0f) targetAlt = 10.0f;
                }

                float verticalSpeed = 500.0f * deltaTime;
                if (transform.altitude < targetAlt) {
                    transform.altitude += verticalSpeed;
                    if (transform.altitude > targetAlt) transform.altitude = targetAlt;
                } else if (transform.altitude > targetAlt) {
                    transform.altitude -= verticalSpeed;
                    if (transform.altitude < targetAlt) transform.altitude = targetAlt;
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

                float speedMps = kin.currentSpeedKnots * MPS_PER_KNOT;
                float machNumber = speedMps / aero ->cachedSpeedOfSound;

                float dragCoeff = DRAG_COEFF_SUBSONIC;
                if (machNumber > 0.8f && machNumber < 1.2f) dragCoeff = DRAG_COEFF_TRANSONIC;
                else if (machNumber >= 1.2f) dragCoeff = DRAG_COEFF_SUPERSONIC;

                float dragForce = 0.5f * aero->cachedAirDensity * (speedMps * speedMps) * dragCoeff * aero->areaM2;

                if (kin.currentSpeedKnots < kin.desiredSpeedKnots) {
                    float accelMps = (aero->currentFuelKg > 0.0f) ? (aero->thrustNewtons * aero->inverseMass) : 0.0f;
                    kin.currentSpeedKnots += (accelMps * KNOTS_PER_MPS) * deltaTime;
                }

                float decelMps = dragForce * aero->inverseMass;
                kin.currentSpeedKnots -= (decelMps * KNOTS_PER_MPS) * deltaTime;

                bool isStalled = (aero->currentFuelKg <= 0.0f && kin.currentSpeedKnots < 200.0f);
                if (transform.altitude < 0.0f || isStalled) {
                    kin.isDead = true;
                }
            }
            // ==============================
            // === STANDARD HYDRODYNAMICS ===
            // ==============================
            else {
                if (kin.currentSpeedKnots < kin.desiredSpeedKnots) {
                    kin.currentSpeedKnots += kin.accelerationRate * deltaTime;
                    if (kin.currentSpeedKnots > kin.desiredSpeedKnots) kin.currentSpeedKnots = kin.desiredSpeedKnots;
                } else if (kin.currentSpeedKnots > kin.desiredSpeedKnots) {
                    kin.currentSpeedKnots -= kin.accelerationRate * deltaTime;
                    if (kin.currentSpeedKnots < kin.desiredSpeedKnots) kin.currentSpeedKnots = kin.desiredSpeedKnots;
                }
            }

            if (kin.currentSpeedKnots < 0.0f) kin.currentSpeedKnots = 0.0f;

            if (kin.currentSpeedKnots > 0.01f) {
                float currentSpeedNmps = MathUtils::KnotsToNmPerSec(kin.currentSpeedKnots);
                if (MathUtils::LengthSq(kin.headingVector) < 0.0001f) kin.headingVector = {1.0f, 0.0f};
                kin.velocity = MathUtils::Scale(kin.headingVector, currentSpeedNmps);
            } else {
                kin.velocity = {0.0f, 0.0f};
            }

            transform.pos = MathUtils::Add(transform.pos, MathUtils::Scale(kin.velocity, deltaTime));
        });

        for (auto entity : view) {
            if (registry.get<Kinematics>(entity).isDead) {
                registry.emplace_or_replace<DeadTag>(entity);
            }
        }
    }

    // --- RADAR ---
    static void RadarSystem(entt::registry& registry, float deltaTime) {
        auto& map = registry.ctx().get<MapProjection>();
        auto& detectionData = registry.ctx().get<RadarDetectionData>();

        auto observers = registry.view<Transform2D, RadarEmitter, IFF>();
        auto targets = registry.view<Transform2D, RadarSignature, IFF>();

        for (auto& [obsId, trackList] : detectionData.activeTracks) {
            for (auto& track : trackList) {
                track.ageSec += deltaTime;
            }
            std::erase_if(trackList, [](const RadarTrack& t) {
                return t.ageSec > TRACK_STALE_TIMEOUT_SEC;
            });
        }

        for (auto observer : observers) {
            auto& obsTransform = observers.get<Transform2D>(observer);
            auto& obsRadar = observers.get<RadarEmitter>(observer);
            auto& obsIFF = observers.get<IFF>(observer);
            uint32_t obsId = static_cast<uint32_t> (observer);

            obsRadar.timeSinceLastScan += deltaTime;

            if (obsRadar.timeSinceLastScan >= obsRadar.scanRateSec) {
                float timeDelta = obsRadar.timeSinceLastScan;
                obsRadar.timeSinceLastScan = 0.0f;
                std::vector<MathUtils::Vec2> newPings;

                for (auto target : targets) {
                    if (observer == target) continue;

                    auto& tgtTransform = targets.get<Transform2D>(target);
                    auto& tgtSig = targets.get<RadarSignature>(target);
                    auto& tgtIFF = targets.get<IFF>(target);

                    if (obsIFF.isHostile == tgtIFF.isHostile) continue;

                    MathUtils::Vec2 diff = MathUtils::Sub(obsTransform.pos, tgtTransform.pos);
                    float distSq = MathUtils::LengthSq(diff);

                    if (distSq > obsRadar.rangeNmSq) continue;

                    bool detected = Physics::CheckRadarDetection(map,
                        obsTransform.pos, obsTransform.altitude, obsRadar.rangeNM,
                        tgtTransform.pos, tgtTransform.altitude, tgtSig.rcsFourthRoot, std::sqrt(distSq));

                    if (detected) newPings.push_back(tgtTransform.pos);
                }

                auto& myTracks = detectionData.activeTracks[observer];
                std::vector<RadarTrack> newlyDiscoveredTracks;

                for (const auto& pingPos : newPings) {
                    bool matched = false;
                    float closestDistSq = 4.0f;
                    int bestMatchIndex = -1;

                    for (size_t i = 0; i < myTracks.size(); ++i) {
                        float dSq = MathUtils::LengthSq(MathUtils::Sub(pingPos, myTracks[i].pos));
                        if (dSq < closestDistSq) {
                            closestDistSq = dSq;
                            bestMatchIndex = i;
                    }
                }

                if (bestMatchIndex != -1) {
                    MathUtils::Vec2 calculatedVel = MathUtils::Scale(
                        MathUtils::Sub(pingPos, myTracks[bestMatchIndex].pos),
                        1.0f / timeDelta);
                    myTracks[bestMatchIndex].pos = pingPos;
                     myTracks[bestMatchIndex].vel = calculatedVel;
                      myTracks[bestMatchIndex].ageSec = 0.0f;
                      matched = true;
                }

                if (!matched) newlyDiscoveredTracks.push_back({pingPos, {0.0f, 0.0f}, 0.0f});
            }
            myTracks.insert(myTracks.end(), newlyDiscoveredTracks.begin(), newlyDiscoveredTracks.end());
        }
    }
}

    // --- COMBAT SYSTEM ---
    static void CombatSystem(entt::registry& registry, float deltaTime) {
        auto view = registry.view<Transform2D, Kinematics, IFF, AutonomousGuidance, Magazine>();

        for (auto entity: view) {
            auto& transform = view.get<Transform2D>(entity);
            auto& kin = view.get<Kinematics>(entity);
            auto& iff = view.get<IFF>(entity);
            auto& brain = view.get<AutonomousGuidance>(entity);
            auto& mag = view.get<Magazine>(entity);

            if (mag.fireCooldown > 0.0f) mag.fireCooldown -= deltaTime;

            if (brain.currentState == TacticalState::STANDOFF ||
                brain.currentState == TacticalState::DEFEND ||
                brain.currentState == TacticalState::INTERCEPT){

                if (mag.fireCooldown <= 0.0f) {
                    auto& detectionData = registry.ctx().get<RadarDetectionData>();
                    uint32_t myId = static_cast<uint32_t>(entity);

                    if (detectionData.activeTracks.find(entity) == detectionData.activeTracks.end() ||
                        detectionData.activeTracks[entity].empty()) {
                        continue;
                    }

                    auto& tracks = detectionData.activeTracks[entity];
                    const RadarTrack* bestTarget = nullptr;
                    float closestDistSq = std::numeric_limits<float>::max();

                    for (const auto& track : tracks) {
                        float dSq = MathUtils::LengthSq(MathUtils::Sub(transform.pos, track.pos));
                        if (dSq < closestDistSq) {
                            closestDistSq = dSq;
                            bestTarget = &track;
                        }
                    }

                    if (!bestTarget) continue;

                    MathUtils::Vec2 initialTargetPos = bestTarget->pos;
                    MathUtils::Vec2 initialTargetVel = bestTarget->vel;
                    float distToTarget = std::sqrt(closestDistSq);

                    for (auto& [weaponId, ammoCount] : mag.currentAmmo) {
                        if (ammoCount > 0) {
                            const MissileStats& mStats = TacticalDatabase::GetMissile(weaponId);

                            float mRange = mStats.maxRangeNM > 1.0f ? mStats.maxRangeNM : 150.0f;
                            if (distToTarget > mRange) continue; // Out of range

                            ammoCount -= 1;
                            mag.fireCooldown = 3.0f;

                            auto missile = registry.create();
                            float missileSpeed = mStats.maxSpeedKnots > 10.0f ? mStats.maxSpeedKnots : 2000.0f;

                            registry.emplace<SeekerHead>(missile,
                                entity,                                         // Datalink Source
                                mStats.seekerType,                              // Seeker Type
                                mStats.seekerRangeNM,                           // Seeker Range
                                GuidancePhase::MIDCOURSE_DATALINK,              // Phase
                                initialTargetPos,                               // Where it was spotted
                                initialTargetVel                                // How fast it was moving
                            );

                            registry.emplace<Transform2D>(missile,
                                transform.pos,                               // Location (x, y)
                                transform.altitude,                          // Altitude
                                transform.heading);                          // Heading Vector

                            registry.emplace<Kinematics>(missile,
                                kin.velocity,                                // Inherit ship's momentum as it leaves the tube
                                kin.headingVector,                           // Launch facing the same direction as the ship
                                missileSpeed,                           // Max Speed (Knots)
                                600.0f,                                 // Apply Immediate 600 Knot Booster Kick
                                missileSpeed,                           // Desired speed in knots (Push the throttle to 100%)
                                // TODO: Change from hardcoded 50 acceleration rate to using realistic missile rates
                                50.0f);                                         // Massive acceleration rate (knots per second)

                            registry.emplace<Aerodynamics>(missile,
                                mStats.massKg,                                  // Mass in Kg
                                1.0f / mStats.massKg,                           // Inverse Mass
                                mStats.areaM2,                                  // Area in Meters Squared
                                mStats.thrustNewtons,                           // Thrust in Newtons
                                mStats.maxFuelKg,                               // Maximum fuel in Kg
                                mStats.burnRateKgSec,                           // How fast the fuel is burned per second
                                mStats.cruiseAltitudeMeters                     // Cruise Altitude in Meters
                                );

                            registry.emplace<Warhead>(missile,
                                mStats.warheadYield,                            // Warheads Yield (WIP units)
                                mStats.lethalRadiusNM,                          // Lethal blast radius in Nautical Miles
                                mStats.lethalRadiusNM * mStats.lethalRadiusNM   // Lethal blast radius squared
                                );

                            registry.emplace<RadarSignature>(missile,
                                mStats.rcs,                                     // Radar Cross-Section
                                mStats.rcsFourthRoot                            // RCS Fourth Root (One time pow call)
                                );

                            registry.emplace<IFF>(missile,iff.isHostile);
                            break; // Break to make sure we only fire ONE missile this frame
                        }
                    }
                }
            }
        }
    }

    static void ProximityFuseSystem(entt::registry& registry) {
        auto missiles = registry.view<Transform2D, Warhead, IFF>();
        auto targets = registry.view<Transform2D, Hull, IFF>();

        for (auto missile : missiles) {
            auto& mTrans = missiles.get<Transform2D>(missile);
            auto& warhead = missiles.get<Warhead>(missile);
            auto& mIFF = missiles.get<IFF>(missile);

            for (auto target: targets) {
                auto& tTrans = targets.get<Transform2D>(target);
                auto& tIFF = targets.get<IFF>(target);

                if (mIFF.isHostile == tIFF.isHostile) continue;

                // Fast-Fail Distance Check
                MathUtils::Vec2 diff = MathUtils::Sub(mTrans.pos, tTrans.pos);
                float distNmSq = MathUtils::LengthSq(diff);

                // DETONATION
                if (distNmSq <= warhead.lethalRadiusNmSq) {
                    auto& hull = targets.get<Hull>(target);
                    hull.currentHP -= warhead.yieldDamage;

                    if (hull.currentHP <= 0.0f) {
                        registry.emplace_or_replace<DeadTag>(target);
                    }

                    registry.emplace_or_replace<DeadTag>(missile);
                    break;
                }
            }
        }
    }
};
