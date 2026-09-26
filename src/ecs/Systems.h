#pragma once
#include "Components.h"
#include "../core/Constants.h"
#include "../core/Physics.h"
#include "../utils/ThreatAnalysis.h"
#include "../utils/MetricsLogger.h"
#include <iostream>
#include <execution>
#include <random>

constexpr bool VERBOSE_COMBAT_LOG = false;
constexpr float MAX_MISSILE_TURN_RATE_DEG_SEC = 25.0f; // todo: calculate based on actual missile

class Systems {
public:
    // --- NAVIGATION SYSTEM  ---
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
                    seeker->timeSinceLaunchSec += deltaTime;
                    bool missileIsHostile = registry.get<IFF>(entity).isHostile;

                    // Network-wide datalink: check EVERY friendly ship's radar picture
                    RadarTrack* best = nullptr;
                    float closestDist = DATALINK_ENGAGEMENT_CORRELATION_RADIUS_NM;

                    auto& detectionData = registry.ctx().get<RadarDetectionData>();
                    auto radarShips = registry.view<RadarEmitter, IFF>();
                    for (auto observer : radarShips) {
                        if (radarShips.get<IFF>(observer).isHostile != missileIsHostile) continue;

                        for (auto& track : detectionData.activeTracks[observer]) {
                            float d = MathUtils::GetDistance(seeker->targetPos, track.pos);
                            if (d < closestDist) {
                                closestDist = d;
                                best = &track;
                            }
                        }
                    }
                    if (best) {
                        seeker->targetPos = best->pos;
                        seeker->targetVel = best->vel;
                        seeker->targetAltitude = best->altitude;
                        seeker->timeSinceLastCorrelation = 0.0f;
                    } else {
                        seeker->timeSinceLastCorrelation += deltaTime;
                        if (seeker->timeSinceLastCorrelation > MIDCOURSE_LOST_TIMEOUT_SEC) {
                            seeker->phase = GuidancePhase::INERTIAL_BLIND;
                        }
                    }

                    MathUtils::Vec2 interceptPos = MathUtils::PredictIntercept(
                        trans.pos, kin.currentSpeedKnots, seeker->targetPos, seeker->targetVel
                    );

                    MathUtils::Vec2 toIntercept = MathUtils::Sub(interceptPos, trans.pos);
                    float distToIntercept = MathUtils::Length(toIntercept);

                    if (distToIntercept > 0.001f) {
                        MathUtils::Vec2 desiredHeading = MathUtils::Scale(toIntercept, 1.0f / distToIntercept);

                        float currentAngle = std::atan2(kin.headingVector.y, kin.headingVector.x) * RAD_TO_DEG;
                        float desiredAngle = std::atan2(desiredHeading.y, desiredHeading.x) * RAD_TO_DEG;
                        float angleDiff = MathUtils::GetShortestAngleDiff(currentAngle, desiredAngle);

                        float maxStep = MAX_MISSILE_TURN_RATE_DEG_SEC * deltaTime;
                        float clampedStep = std::clamp(angleDiff, -maxStep, maxStep);

                        float newAngle = currentAngle + clampedStep;
                        kin.headingVector.x = std::cos(newAngle * DEG_TO_RAD);
                        kin.headingVector.y = std::sin(newAngle * DEG_TO_RAD);
                    }

                    if (seeker->type == SeekerType::ACTIVE_RADAR && distToIntercept < seeker->rangeNM &&
                        seeker->timeSinceLaunchSec > MIN_TIME_BEFORE_SEEKER_ACTIVATION_SEC) {
                        seeker->phase = GuidancePhase::TERMINAL_PITBULL;
                        registry.emplace_or_replace<RadarEmitter>(entity,
                            seeker->rangeNM, seeker->rangeNM * seeker->rangeNM, 0.25f, 0.0f
                        );
                        if (VERBOSE_COMBAT_LOG) {
                            std::cout << "[SEEKER] Missile " << (uint32_t)entity << " going active, dist to intercept: "
                                      << distToIntercept << "nm\n";
                        }
                    }
                }
                else if (seeker->phase == GuidancePhase::TERMINAL_PITBULL) {
                    auto& myTracks = registry.ctx().get<RadarDetectionData>().activeTracks[entity];

                    bool foundCorrelatedTrack = false;
                    if (!myTracks.empty()) {
                        float closestDist = 999999.0f;
                        RadarTrack* best = nullptr;
                        for (auto& track : myTracks) {
                            // FOV check : is this track within the seeker's forward-facing cone?
                            MathUtils::Vec2 toTrack = MathUtils::Sub(track.pos, trans.pos);
                            float distToTrack = MathUtils::Length(toTrack);
                            if (distToTrack < 0.001f) continue;

                            MathUtils::Vec2 toTrackDir = MathUtils::Scale(toTrack, 1.0f / distToTrack);
                            float dotWithHeading = MathUtils::Dot(kin.headingVector, toTrackDir);
                            float angleFromHeadingDeg = std::acos(std::clamp(dotWithHeading, -1.0f, 1.0f)) * RAD_TO_DEG;

                            if (angleFromHeadingDeg > SEEKER_FOV_DEGREES) continue; // Outside seeker's cone, can't see it

                            float distFromAssignedTarget = MathUtils::GetDistance(seeker->targetPos, track.pos);
                            if (distFromAssignedTarget > SEEKER_IDENTITY_GATE_NM) continue;

                            if (distToTrack < closestDist) {
                                closestDist = distToTrack;
                                best = &track;
                            }
                        }

                        constexpr float TRACK_FRESHNESS_THRESHOLD_SEC = 1.0f; // must have refreshed recently to count as a real lock
                        if (best && closestDist < seeker->rangeNM && best->ageSec < TRACK_FRESHNESS_THRESHOLD_SEC) {
                            seeker->targetPos = best->pos;
                            seeker->targetVel = best->vel;
                            seeker->targetAltitude = best->altitude;
                            foundCorrelatedTrack = true;
                            seeker->timeSinceLastCorrelation = 0.0f;
                        }
                    }

                    if (foundCorrelatedTrack) {
                        float turnRateDeg = MathUtils::CalculateProNavTurnRate(
                            trans.pos, kin.velocity, seeker->targetPos, seeker->targetVel, 4.0f
                        );
                        turnRateDeg = std::clamp(turnRateDeg, -MAX_MISSILE_TURN_RATE_DEG_SEC, MAX_MISSILE_TURN_RATE_DEG_SEC);

                        float currentAngle = std::atan2(kin.headingVector.y, kin.headingVector.x) * RAD_TO_DEG;
                        currentAngle += (turnRateDeg * deltaTime);
                        kin.headingVector.x = std::cos(currentAngle * DEG_TO_RAD);
                        kin.headingVector.y = std::sin(currentAngle * DEG_TO_RAD);
                    } else {
                        seeker->timeSinceLastCorrelation += deltaTime;

                        constexpr float SEARCH_GRACE_PERIOD_SEC = 4.0f; // Todo: Make this a global variable
                        if (seeker->timeSinceLastCorrelation > SEARCH_GRACE_PERIOD_SEC) {
                            // Grace period expired - try network retarget before giving up
                            bool missileIsHostile = registry.get<IFF>(entity).isHostile;
                            RadarTrack* bestNetworkTarget = nullptr;
                            float bestScore = -1.0f;

                            auto& networkDetectionData = registry.ctx().get<RadarDetectionData>();
                            auto radarShips = registry.view<RadarEmitter, IFF>();
                            for (auto observer : radarShips) {
                                if (radarShips.get<IFF>(observer).isHostile != missileIsHostile) continue;
                                for (auto& track : networkDetectionData.activeTracks[observer]) {
                                    constexpr float TRACK_FRESHNESS_THRESHOLD_SEC = 1.0f;
                                    if (track.ageSec > TRACK_FRESHNESS_THRESHOLD_SEC) continue;
                                    if (track.consistentObservationSec < 1.0f) continue;
                                    if (track.threatScore > bestScore) {
                                        bestScore = track.threatScore;
                                        bestNetworkTarget = &track;
                                    }
                                }
                            }

                            if (bestNetworkTarget) {
                                seeker->targetPos = bestNetworkTarget->pos;
                                seeker->targetVel = bestNetworkTarget->vel;
                                seeker->targetAltitude = bestNetworkTarget->altitude;
                                seeker->timeSinceLastCorrelation = 0.0f;
                            } else {
                                kin.isDead = true;
                                if (auto* warhead = registry.try_get<Warhead>(entity)) {
                                    float simTime = registry.ctx().contains<float>() ? registry.ctx().get<float>() : 0.0f;
                                    MetricsLogger::Log(simTime, "CRASH", (uint32_t)warhead->shooter, 0, warhead->weaponId, 0.0f, "SELF_DESTRUCT_NO_TARGET");
                                }
                            }
                        }
                    }
                }
                else if (seeker->phase == GuidancePhase::INERTIAL_BLIND) {
                    seeker->timeSinceLastCorrelation += deltaTime;

                    float distToTargetNM = MathUtils::GetDistance(trans.pos, seeker->targetPos);

                    if (distToTargetNM > 0.001f) {
                        MathUtils::Vec2 desiredHeading = MathUtils::Scale(
                            MathUtils::Sub(seeker->targetPos, trans.pos), 1.0f / distToTargetNM);

                        float currentAngle = std::atan2(kin.headingVector.y, kin.headingVector.x) * RAD_TO_DEG;
                        float desiredAngle = std::atan2(desiredHeading.y, desiredHeading.x) * RAD_TO_DEG;
                        float angleDiff = MathUtils::GetShortestAngleDiff(currentAngle, desiredAngle);

                        float maxStep = MAX_MISSILE_TURN_RATE_DEG_SEC * deltaTime;
                        float clampedStep = std::clamp(angleDiff, -maxStep, maxStep);

                        float newAngle = currentAngle + clampedStep;
                        kin.headingVector.x = std::cos(newAngle * DEG_TO_RAD);
                        kin.headingVector.y = std::sin(newAngle * DEG_TO_RAD);
                    }

                    constexpr float INERTIAL_ARRIVAL_NM = 0.5f;
                    constexpr float INERTIAL_MAX_FLIGHT_SEC = 20.0f; // hard cap once fully blind
                    if (distToTargetNM < INERTIAL_ARRIVAL_NM || seeker->timeSinceLastCorrelation > INERTIAL_MAX_FLIGHT_SEC) {
                        kin.isDead = true;
                        if (auto* warhead = registry.try_get<Warhead>(entity)) {
                            float simTime = registry.ctx().contains<float>() ? registry.ctx().get<float>() : 0.0f;
                            MetricsLogger::Log(simTime, "CRASH", (uint32_t)warhead->shooter, 0, warhead->weaponId, 0.0f, "SELF_DESTRUCT_INERTIAL_LOST");
                        }
                    }
                }
            }

            // =====================
            // === SHIP GUIDANCE ===
            // =====================
            if (auto* brain = registry.try_get<AutonomousGuidance>(entity)) {
                auto* myIFF = registry.try_get<IFF>(entity);
                if (!myIFF) continue;

                auto* myRadar = registry.try_get<RadarEmitter>(entity);
                if (!myRadar) continue;

                brain->targetingCooldown -= deltaTime;

                if (brain->targetingCooldown <= 0.0f) {
                    brain->targetingCooldown = 2.0f;
                    brain->hasTarget = false;

                    auto& myTracks = registry.ctx().get<RadarDetectionData>().activeTracks[entity];
                    float closestDistSq = std::numeric_limits<float>::max();

                    for (auto& track : myTracks) {
                        float distSq = MathUtils::LengthSq(MathUtils::Sub(trans.pos, track.pos));
                        if (distSq < closestDistSq) {
                            closestDistSq = distSq;
                            brain->cachedTargetPos = track.pos;
                            brain->hasTarget = true;
                        }
                    }
                }

                float closestDist = MathUtils::GetDistance(trans.pos, brain->cachedTargetPos);
                MathUtils::Vec2 toTarget = MathUtils::Sub(brain->cachedTargetPos, trans.pos);
                bool foundTarget = brain->hasTarget;
                float detectionRange = myRadar->rangeNM;

                if (foundTarget && closestDist < detectionRange) {
                    if (brain->currentState == TacticalState::PATROL || brain->currentState == TacticalState::TRANSIT) {
                        if (!brain->pendingEngagement) {
                            brain->pendingEngagement = true;
                            std::uniform_real_distribution<float> delayDist(2.0f, 8.0f);
                            brain->reactionTimer = delayDist(registry.ctx().get<std::mt19937>());
                        }

                        brain->reactionTimer -= deltaTime;
                        if (brain->reactionTimer <= 0.0f) {
                            brain->currentState = TacticalState::INTERCEPT;
                            brain->pendingEngagement = false;
                            if (VERBOSE_COMBAT_LOG) {
                                std::cout << "[STATE] Entity " << (uint32_t)entity << " PATROL -> INTERCEPT (range: "
                                          << closestDist << "nm)\n";
                           }
                        }
                    }
                }

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
                            if (closestDist > brain->desiredStandoffNM + 1.0f) {
                                if (closestDist > MATH_EPSILON) {
                                    kin.headingVector = MathUtils::Scale(toTarget, 1.0f / closestDist);
                                }
                                kin.desiredSpeedKnots = kin.maxSpeedKnots;
                            } else {
                                brain->currentState = TacticalState::STANDOFF;
                                kin.desiredSpeedKnots = 0.0f;
                                if (VERBOSE_COMBAT_LOG) {
                                    std::cout << "[STATE] Entity " << (uint32_t)entity << " INTERCEPT -> STANDOFF (range: "
                                              << closestDist << "nm, standoff target: " << brain->desiredStandoffNM << "nm)\n";
                                }
                            }
                        } else {
                            brain->currentState = TacticalState::PATROL;
                        }
                        break;
                    }
                    case TacticalState::STANDOFF: {
                        if (foundTarget) {
                            if (closestDist > brain->desiredStandoffNM + 1.0f) {
                                kin.headingVector = MathUtils::Scale(toTarget, 1.0f / closestDist);
                                kin.desiredSpeedKnots = kin.maxSpeedKnots;
                            } else if (closestDist < brain->desiredStandoffNM - 1.0f) {
                                kin.headingVector = MathUtils::Scale(toTarget, -1.0f / closestDist);
                                kin.desiredSpeedKnots = kin.maxSpeedKnots;
                            } else {
                                kin.desiredSpeedKnots = 0.0f;
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

    // --- MOVEMENT SYSTEM ---
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
                float targetAlt = aero->cruiseAltitudeMeters < 100.0f ? 10000.0f : aero->cruiseAltitudeMeters;

                if (auto* seeker = registry.try_get<SeekerHead>(entity)) {
                    float distToTarget = MathUtils::GetDistance(transform.pos, seeker->targetPos);
                    if (distToTarget < 5.0f) {
                        if (seeker->isInterceptor) {
                            targetAlt = seeker->targetAltitude;
                        } else {
                            targetAlt = 10.0f;
                        }
                    }
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
                    if (!kin.isDead) {
                        if (auto* warhead = registry.try_get<Warhead>(entity)) {
                            float simTime = registry.ctx().contains<float>() ? registry.ctx().get<float>() : 0.0f;
                            std::string reason = isStalled ? "OUT_OF_FUEL" : "HIT_WATER";
                            MetricsLogger::Log(simTime, "CRASH", (uint32_t)warhead->shooter, 0, warhead->weaponId, 0.0f, reason);
                        }
                        kin.isDead = true;
                    }
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

        for (auto it = detectionData.activeTracks.begin(); it != detectionData.activeTracks.end(); ) {
            if (!registry.valid(it->first)) {
                it = detectionData.activeTracks.erase(it);
            } else {
                for (auto& track : it->second) track.ageSec += deltaTime;
                std::erase_if(it->second, [](const RadarTrack& t) { return t.ageSec > TRACK_STALE_TIMEOUT_SEC; });
                ++it;
            }
        }

        for (auto observer : observers) {
            auto& obsTransform = observers.get<Transform2D>(observer);
            auto& obsRadar = observers.get<RadarEmitter>(observer);
            auto& obsIFF = observers.get<IFF>(observer);

            obsRadar.timeSinceLastScan += deltaTime;

            if (obsRadar.timeSinceLastScan >= obsRadar.scanRateSec) {
                float timeDelta = obsRadar.timeSinceLastScan;
                obsRadar.timeSinceLastScan = 0.0f;
                std::vector<RadarTrack> newPings;

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

                    if (detected) {
                        RadarTrack ping;
                        ping.pos = tgtTransform.pos;
                        ping.altitude = tgtTransform.altitude;
                        newPings.push_back(ping);
                    }
                }

                auto& myTracks = detectionData.activeTracks[observer];
                std::vector<RadarTrack> newlyDiscoveredTracks;

                for (const auto& ping : newPings) {
                    bool matched = false;
                    float closestDistSq = TRACK_CORRELATION_GATE_NM_SQ;
                    int bestMatchIndex = -1;

                    for (size_t i = 0; i < myTracks.size(); ++i) {
                        MathUtils::Vec2 predictedPos = MathUtils::Add(myTracks[i].pos, MathUtils::Scale(myTracks[i].vel, timeDelta));

                        float dSq = MathUtils::LengthSq(MathUtils::Sub(ping.pos, predictedPos));
                        if (dSq < closestDistSq) {
                            closestDistSq = dSq;
                            bestMatchIndex = i;
                        }
                    }

                    constexpr float MAX_PLAUSIBLE_SPEED_KNOTS = 4000.0f; // Todo: pull from the missile json
                    if (bestMatchIndex != -1) {
                        MathUtils::Vec2 calculatedVel = MathUtils::Scale(
                            MathUtils::Sub(ping.pos, myTracks[bestMatchIndex].pos),
                            1.0f / timeDelta);

                        float derivedSpeedKnots = MathUtils::Length(calculatedVel) * 3600.0f;
                        if (derivedSpeedKnots > MAX_PLAUSIBLE_SPEED_KNOTS) {
                            // Treat as a bad/noisy observation — hold last known velocity instead of trusting the spike
                            calculatedVel = myTracks[bestMatchIndex].vel;
                            derivedSpeedKnots = myTracks[bestMatchIndex].speedKnots;
                        }
                        // Todo: Maybe make something so if an object continues to go this fast we pick it up as a enemy

                        myTracks[bestMatchIndex].pos = ping.pos;
                        myTracks[bestMatchIndex].vel = calculatedVel;
                        myTracks[bestMatchIndex].altitude = ping.altitude;
                        myTracks[bestMatchIndex].ageSec = 0.0f;
                        myTracks[bestMatchIndex].speedKnots = derivedSpeedKnots;

                        MathUtils::Vec2 toObserver = MathUtils::Sub(obsTransform.pos, ping.pos);
                        float dist = MathUtils::Length(toObserver);
                        if (dist > 0.001f) {
                            MathUtils::Vec2 dir = MathUtils::Scale(toObserver, 1.0f / dist);
                            myTracks[bestMatchIndex].closingSpeedKnots = MathUtils::Dot(calculatedVel, dir) * 3600.0f;
                        }

                        myTracks[bestMatchIndex].timeToImpactSec = ThreatAnalysis::EstimateTimeToImpact(
                            obsTransform.pos, ping.pos, calculatedVel);

                        ThreatClass newClassification = ThreatAnalysis::Classify(myTracks[bestMatchIndex].speedKnots, ping.altitude);
                        if (newClassification == myTracks[bestMatchIndex].classification) {
                            myTracks[bestMatchIndex].consistentObservationSec += deltaTime;
                        } else {
                            myTracks[bestMatchIndex].consistentObservationSec = 0.0f;
                        }
                        myTracks[bestMatchIndex].classification = newClassification;
                        myTracks[bestMatchIndex].threatScore = ThreatAnalysis::ComputeThreatScore(myTracks[bestMatchIndex]);
                        matched = true;
                    }

                    if (!matched) {
                        RadarTrack newTrack;
                        newTrack.pos = ping.pos;
                        newTrack.vel = {0.0f, 0.0f};
                        newTrack.ageSec = 0.0f;
                        newTrack.altitude = ping.altitude;
                        newlyDiscoveredTracks.push_back(newTrack);
                    }
                }
                myTracks.insert(myTracks.end(), newlyDiscoveredTracks.begin(), newlyDiscoveredTracks.end());
            }
        }
    }

    // --- COMBAT SYSTEM ---
    static void CombatSystem(entt::registry& registry, float deltaTime) {
        registry.ctx().get<SharedThreatPicture>().Clear();
        auto& detectionData = registry.ctx().get<RadarDetectionData>();

        auto shipView = registry.view<EngagementLog>();
        for (auto entity : shipView) {
            auto& log = shipView.get<EngagementLog>(entity);
            for (auto& eng : log.current) {
                if (eng.missileAlive) {
                    eng.missileAlive = registry.valid(eng.missileEntity);
                }
            }
            std::erase_if(log.current, [](const ActiveEngagement& e) { return !e.missileAlive; });
        }

        // --- Engagement decisions ---
        auto combatView = registry.view<Transform2D, IFF, Magazine, AutonomousGuidance, EngagementLog>();
        std::vector<RadarTrack*> sortedThreats;
        sortedThreats.reserve(16);

        for (auto entity: combatView) {
            auto& transform = combatView.get<Transform2D>(entity);
            auto& iff = combatView.get<IFF>(entity);
            auto& mag = combatView.get<Magazine>(entity);
            auto& brain = combatView.get<AutonomousGuidance>(entity);
            auto& log = combatView.get<EngagementLog>(entity);

            if (brain.currentState != TacticalState::STANDOFF &&
                brain.currentState != TacticalState::DEFEND   &&
                brain.currentState != TacticalState::INTERCEPT) continue;

            if (mag.fireCooldown > 0.0f){
                mag.fireCooldown -= deltaTime;
                continue;
            }

            auto& tracks = detectionData.activeTracks[entity];
            if (tracks.empty()) continue;

            // --- Sort tracks by threat score (highest priority first) ---
            sortedThreats.clear();
            for (auto& track : tracks) sortedThreats.push_back(&track);
            std::sort(sortedThreats.begin(), sortedThreats.end(),
                [](const RadarTrack* a, const RadarTrack* b) {
                    return a->threatScore > b->threatScore;
                });

            // --- Evaluate each threat and decide ---
            for (RadarTrack* threat : sortedThreats) {
                // Also add a distance check / threat check (cannot wait 2 seconds if a missile will hit us in 1)
                if (threat->consistentObservationSec < 2.0f) continue; // Hard coded confirmation for now

                // DOCTRINE RULE 2: Select the right weapon for this target
                std::string selectedWeapon = SelectWeapon(mag, *threat);
                if (selectedWeapon.empty()) continue;

                const MissileStats& mStats = TacticalDatabase::GetMissile(selectedWeapon);
                float distToTarget = MathUtils::GetDistance(transform.pos, threat->pos);

                // DOCTRINE RULE 3: Target must be in weapon employment zone
                if (distToTarget > mStats.maxRangeNM) continue;

                // Don't fire at close range - too late for missile to arm/guide
                if (distToTarget < MIN_ENGAGEMENT_RANGE_NM) continue;

                // DOCTRINE RULE 4: Shoot-Look-Shoot: Are we ALREADY engaging this threat with a missile in flight?
                constexpr int MAX_SHOTS_PER_THREAT = 2;

                int activeShots = CountActiveShotsAgainst(log, *threat);
                if (activeShots >= MAX_SHOTS_PER_THREAT) {
                    continue; // Already saturated this target, move on to next threat
                }

                if (activeShots == 1 && threat->timeToImpactSec > 30.0f) {
                    continue; // First shot is still in flight with time to spare - let it work
                }

                // DOCTRINE RULE 5: Ammo conservation
                // Never fire last missile (keep at least 1 in reserve per type)
                if (mag.currentAmmo.at(selectedWeapon) <= 1 &&
                    threat->classification != ThreatClass::MISSILE_INBOUND){
                    continue; // Save last missile for incoming missiles
                }

                // DOCTRINE RULE 6: Determine salvo size
                int salvoCount = DetermineSalvoSize(*threat);

                // DOCTRINE RULE 7: Don't engage targets another ship is already handling
                // (Unless it's an incoming missile - then everyone who can shoot, does)
                auto& sharedPicture = registry.ctx().get<SharedThreatPicture>();
                if  (threat->classification != ThreatClass::MISSILE_INBOUND) {
                    if (sharedPicture.IsAssigned(threat->pos)) continue;
                }

                // Claim the target before firing
                sharedPicture.Assign(threat->pos, entity);

                // --- FIRE ---
                for (int shot = 0; shot < salvoCount; ++shot) {
                    if (mag.currentAmmo[selectedWeapon] <= 0) break;

                    auto missileEntity = LaunchMissile(registry, entity, transform,
                        registry.get<Kinematics>(entity), iff, *threat, mStats, selectedWeapon);
                    if (VERBOSE_COMBAT_LOG) {
                        std::cout << "[FIRE] Entity " << (uint32_t)entity << " launched " << selectedWeapon
                                  << " at threat (class=" << (int)threat->classification
                                  << ", dist=" << distToTarget << "nm, TTI=" << threat->timeToImpactSec << "s)\n";
                    }

                    log.current.push_back({
                        threat->pos,
                        threat->vel,
                        missileEntity,
                        threat->classification,
                        0.0f,    // timeFiredSec -> put a global sim time here
                        true
                    });
                    log.totalMissilesFired++;
                    mag.currentAmmo[selectedWeapon]--;
                }

                mag.fireCooldown = (threat->classification == ThreatClass::MISSILE_INBOUND)
                    ? 0.5f    // Fast response for incoming missiles
                    : 5.0f;   // Normal cooldown for surface targets

                break;
            }
        }
    }

    static void ProximityFuseSystem(entt::registry& registry) {
        auto missiles = registry.view<Transform2D, Warhead, IFF>();
        auto shipTargets = registry.view<Transform2D, Hull, IFF, Kinematics, RadarSignature>();

        for (auto missile : missiles) {
            if (registry.all_of<DeadTag>(missile)) continue;

            auto& mTrans = missiles.get<Transform2D>(missile);
            auto& warhead = missiles.get<Warhead>(missile);
            auto& mIFF = missiles.get<IFF>(missile);

            for (auto target: shipTargets) {
                auto& tTrans = shipTargets.get<Transform2D>(target);
                auto& tIFF = shipTargets.get<IFF>(target);
                auto& tKin = shipTargets.get<Kinematics>(target);
                auto& tSig = shipTargets.get<RadarSignature>(target);

                if (mIFF.isHostile == tIFF.isHostile) continue;

                // Fast-Fail Distance Check
                MathUtils::Vec2 diff = MathUtils::Sub(mTrans.pos, tTrans.pos);
                float distNmSq = MathUtils::LengthSq(diff);

                float altDiffMeters = std::abs(mTrans.altitude - tTrans.altitude);

                // DETONATION SEQUENCE
                if (distNmSq <= warhead.lethalRadiusNmSq && altDiffMeters <= FUSE_ALTITUDE_TOLERANCE_METERS) {

                    // --- DYNAMIC PK CALCULATION ---
                    float dynamicPk = warhead.baseReliability;

                    // Todo: More realistic calculation instead of broad assumptions
                    // Kinematic Evasion: Is the target moving fast?
                    if (tKin.currentSpeedKnots > 1400.0f) {
                        dynamicPk *= 0.25f;
                    }
                    else if (tKin.currentSpeedKnots > 750.0f) {
                        dynamicPk *= 0.4f; // Targets moving this fast significantly harder to hit
                    }
                    else if (tKin.currentSpeedKnots > 350.0f) {
                        dynamicPk *= 0.75f; // Fast targets are 25% harder to hit
                    } else if (tKin.currentSpeedKnots > 35.0f) {
                        dynamicPk *= 0.95f; // Fast ships are slightly harder to hit
                    }

                    // Signature Evasion: Is the target stealthy/small?
                    if (tSig.rcs < 1.0f) {
                        dynamicPk *= 0.7f; // Small RCS degrades seeker lock
                    }

                    // --- ROLL THE DICE ---
                    float simTime = registry.ctx().contains<float>() ? registry.ctx().get<float>() : 0.0f;

                    // Use a fallback RNG if the global one isn't injected yet, to prevent crashes
                    std::mt19937 fallbackRng(std::random_device{}());
                    auto& rng = registry.ctx().contains<std::mt19937>() ? registry.ctx().get<std::mt19937>() : fallbackRng;

                    std::uniform_real_distribution<float> distRoll(0.0f, 1.0f);
                    float roll = distRoll(rng);

                    float distNM = std::sqrt(distNmSq);

                    // --- DETERMINE OUTCOME ---
                    if (roll <= dynamicPk) {
                        // HIT
                        auto& hull = shipTargets.get<Hull>(target);
                        hull.currentHP -= warhead.yieldDamage;

                        MetricsLogger::Log(simTime, "IMPACT", (uint32_t)warhead.shooter, 0, warhead.weaponId, distNM, "HIT");

                        if (VERBOSE_COMBAT_LOG) std::cout << "[IMPACT] Missile HIT entity " << (uint32_t)target <<"\n";

                        if (hull.currentHP <= 0.0f) {
                            registry.emplace_or_replace<DeadTag>(target);
                        }
                    } else {
                        // MISS
                        MetricsLogger::Log(simTime, "IMPACT", (uint32_t)warhead.shooter, 0, warhead.weaponId, distNM, "MISS");

                        if (VERBOSE_COMBAT_LOG) std::cout << "[IMPACT] Missile MISSED entity " << (uint32_t)target << " (Roll: " << roll << ", Required: " << dynamicPk << ")\n";
                    }

                    registry.emplace_or_replace<DeadTag>(missile);
                    break;
                }
            }

            if (registry.all_of<DeadTag>(missile)) continue;

            for (auto otherMissile : missiles) {
                if (otherMissile == missile) continue;
                if (registry.all_of<DeadTag>(otherMissile)) continue;

                auto& oIFF = missiles.get<IFF>(otherMissile);
                if (mIFF.isHostile == oIFF.isHostile) continue;

                auto& oTrans = missiles.get<Transform2D>(otherMissile);
                float distNmSq = MathUtils::LengthSq(MathUtils::Sub(mTrans.pos, oTrans.pos));
                float altDiffMeters = std::abs(mTrans.altitude - oTrans.altitude);

                if (distNmSq <= warhead.lethalRadiusNmSq && altDiffMeters <= FUSE_ALTITUDE_TOLERANCE_METERS) {
                    float simTime = registry.ctx().contains<float>() ? registry.ctx().get<float>() : 0.0f;
                    auto& otherWarhead = missiles.get<Warhead>(otherMissile);

                    MetricsLogger::Log(simTime, "INTERCEPT", (uint32_t)warhead.shooter, 3, warhead.weaponId, std::sqrt(distNmSq), "KILLED_THREAT");
                    MetricsLogger::Log(simTime, "SHOT_DOWN", (uint32_t)otherWarhead.shooter, 0, otherWarhead.weaponId, std::sqrt(distNmSq), "INTERCEPTED");

                    registry.emplace_or_replace<DeadTag>(missile);
                    registry.emplace_or_replace<DeadTag>(otherMissile);
                    if (VERBOSE_COMBAT_LOG) {
                        std::cout << "[INTERCEPT] Missile " << (uint32_t)missile
                                  << " destroyed missile " << (uint32_t)otherMissile << "\n";
                    }
                    break;
                }

            }
        }
    }
private:
    // Select the best weapon for a given threat type
    static std::string SelectWeapon(const Magazine& mag, const RadarTrack& threat) {
        // Priority order depends on threat class
        // Missiles need interceptors; ships need anti-ship missiles

        bool needInterceptor = (threat.classification == ThreatClass::MISSILE_INBOUND ||
                                threat.classification == ThreatClass::AIRCRAFT);

        for (const auto& [weaponId, count] : mag.currentAmmo) {
            if (count <= 0) continue;

            const MissileStats& m = TacticalDatabase::GetMissile(weaponId);

            if (needInterceptor && m.isInterceptor) return weaponId;
            if (!needInterceptor && !m.isInterceptor) return weaponId;
        }

        // Fallback: use anything available if no ideal weapon
        for (const auto& [weaponId, count] : mag.currentAmmo) {
            if (count > 0) return weaponId;
        }

        return ""; // No ammo at all
    }

    // Check if we already have a missile heading toward this threat position
    static int CountActiveShotsAgainst(const EngagementLog& log, const RadarTrack& threat) {
        int count = 0;
        for (const auto& eng : log.current) {
            if (!eng.missileAlive) continue;
            float dist = MathUtils::GetDistance(eng.targetPos, threat.pos);
            if (dist < DATALINK_ENGAGEMENT_CORRELATION_RADIUS_NM) {
                count++;
            }
        }
        return count;
    }

    // How many missiles to fire at this threat?
    // Based on estimated probability of kill and threat importance
    static int DetermineSalvoSize(const RadarTrack& threat) {
        switch (threat.classification) {
            case ThreatClass::MISSILE_INBOUND:
                // Inbound missiles get two shots immediately - too important to risk
                // Pk per shot ~0.7, two shots gives ~0.91 Pk
                if (threat.timeToImpactSec < 60.0f) return 2; // Danger close - ripple fire
                return 1; // Time to assess first shot
            case ThreatClass::AIRCRAFT:
                return 1; // One shot, reassess
            case ThreatClass::SURFACE_SHIP:
                return 1; // Shoot-look-shoot
            default:
                return 1;
        }
    }

    // Extract the actual launch logic from CombatSystem into its own function
    static entt::entity LaunchMissile(
        entt::registry& registry,
        entt::entity shooter,
        const Transform2D& shooterTransform,
        const Kinematics& shooterKin,
        const IFF& shooterIFF,
        const RadarTrack& target,
        const MissileStats& mStats,
        const std::string& weaponId)
    {
        auto missile = registry.create();

        MathUtils::Vec2 initialHeading = { 1.0f, 0.0f }; // fallback
        MathUtils::Vec2 toTarget = MathUtils::Sub(target.pos, shooterTransform.pos);
        float distToTarget = MathUtils::Length(toTarget);
        if (distToTarget > 0.001f) {
            initialHeading = MathUtils::Scale(toTarget, 1.0f / distToTarget);
        }
        registry.emplace<SeekerHead>(missile,
            shooter,
            mStats.seekerType,
            mStats.seekerRangeNM,
            GuidancePhase::MIDCOURSE_DATALINK,
            target.pos,
            target.vel,
            target.altitude,
            mStats.isInterceptor
        );
        registry.emplace<Transform2D>(missile,
            shooterTransform.pos, shooterTransform.altitude, shooterTransform.heading);

        float launchSpeedKnots = mStats.maxSpeedKnots * LAUNCH_SPEED_FRACTION;
        registry.emplace<Kinematics>(missile,
            shooterKin.velocity,  // velocity
            initialHeading,    // headingVector
            mStats.maxSpeedKnots, // maxSpeedKnots
            launchSpeedKnots,  // currentSpeedKnots
            mStats.maxSpeedKnots, // desiredSpeedKnots
            50.0f);               // accelerationRate

        registry.emplace<Aerodynamics>(missile,
            mStats.massKg, 1.0f / mStats.massKg, mStats.areaM2,
            mStats.thrustNewtons, mStats.maxFuelKg, mStats.burnRateKgSec,
            mStats.cruiseAltitudeMeters);

        float baseReliability = 0.90f; // Todo: Pull this from JSON file
        registry.emplace<Warhead>(missile,
            mStats.warheadYield, mStats.lethalRadiusNM,
            mStats.lethalRadiusNM * mStats.lethalRadiusNM,
            baseReliability, weaponId, shooter);

        registry.emplace<RadarSignature>(missile, mStats.rcs, mStats.rcsFourthRoot);
        registry.emplace<IFF>(missile, shooterIFF.isHostile);

        float simTime = registry.ctx().contains<float>() ? registry.ctx().get<float>() : 0.0f;

        MetricsLogger::Log(simTime, "FIRE", (uint32_t)shooter, (int)target.classification, weaponId, distToTarget, "IN_FLIGHT");

        return missile;
    }
};