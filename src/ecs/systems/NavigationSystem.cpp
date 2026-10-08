#include "../Systems.h"
#include "../Components.h"

#include "../../core/Constants.h"
#include "../../core/Physics.h"
#include "../../utils/MetricsLogger.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>

namespace {
    // ========================
    // === MISSILE GUIDANCE ===
    // ========================
    RadarTrack* FindTerminalTrack(SeekerHead& seeker, Transform2D& trans,
                                  Kinematics& kin, std::vector<RadarTrack>& myTracks)
    {
        // Where the target we were assigned should be by now, if it kept its course
        const MathUtils::Vec2 expectedTargetPos =
            MathUtils::ExtrapolatePosition(seeker.targetPos, seeker.targetVel, seeker.timeSinceLastCorrelation);

        float closestDist = 999999.0f;
        RadarTrack* best = nullptr;

        for (auto& track : myTracks) {
            const float distToTrack = MathUtils::GetDistance(trans.pos, track.pos);
            if (distToTrack < 0.001f) continue;

            // Rule 1: it must be inside the seeker's forward-facing cone
            if (MathUtils::AngleOffHeadingDegrees(kin.headingVector, trans.pos, track.pos) > SEEKER_FOV_DEGREES) continue;

            // Rule 2: it must be near where our assigned target should be
            if (MathUtils::GetDistance(expectedTargetPos, track.pos) > SEEKER_IDENTITY_GATE_NM) continue;

            // Rule 3: it must be moving like our assigned target was
            if (MathUtils::VelocityDifferenceKnots(track.vel, seeker.targetVel) > MAX_PLAUSIBLE_VEL_CHANGE_KNOTS) continue;

            if (distToTrack < closestDist) {
                closestDist = distToTrack;
                best = &track;
            }
        }
        if (best && closestDist < seeker.rangeNM && best->ageSec < TRACK_ACTIONABLE_FRESHNESS_SEC) return best;
        return nullptr;
    }

    void RetargetTerminalMissile(entt::registry& registry, entt::entity entity, SeekerHead& seeker) {
        bool missileIsHostile = registry.get<IFF>(entity).isHostile;
        RadarTrack* bestNetworkTarget = nullptr;
        float bestScore = -1.0f;

        auto& networkDetectionData = registry.ctx().get<RadarDetectionData>();
        auto radarShips = registry.view<RadarEmitter, IFF>();
        for (auto observer : radarShips) {
            if (radarShips.get<IFF>(observer).isHostile != missileIsHostile) continue;
            for (auto& track : networkDetectionData.activeTracks[observer]) {
                if (track.ageSec > TRACK_ACTIONABLE_FRESHNESS_SEC) continue;
                if (track.consistentObservationSec < 1.0f) continue;
                if (track.threatScore > bestScore) {
                    bestScore = track.threatScore;
                    bestNetworkTarget = &track;
                }
            }
        }

        if (bestNetworkTarget) {
            seeker.targetPos = bestNetworkTarget->pos;
            seeker.targetVel = bestNetworkTarget->vel;
            seeker.targetAltitude = bestNetworkTarget->altitude;
            seeker.timeSinceLastCorrelation = 0.0f;
            seeker.hasLiveTarget = true;
        } else {
            Systems::DestroyMissile(registry, entity, MissileEventType::SELF_DESTRUCT_NO_TARGET);
        }
    }

    void UpdateMidcourseGuidance( entt::registry& registry, entt::entity entity,
            Transform2D& trans, Kinematics& kin, SeekerHead& seeker, float deltaTime) {
        seeker.timeSinceLaunchSec += deltaTime;
        bool missileIsHostile = registry.get<IFF>(entity).isHostile;

        // Network-wide datalink: check EVERY friendly ship's radar picture
        RadarTrack* best = nullptr;
        float closestDist = DATALINK_ENGAGEMENT_CORRELATION_RADIUS_NM;

        auto& detectionData = registry.ctx().get<RadarDetectionData>();
        auto radarShips = registry.view<RadarEmitter, IFF>();

        for (auto observer : radarShips) {
            if (radarShips.get<IFF>(observer).isHostile != missileIsHostile) continue;

            for (auto& track : detectionData.activeTracks[observer]) {
                MathUtils::Vec2 predictedTargetPos = MathUtils::ExtrapolatePosition(seeker.targetPos,
                    seeker.targetVel, seeker.timeSinceLaunchSec);

                float d = MathUtils::GetDistance(predictedTargetPos, track.pos);
                if (d < closestDist) {
                    closestDist = d;
                    best = &track;
                }
            }
        }
        if (best) {
            seeker.targetPos = best->pos;
            seeker.targetVel = best->vel;
            seeker.targetAltitude = best->altitude;
            seeker.timeSinceLastCorrelation = 0.0f;
            seeker.hasLiveTarget = true;
        } else {
            seeker.timeSinceLastCorrelation += deltaTime;
            seeker.hasLiveTarget = false;
            if (seeker.timeSinceLastCorrelation > DATALINK_TIMEOUT_SEC) seeker.phase = GuidancePhase::INERTIAL_BLIND;
        }

        MathUtils::Vec2 interceptPos = MathUtils::PredictIntercept(
            trans.pos, kin.currentSpeedKnots, seeker.targetPos, seeker.targetVel);

        float distToIntercept = MathUtils::GetDistance(interceptPos, trans.pos);

        if (distToIntercept > 0.001f) {
            MathUtils::Vec2 desiredHeading = MathUtils::GetDirection(trans.pos, interceptPos);

            float currentAngle = std::atan2(kin.headingVector.y, kin.headingVector.x) * RAD_TO_DEG;
            float desiredAngle = std::atan2(desiredHeading.y, desiredHeading.x) * RAD_TO_DEG;
            float angleDiff = MathUtils::GetShortestAngleDiff(currentAngle, desiredAngle);

            float maxTurnRateDeg = MathUtils::GetMaxTurnRateDegSec(kin.currentSpeedKnots, seeker.maxLateralGs);
            float maxStep = maxTurnRateDeg * deltaTime;
            float clampedStep = std::clamp(angleDiff, -maxStep, maxStep);

            float newAngle = currentAngle + clampedStep;
            kin.headingVector.x = std::cos(newAngle * DEG_TO_RAD);
            kin.headingVector.y = std::sin(newAngle * DEG_TO_RAD);
        }

        if (seeker.type == SeekerType::ACTIVE_RADAR && distToIntercept < seeker.rangeNM &&
            seeker.timeSinceLaunchSec > MIN_TIME_BEFORE_SEEKER_ACTIVATION_SEC) {
            seeker.phase = GuidancePhase::TERMINAL_PITBULL;
            registry.emplace_or_replace<RadarEmitter>(entity,seeker.rangeNM, seeker.rangeNM * seeker.rangeNM, 0.25f, 0.0f);

            if (VERBOSE_COMBAT_LOG) {
                std::cout << "[T=" << Systems::GetSimTime(registry) << "s] [SEEKER] Missile " << (uint32_t)entity << " going active, dist to intercept: "
                                      << distToIntercept << "nm\n";
            }
            }
    }

    void UpdateTerminalGuidance(entt::registry& registry, entt::entity entity,
                Transform2D& trans, Kinematics& kin, SeekerHead& seeker, float deltaTime) {
        auto& myTracks = registry.ctx().get<RadarDetectionData>().activeTracks[entity];

        bool foundCorrelatedTrack = false;
        if (!myTracks.empty()) {
            RadarTrack* best = FindTerminalTrack(seeker, trans, kin, myTracks);
            if (best) {
                seeker.targetPos = best->pos;
                seeker.targetVel = best->vel;
                seeker.targetAltitude = best->altitude;
                seeker.timeSinceLastCorrelation = 0.0f;
                foundCorrelatedTrack = true;
            }
            seeker.hasLiveTarget = foundCorrelatedTrack;
        }
        if (foundCorrelatedTrack) {
            float maxTurnRateDeg = MathUtils::GetMaxTurnRateDegSec(kin.currentSpeedKnots, seeker.maxLateralGs);
            float turnRateDeg = MathUtils::CalculateProNavTurnRate(
                trans.pos, kin.velocity, seeker.targetPos, seeker.targetVel, 4.0f);
            turnRateDeg = std::clamp(turnRateDeg, -maxTurnRateDeg, maxTurnRateDeg);

            float currentAngle = std::atan2(kin.headingVector.y, kin.headingVector.x) * RAD_TO_DEG;
            currentAngle += (turnRateDeg * deltaTime);
            kin.headingVector.x = std::cos(currentAngle * DEG_TO_RAD);
            kin.headingVector.y = std::sin(currentAngle * DEG_TO_RAD);
        } else {
            seeker.timeSinceLastCorrelation += deltaTime;

            if (seeker.timeSinceLastCorrelation > DATALINK_TIMEOUT_SEC) {
                RetargetTerminalMissile(registry, entity, seeker);
            }
        }
    }

    void UpdateInertialGuidance(entt::registry& registry, entt::entity entity,
            Transform2D& trans, Kinematics& kin, SeekerHead& seeker, float deltaTime) {
        seeker.timeSinceLastCorrelation += deltaTime;

        float distToTargetNM = MathUtils::GetDistance(trans.pos, seeker.targetPos);

        if (distToTargetNM > 0.001f) {
            MathUtils::Vec2 desiredHeading = MathUtils::GetDirection(trans.pos,seeker.targetPos);

            float currentAngle = std::atan2(kin.headingVector.y, kin.headingVector.x) * RAD_TO_DEG;
            float desiredAngle = std::atan2(desiredHeading.y, desiredHeading.x) * RAD_TO_DEG;
            float angleDiff = MathUtils::GetShortestAngleDiff(currentAngle, desiredAngle);

            float maxTurnRateDeg = MathUtils::GetMaxTurnRateDegSec(kin.currentSpeedKnots, seeker.maxLateralGs);
            float maxStep = maxTurnRateDeg * deltaTime;
            float clampedStep = std::clamp(angleDiff, -maxStep, maxStep);

            float newAngle = currentAngle + clampedStep;
            kin.headingVector.x = std::cos(newAngle * DEG_TO_RAD);
            kin.headingVector.y = std::sin(newAngle * DEG_TO_RAD);
        }

        constexpr float INERTIAL_ARRIVAL_NM = 0.5f;
        constexpr float INERTIAL_MAX_FLIGHT_SEC = 20.0f;
        if (distToTargetNM < INERTIAL_ARRIVAL_NM || seeker.timeSinceLastCorrelation > INERTIAL_MAX_FLIGHT_SEC) {
            Systems::DestroyMissile(registry, entity, MissileEventType::SELF_DESTRUCT_INERTIAL_LOST);
        }
    }

    void UpdateMissileGuidance( entt::registry& registry, entt::entity entity,
            Transform2D& trans, Kinematics& kin, SeekerHead& seeker, float deltaTime) {
        if (seeker.phase == GuidancePhase::MIDCOURSE_DATALINK) {
            UpdateMidcourseGuidance(registry, entity, trans, kin, seeker, deltaTime);
        }
        else if (seeker.phase == GuidancePhase::TERMINAL_PITBULL) {
            UpdateTerminalGuidance(registry, entity, trans, kin, seeker, deltaTime);
        }
        else if (seeker.phase == GuidancePhase::INERTIAL_BLIND) {
            UpdateInertialGuidance(registry, entity, trans, kin, seeker, deltaTime);
        }
    }

    // =====================
    // === SHIP GUIDANCE ===
    // =====================
    RadarTrack* FindClosestShipTarget(entt::registry& registry, entt::entity entity, Transform2D& trans) {
        auto& myTracks = registry.ctx().get<RadarDetectionData>().activeTracks[entity];
        RadarTrack* bestTrack = nullptr;
        float bestDistSq = std::numeric_limits<float>::max();

        for (auto& track : myTracks) {
            if (track.ageSec > TRACK_ACTIONABLE_FRESHNESS_SEC) continue;

            float distSq = MathUtils::LengthSq(MathUtils::Sub(trans.pos, track.pos));

            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestTrack = &track;
            }
        }
        return bestTrack;
    }

    void UpdateShipTarget(entt::registry& registry, entt::entity entity, Transform2D& trans, AutonomousGuidance& brain) {
        RadarTrack* bestTrack = FindClosestShipTarget(registry, entity, trans);

        if (bestTrack) {
            brain.cachedTargetPos = bestTrack->pos;
            brain.hasTarget = true;
        } else {
            brain.hasTarget = false;
        }
    }

    void UpdatePatrol(Transform2D& trans, Kinematics& kin, AutonomousGuidance& brain) {
        if (!brain.waypoints.empty()) {
            MathUtils::Vec2 targetWp = brain.waypoints[brain.currentWaypointIndex];
            float distToWp = MathUtils::GetDistance(trans.pos, targetWp);

            if (distToWp < 0.5f) {
                brain.currentWaypointIndex = (brain.currentWaypointIndex + 1) % brain.waypoints.size();
                targetWp = brain.waypoints[brain.currentWaypointIndex];
                distToWp = MathUtils::GetDistance(trans.pos, targetWp);
            }

            if (distToWp > MATH_EPSILON) {
                kin.headingVector = MathUtils::GetDirection(trans.pos, targetWp);
            }
            kin.desiredSpeedKnots = kin.maxSpeedKnots;
        } else {
            kin.desiredSpeedKnots = 0.0f;
        }
    }

    void UpdateTransit(Transform2D& trans, Kinematics& kin, AutonomousGuidance& brain) {
        if (!brain.waypoints.empty()) {
            MathUtils::Vec2 destination = brain.waypoints.back();
            float distToDestination = MathUtils::GetDistance(trans.pos, destination);

            if (distToDestination > 0.5f) {
                // Still traveling - point toward destination and go full speed
                kin.headingVector = MathUtils::GetDirection(trans.pos, destination);
                kin.desiredSpeedKnots = kin.maxSpeedKnots;
            } else {
                // Arrived - hand off to patrol behavior
                brain.currentState = TacticalState::PATROL;
                brain.currentWaypointIndex = 0;
            }
        } else {
            // No waypoints assigned, just stop and patrol in place
            brain.currentState = TacticalState::PATROL;
        }
    }

    void UpdateShipTacticalState(entt::registry& registry, entt::entity entity, Transform2D& trans,
            Kinematics& kin, AutonomousGuidance& brain, float deltaTime)
    {
        float closestDist = MathUtils::GetDistance(trans.pos, brain.cachedTargetPos);

        bool foundTarget = brain.hasTarget;

        if (foundTarget && closestDist < registry.get<RadarEmitter>(entity).rangeNM) {
            if (brain.currentState == TacticalState::PATROL || brain.currentState == TacticalState::TRANSIT) {
                if (!brain.pendingEngagement) {
                    brain.pendingEngagement = true;
                    std::uniform_real_distribution<float> delayDist(2.0f, 8.0f);
                    brain.reactionTimer = delayDist(registry.ctx().get<std::mt19937>());
                }

                brain.reactionTimer -= deltaTime;
                if (brain.reactionTimer <= 0.0f) {
                    brain.currentState = TacticalState::INTERCEPT;
                    brain.pendingEngagement = false;
                    if (VERBOSE_COMBAT_LOG) {
                        std::cout << "[T=" << Systems::GetSimTime(registry) << "s] [STATE] Entity " << (uint32_t)entity << " PATROL -> INTERCEPT (range: "
                                          << closestDist << "nm)\n";
                    }
                }
            }
        }

        switch (brain.currentState) {
            case TacticalState::DEFEND: {
                if (foundTarget) {
                    if (closestDist < brain.desiredStandoffNM ) {
                        if (closestDist > MATH_EPSILON) {
                            MathUtils::Vec2 awayDirection = MathUtils::GetDirection(brain.cachedTargetPos,trans.pos);
                            kin.headingVector = awayDirection;
                        }
                        kin.desiredSpeedKnots = kin.maxSpeedKnots;
                    } else kin.desiredSpeedKnots = 0.0f;
                } else brain.currentState = TacticalState::PATROL;

                break;
            }

            case TacticalState::INTERCEPT: {
                if (foundTarget) {
                    if (closestDist > brain.desiredStandoffNM + 1.0f) {
                        if (closestDist > MATH_EPSILON) {
                            kin.headingVector = MathUtils::GetDirection(trans.pos, brain.cachedTargetPos);
                        }

                        kin.desiredSpeedKnots = kin.maxSpeedKnots;
                    } else {
                        brain.currentState = TacticalState::STANDOFF;
                        kin.desiredSpeedKnots = 0.0f;
                        if (VERBOSE_COMBAT_LOG) {
                            std::cout << "[T=" << Systems::GetSimTime(registry) << "s] [STATE] Entity " << (uint32_t)entity << " INTERCEPT -> STANDOFF (range: "
                                             << closestDist << "nm, standoff target: " << brain.desiredStandoffNM << "nm)\n";
                        }
                    }
                } else brain.currentState = TacticalState::PATROL;

                break;
            }

            case TacticalState::STANDOFF: {
                if (foundTarget) {
                    if (closestDist > brain.desiredStandoffNM + 1.0f) {
                        kin.headingVector = MathUtils::GetDirection(trans.pos, brain.cachedTargetPos);
                        kin.desiredSpeedKnots = kin.maxSpeedKnots;
                    } else if (closestDist < brain.desiredStandoffNM - 1.0f) {
                        kin.headingVector = MathUtils::GetDirection(trans.pos, brain.cachedTargetPos);
                        kin.desiredSpeedKnots = kin.maxSpeedKnots;
                    } else kin.desiredSpeedKnots = 0.0f;
                } else brain.currentState = TacticalState::PATROL;

                break;
            }

            case TacticalState::PATROL: {
                UpdatePatrol(trans, kin, brain);

                break;
            }

            case TacticalState::TRANSIT: {
                UpdateTransit(trans, kin, brain);

                break;
            }

            default: {
                kin.desiredSpeedKnots = 0.0f;

                break;
            }
        }
    }

    void UpdateShipGuidance(entt::registry& registry, entt::entity entity, Transform2D& trans,
        Kinematics& kin, AutonomousGuidance& brain, float deltaTime) {
        auto* myIFF = registry.try_get<IFF>(entity);

        if (!myIFF) return;

        auto* myRadar = registry.try_get<RadarEmitter>(entity);
        if (!myRadar) return;

        UpdateShipTarget(registry, entity, trans, brain);

        UpdateShipTacticalState(registry, entity, trans, kin, brain, deltaTime);
    }
}

    float Systems::PlanMissileAltitude(const Transform2D& trans, const Kinematics& kin,
            const Aerodynamics& aero, const SeekerHead&seeker) {
        float targetAlt = trans.altitude;  // no live target: hold altitude

        if (seeker.hasLiveTarget) {
            targetAlt = aero.cruiseAltitudeMeters < 100.0f ? 10000.0f : aero.cruiseAltitudeMeters;

            float distToTargetNM = MathUtils::GetDistance(trans.pos, seeker.targetPos);

            if (distToTargetNM > 0.001f) {
                MathUtils::Vec2 missileVel = MathUtils::GetVelocityNmPerSec(kin.headingVector, kin.currentSpeedKnots);

                float missileClosingSpeedNmSec = MathUtils::GetClosingSpeedNmPerSec(trans.pos, missileVel, seeker.targetPos);
                float targetClosingSpeedNmSec = MathUtils::GetClosingSpeedNmPerSec(seeker.targetPos, seeker.targetVel, trans.pos);

                float combinedClosingSpeedNmSec = missileClosingSpeedNmSec + targetClosingSpeedNmSec;

                if (combinedClosingSpeedNmSec > 0.0001f) {
                    float timeToImpactSec = distToTargetNM / combinedClosingSpeedNmSec;

                    float altDiffMeters  = std::abs(seeker.targetAltitude - trans.altitude);

                    float maxLateralAccelMps2 = seeker.maxLateralGs * Physics::GRAVITY;

                    float timeToDescendSec = 0.0f;

                    if (maxLateralAccelMps2 > 0.0f) {
                        float peakVerticalSpeedMps = std::sqrt(2.0f * maxLateralAccelMps2 * altDiffMeters);

                        peakVerticalSpeedMps = std::min(peakVerticalSpeedMps, kin.currentSpeedKnots * MPS_PER_KNOT);

                        timeToDescendSec = peakVerticalSpeedMps / maxLateralAccelMps2;
                    }

                    if (timeToImpactSec <= (timeToDescendSec + DESCENT_SAFETY_MARGIN_SEC)) {
                        targetAlt = seeker.targetAltitude;
                    }
                }
            }
        }

        return targetAlt;
    }
    void Systems::NavigationSystem(entt::registry& registry, float deltaTime) {
        auto view = registry.view<Transform2D, Kinematics>();
        for (auto entity : view) {
            auto& trans = view.get<Transform2D>(entity);
            auto& kin = view.get<Kinematics>(entity);

            if (auto* seeker = registry.try_get<SeekerHead>(entity)) {
                UpdateMissileGuidance(registry,entity,trans,kin, *seeker, deltaTime);
                // Tell the physics what altitude to fly at (MovementSystem just obeys this number)
                if (!registry.all_of<DeadTag>(entity)) {
                    if (auto* aero = registry.try_get<Aerodynamics>(entity)) {
                        aero->desiredAltitudeMeters = PlanMissileAltitude(trans, kin, *aero, *seeker);
                    }
                }
            }

            if (auto* brain = registry.try_get<AutonomousGuidance>(entity)) {
                UpdateShipGuidance(registry, entity, trans, kin, *brain, deltaTime);
            }
        }
    }
