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
                Kinematics& kin, std::vector<RadarTrack>& myTracks) {
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

            if (angleFromHeadingDeg > SEEKER_FOV_DEGREES) continue;

            float timeSinceAssignedTargetUpdateSec = seeker.timeSinceLastCorrelation;
            MathUtils::Vec2 predictedAssignedTargetPos = MathUtils::Add(
                seeker.targetPos, MathUtils::Scale(seeker.targetVel, timeSinceAssignedTargetUpdateSec));

            float distFromAssignedTarget = MathUtils::GetDistance(predictedAssignedTargetPos, track.pos);
            if (distFromAssignedTarget > SEEKER_IDENTITY_GATE_NM) continue;

            float velDiffKnots = MathUtils::Length(MathUtils::Sub(track.vel, seeker.targetVel)) * 3600.0f;
            if (velDiffKnots > MAX_PLAUSIBLE_VEL_CHANGE_KNOTS) continue;

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
                MathUtils::Vec2 predictedTargetPos = MathUtils::Add(
                    seeker.targetPos,
                    MathUtils::Scale(seeker.targetVel, seeker.timeSinceLaunchSec));

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

        MathUtils::Vec2 toIntercept = MathUtils::Sub(interceptPos, trans.pos);
        float distToIntercept = MathUtils::Length(toIntercept);

        if (distToIntercept > 0.001f) {
            MathUtils::Vec2 desiredHeading = MathUtils::Scale(toIntercept, 1.0f / distToIntercept);

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
            MathUtils::Vec2 desiredHeading = MathUtils::Scale(
                MathUtils::Sub(seeker.targetPos, trans.pos), 1.0f / distToTargetNM);

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
                kin.headingVector = MathUtils::Scale(MathUtils::Sub(targetWp, trans.pos), 1.0f / distToWp);
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
                kin.headingVector = MathUtils::Scale(
                    MathUtils::Sub(destination, trans.pos),
                    1.0f / distToDestination);
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
            Kinematics& kin, AutonomousGuidance& brain, float deltaTime) {
        float closestDist = MathUtils::GetDistance(trans.pos, brain.cachedTargetPos);
        MathUtils::Vec2 toTarget = MathUtils::Sub(brain.cachedTargetPos, trans.pos);
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
                            MathUtils::Vec2 awayDirection = MathUtils::Scale(toTarget, -1.0f / closestDist);
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
                            kin.headingVector = MathUtils::Scale(toTarget, 1.0f / closestDist);
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
                        kin.headingVector = MathUtils::Scale(toTarget, 1.0f / closestDist);
                        kin.desiredSpeedKnots = kin.maxSpeedKnots;
                    } else if (closestDist < brain.desiredStandoffNM - 1.0f) {
                        kin.headingVector = MathUtils::Scale(toTarget, -1.0f / closestDist);
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

    void Systems::NavigationSystem(entt::registry& registry, float deltaTime) {
        auto view = registry.view<Transform2D, Kinematics>();
        for (auto entity : view) {
            auto& trans = view.get<Transform2D>(entity);
            auto& kin = view.get<Kinematics>(entity);

            if (auto* seeker = registry.try_get<SeekerHead>(entity)) {
                UpdateMissileGuidance(registry,entity,trans,kin, *seeker, deltaTime);
            }

            if (auto* brain = registry.try_get<AutonomousGuidance>(entity)) {
                UpdateShipGuidance(registry, entity, trans, kin, *brain, deltaTime);
            }
        }
    }
