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
    RadarTrack* FindTerminalTrack(
     const SeekerHead& seeker,
     Transform2D& trans,
     Kinematics& kin,
     std::vector<RadarTrack>& myTracks)
    {
        if (seeker.targetGlobalTrackId == NO_GLOBAL_TRACK)
            return nullptr;

        for (auto& track : myTracks)
        {
            // ========================================================
            // 1. TARGET IDENTITY
            // ========================================================

            if (track.globalTrackId != seeker.targetGlobalTrackId)
                continue;

            // ========================================================
            // 2. TRACK QUALITY
            // ========================================================

            if (track.lifecycle == TrackLifecycle::MISSING ||
                track.lifecycle == TrackLifecycle::DESTROYED ||
                track.lifecycle == TrackLifecycle::TENTATIVE)
                continue;

            if (!track.hasVelocity)
                continue;

            // ========================================================
            // 3. SEEKER RANGE
            // ========================================================

            const float distToTrack =
                MathUtils::GetDistance(trans.pos, track.pos);

            if (distToTrack < 0.001f)
                continue;

            if (distToTrack > seeker.rangeNM)
                continue;

            // ========================================================
            // 4. SEEKER FIELD OF VIEW
            // ========================================================

            if (MathUtils::AngleOffHeadingDegrees(
                kin.headingVector,
                trans.pos,
                track.pos) > SEEKER_FOV_DEGREES)
            {
                continue;
            }

            // This is the assigned target, with a confirmed
            // track and valid velocity estimate.
            return &track;
        }

        return nullptr;
    }

    void UpdateMidcourseGuidance(
        entt::registry& registry,
        entt::entity entity,
        Transform2D& trans,
        Kinematics& kin,
        SeekerHead& seeker,
        float deltaTime)
    {
        seeker.timeSinceLaunchSec += deltaTime;

        const bool missileIsHostile =
            registry.get<IFF>(entity).isHostile;

        // ============================================================
        // 1. LOOK UP THE ASSIGNED GLOBAL TRACK
        // ============================================================

        auto& masterTable =
            registry.ctx().get<MasterTrackTable>();

        const auto& side =
            masterTable.GetSide(missileIsHostile);

        bool receivedNewMeasurement = false;

        if (seeker.targetGlobalTrackId != NO_GLOBAL_TRACK)
        {
            auto it = side.activeGlobalTracks.find(
                seeker.targetGlobalTrackId
            );

            if (it != side.activeGlobalTracks.end())
            {
                const RadarTrack& target =
                    it->second.track;

                if (target.lifecycle == TrackLifecycle::CONFIRMED)
                {
                    constexpr float TIME_EPSILON = 0.0001f;

                    // Only consume an update when the global track
                    // has received a genuinely newer measurement.
                    if (target.lastMeasurementTimeSec >
                        seeker.lastTargetMeasurementTimeSec + TIME_EPSILON)
                    {
                        seeker.targetPos = target.pos;
                        seeker.targetVel = target.vel;

                        seeker.targetAltitude = target.altitude;

                        seeker.targetVerticalVelocityNMPerSec =
                            target.verticalVelocityNMPerSec;

                        seeker.lastTargetMeasurementTimeSec =
                            target.lastMeasurementTimeSec;

                        seeker.timeSinceLastCorrelation = 0.0f;

                        receivedNewMeasurement = true;
                    }
                }
            }
        }

        // ============================================================
        // 2. HANDLE DATALINK LOSS
        // ============================================================

        if (!receivedNewMeasurement)
        {
            seeker.timeSinceLastCorrelation += deltaTime;
        }

        seeker.hasLiveTarget =
            seeker.timeSinceLastCorrelation <= DATALINK_TIMEOUT_SEC;

        // Semi-active seekers depend on external tracking support.
        // Active radar seekers can continue inertially until their
        // own radar enters the terminal phase.
        if (!seeker.hasLiveTarget &&
            seeker.type != SeekerType::ACTIVE_RADAR)
        {
            seeker.phase = GuidancePhase::INERTIAL_BLIND;
        }

        // ============================================================
        // 3. PREDICT TARGET POSITION
        // ============================================================

        const MathUtils::Vec2 predictedTargetPos =
            MathUtils::ExtrapolatePosition(
                seeker.targetPos,
                seeker.targetVel,
                seeker.timeSinceLastCorrelation
            );

        // ============================================================
        // 4. CALCULATE INTERCEPT
        // ============================================================

        const MathUtils::Vec2 interceptPos =
            MathUtils::PredictIntercept(
                trans.pos,
                kin.currentSpeedKnots,
                predictedTargetPos,
                seeker.targetVel
            );

        const float distToIntercept =
            MathUtils::GetDistance(trans.pos, interceptPos);

        if (distToIntercept > 0.001f)
        {
            const MathUtils::Vec2 desiredHeading =
                MathUtils::GetDirection(trans.pos, interceptPos);

            const float currentAngle =
                std::atan2(
                    kin.headingVector.y,
                    kin.headingVector.x
                ) * RAD_TO_DEG;

            const float desiredAngle =
                std::atan2(
                    desiredHeading.y,
                    desiredHeading.x
                ) * RAD_TO_DEG;

            const float angleDiff =
                MathUtils::GetShortestAngleDiff(
                    currentAngle,
                    desiredAngle
                );

            const float maxTurnRateDeg =
                MathUtils::GetMaxTurnRateDegSec(
                    kin.currentSpeedKnots,
                    seeker.maxLateralGs
                );

            const float maxStep =
                maxTurnRateDeg * deltaTime;

            const float clampedStep =
                std::clamp(angleDiff, -maxStep, maxStep);

            const float newAngle =
                currentAngle + clampedStep;

            kin.headingVector.x =
                std::cos(newAngle * DEG_TO_RAD);

            kin.headingVector.y =
                std::sin(newAngle * DEG_TO_RAD);
        }

        // ============================================================
        // 5. TERMINAL SEEKER ACTIVATION
        // ============================================================

        if (seeker.phase == GuidancePhase::MIDCOURSE_DATALINK &&
            seeker.type != SeekerType::PASSIVE_IR &&
            distToIntercept < seeker.rangeNM &&
            seeker.timeSinceLaunchSec > MIN_TIME_BEFORE_SEEKER_ACTIVATION_SEC)
        {
            // Active radar turns on its own radar.
            if (seeker.type == SeekerType::ACTIVE_RADAR)
            {
                registry.emplace_or_replace<RadarEmitter>(
                    entity,
                    seeker.rangeNM,
                    seeker.rangeNM * seeker.rangeNM,
                    0.25f,
                    0.0f
                );
            }

            // Semi-active missiles enter terminal guidance using
            // the designated illuminating ship's radar track.
            seeker.phase = GuidancePhase::TERMINAL_PITBULL;

            if (VERBOSE_COMBAT_LOG)
            {
                std::cout
                    << "[T=" << Systems::GetSimTime(registry)
                    << "s] [SEEKER] Missile "
                    << static_cast<uint32_t>(entity)
                    << " entering terminal guidance. Dist: "
                    << distToIntercept << "nm\n";
            }
        }
    }

    void UpdateTerminalGuidance(
        entt::registry& registry,
        entt::entity entity,
        Transform2D& trans,
        Kinematics& kin,
        SeekerHead& seeker,
        float deltaTime)
    {
        auto& detectionData =
            registry.ctx().get<RadarDetectionData>();

        std::vector<RadarTrack>* trackSource = nullptr;

        // ============================================================
        // 1. CHOOSE THE CORRECT TRACK SOURCE
        // ============================================================

        if (seeker.type == SeekerType::ACTIVE_RADAR)
        {
            // Active radar: use the missile's own local radar picture.
            auto it = detectionData.activeTracks.find(entity);

            if (it != detectionData.activeTracks.end())
            {
                trackSource = &it->second;
            }
        }
        else if (seeker.type == SeekerType::SEMI_ACTIVE)
        {
            // Semi-active: the launching platform must remain
            // an active radar observer on the missile's side.
            if (registry.valid(seeker.shooter) &&
                registry.all_of<RadarEmitter, IFF>(seeker.shooter))
            {
                const bool sameSide =
                    registry.get<IFF>(seeker.shooter).isHostile ==
                    registry.get<IFF>(entity).isHostile;

                if (sameSide)
                {
                    auto it =
                        detectionData.activeTracks.find(seeker.shooter);

                    if (it != detectionData.activeTracks.end())
                    {
                        trackSource = &it->second;
                    }
                }
            }
        }

        // PASSIVE_IR currently has no separate IR sensor or
        // measurement pipeline in this codebase. It will coast
        // on its last target estimate until that is implemented.

        // ============================================================
        // 2. FIND THE ASSIGNED TARGET
        // ============================================================

        RadarTrack* best = nullptr;

        if (trackSource)
        {
            best = FindTerminalTrack(
                seeker,
                trans,
                kin,
                *trackSource
            );
        }

        // ============================================================
        // 3. ACCEPT ONLY NEW MEASUREMENTS
        // ============================================================

        if (best)
        {
            constexpr float TIME_EPSILON = 0.0001f;

            if (best->lastMeasurementTimeSec >
                seeker.lastTargetMeasurementTimeSec + TIME_EPSILON)
            {
                seeker.targetPos = best->pos;
                seeker.targetVel = best->vel;

                seeker.targetAltitude = best->altitude;

                seeker.targetVerticalVelocityNMPerSec =
                    best->verticalVelocityNMPerSec;

                seeker.lastTargetMeasurementTimeSec =
                    best->lastMeasurementTimeSec;

                seeker.timeSinceLastCorrelation = 0.0f;
            }
            else
            {
                // The track still exists, but no new measurement
                // has arrived.
                seeker.timeSinceLastCorrelation += deltaTime;
            }
        }
        else
        {
            seeker.timeSinceLastCorrelation += deltaTime;
        }

        seeker.hasLiveTarget =
            best != nullptr &&
            seeker.timeSinceLastCorrelation <= DATALINK_TIMEOUT_SEC;

        // ============================================================
        // 4. PREDICT TARGET POSITION
        // ============================================================

        const MathUtils::Vec2 predictedTargetPos =
            MathUtils::ExtrapolatePosition(
                seeker.targetPos,
                seeker.targetVel,
                seeker.timeSinceLastCorrelation
            );

        // ============================================================
        // 5. TERMINAL GUIDANCE
        // ============================================================

        if (seeker.hasLiveTarget)
        {
            float maxTurnRateDeg =
                MathUtils::GetMaxTurnRateDegSec(
                    kin.currentSpeedKnots,
                    seeker.maxLateralGs
                );

            float turnRateDeg =
                MathUtils::CalculateProNavTurnRate(
                    trans.pos,
                    kin.velocity,
                    predictedTargetPos,
                    seeker.targetVel,
                    4.0f
                );

            turnRateDeg =
                std::clamp(
                    turnRateDeg,
                    -maxTurnRateDeg,
                    maxTurnRateDeg
                );

            float currentAngle =
                std::atan2(
                    kin.headingVector.y,
                    kin.headingVector.x
                ) * RAD_TO_DEG;

            currentAngle += turnRateDeg * deltaTime;

            kin.headingVector.x =
                std::cos(currentAngle * DEG_TO_RAD);

            kin.headingVector.y =
                std::sin(currentAngle * DEG_TO_RAD);
        }
        else
        {
            // No valid current measurement. Continue toward
            // the extrapolated last-known target position.
            const MathUtils::Vec2 desiredHeading =
                MathUtils::GetDirection(
                    trans.pos,
                    predictedTargetPos
                );

            const float currentAngle =
                std::atan2(
                    kin.headingVector.y,
                    kin.headingVector.x
                ) * RAD_TO_DEG;

            const float desiredAngle =
                std::atan2(
                    desiredHeading.y,
                    desiredHeading.x
                ) * RAD_TO_DEG;

            const float angleDiff =
                MathUtils::GetShortestAngleDiff(
                    currentAngle,
                    desiredAngle
                );

            const float maxTurnRateDeg =
                MathUtils::GetMaxTurnRateDegSec(
                    kin.currentSpeedKnots,
                    seeker.maxLateralGs
                );

            const float maxStep =
                maxTurnRateDeg * deltaTime;

            const float clampedStep =
                std::clamp(angleDiff, -maxStep, maxStep);

            const float newAngle =
                currentAngle + clampedStep;

            kin.headingVector.x =
                std::cos(newAngle * DEG_TO_RAD);

            kin.headingVector.y =
                std::sin(newAngle * DEG_TO_RAD);
        }

        // An active radar seeker can keep searching after losing
        // a lock. Semi-active guidance depends on the illuminator.
        if (seeker.timeSinceLastCorrelation > DATALINK_TIMEOUT_SEC &&
            seeker.type != SeekerType::ACTIVE_RADAR)
        {
            seeker.phase = GuidancePhase::INERTIAL_BLIND;
        }
    }

    void UpdateInertialGuidance(entt::registry& registry, entt::entity entity,
            Transform2D& trans, Kinematics& kin, SeekerHead& seeker, float deltaTime)
    {
        seeker.timeSinceLastCorrelation += deltaTime;

        const MathUtils::Vec2 predictedTargetPos =
            MathUtils::ExtrapolatePosition(
                seeker.targetPos,
                seeker.targetVel,
                seeker.timeSinceLastCorrelation
            );

        float distToTargetNM =
            MathUtils::GetDistance(
                trans.pos,
                predictedTargetPos
            );

        if (distToTargetNM > 0.001f) {
            MathUtils::Vec2 desiredHeading = MathUtils::GetDirection(trans.pos,predictedTargetPos);

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

            float distSq = MathUtils::GetDistanceSq(trans.pos, track.pos);

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

        if (foundTarget && closestDist < registry.get<RadarEmitter>(entity).maxRangeNM) {
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
                        kin.headingVector = MathUtils::GetDirection(brain.cachedTargetPos, trans.pos);
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

float Systems::PlanMissileAltitude(
    const Transform2D& trans,
    const Kinematics& kin,
    const Aerodynamics& aero,
    const SeekerHead& seeker)
{
    // No assigned target means there is no target altitude
    // to guide toward.
    if (seeker.targetGlobalTrackId == NO_GLOBAL_TRACK)
        return trans.altitude;

    // Predict where the target should be vertically.
    const float predictedTargetAltitude =
        seeker.targetAltitude +
        seeker.targetVerticalVelocityNMPerSec *
        seeker.timeSinceLastCorrelation *
        METERS_PER_NM;

    // Predict horizontal target position as well.
    const MathUtils::Vec2 predictedTargetPos =
        MathUtils::ExtrapolatePosition(
            seeker.targetPos,
            seeker.targetVel,
            seeker.timeSinceLastCorrelation
        );

    // Initial altitude plan is the missile's cruise altitude.
    float targetAlt =
        aero.cruiseAltitudeMeters < 100.0f
            ? 10000.0f
            : aero.cruiseAltitudeMeters;

    const float distToTargetNM =
        MathUtils::GetDistance(
            trans.pos,
            predictedTargetPos
        );

    if (distToTargetNM > 0.001f)
    {
        const MathUtils::Vec2 missileVel =
            MathUtils::GetVelocityNmPerSec(
                kin.headingVector,
                kin.currentSpeedKnots
            );

        const float missileClosingSpeedNmSec =
            MathUtils::GetClosingSpeedNmPerSec(
                trans.pos,
                missileVel,
                predictedTargetPos
            );

        const float targetClosingSpeedNmSec =
            MathUtils::GetClosingSpeedNmPerSec(
                predictedTargetPos,
                seeker.targetVel,
                trans.pos
            );

        const float combinedClosingSpeedNmSec =
            missileClosingSpeedNmSec +
            targetClosingSpeedNmSec;

        if (combinedClosingSpeedNmSec > 0.0001f)
        {
            const float timeToImpactSec =
                distToTargetNM / combinedClosingSpeedNmSec;

            const float altDiffMeters =
                std::abs(
                    predictedTargetAltitude - trans.altitude
                );

            const float maxLateralAccelMps2 =
                seeker.maxLateralGs * GRAVITY_MPS2;

            float timeToDescendSec = 0.0f;

            if (maxLateralAccelMps2 > 0.0f)
            {
                float peakVerticalSpeedMps =
                    std::sqrt(
                        2.0f *
                        maxLateralAccelMps2 *
                        altDiffMeters
                    );

                peakVerticalSpeedMps =
                    std::min(
                        peakVerticalSpeedMps,
                        kin.currentSpeedKnots * MPS_PER_KNOT
                    );

                timeToDescendSec =
                    peakVerticalSpeedMps /
                    maxLateralAccelMps2;
            }

            if (timeToImpactSec <=
                (timeToDescendSec + DESCENT_SAFETY_MARGIN_SEC))
            {
                targetAlt = predictedTargetAltitude;
            }
        }
    }

    return targetAlt;
}

void Systems::GuidanceSystem(entt::registry& registry, float deltaTime) {
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
