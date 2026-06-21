#pragma once
#include "Components.h"
#include "../core/Constants.h"
#include "../core/Physics.h"
#include <iostream>
#include <execution>

#include "../utils/ThreatAnalysis.h"

constexpr bool VERBOSE_COMBAT_LOG = true;

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
                                    seeker->targetAltitude = track.altitude;
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
                        if (VERBOSE_COMBAT_LOG) {
                            std::cout << "[SEEKER] Missile " << (uint32_t)entity << " going active, dist to intercept: "
                                      << distToIntercept << "nm\n";
                        }
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
                                seeker->targetAltitude = track.altitude;
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

                auto* myRadar = registry.try_get<RadarEmitter>(entity);
                if (!myRadar) continue;
                float detectionRangeSq = myRadar->rangeNmSq;

                // Only re-scan for targets every 2 seconds, not every frame
                brain->targetingCooldown -= deltaTime;

                if (brain->targetingCooldown <= 0.0f) {
                    brain->targetingCooldown = 2.0f;  // Recalculate every 2 seconds
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

                // Use the cached target instead of re-scanning
                float closestDist = MathUtils::GetDistance(trans.pos, brain->cachedTargetPos);
                MathUtils::Vec2 toTarget = MathUtils::Sub(brain->cachedTargetPos, trans.pos);
                bool foundTarget = brain->hasTarget;

                // --- Combat Interrupt ---
                float detectionRange = myRadar ? myRadar->rangeNM : 50.0f;

                if (foundTarget && closestDist < detectionRange) {
                    if (brain->currentState == TacticalState::PATROL || brain->currentState == TacticalState::TRANSIT) {
                        brain->currentState = TacticalState::INTERCEPT;
                        if (VERBOSE_COMBAT_LOG) {
                            std::cout << "[STATE] Entity " << (uint32_t)entity << " PATROL -> INTERCEPT (range: "
                                      << closestDist << "nm)\n";
                        }
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
                    if (distToTarget < 5.0f) {
                        if (seeker->isInterceptor) {
                            targetAlt = seeker->targetAltitude;
                        } else {
                            // Anti-ship weapons sea-skim on terminal approach
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
                    float closestDistSq = 4.0f;
                    int bestMatchIndex = -1;

                    for (size_t i = 0; i < myTracks.size(); ++i) {
                        float dSq = MathUtils::LengthSq(MathUtils::Sub(ping.pos, myTracks[i].pos));
                        if (dSq < closestDistSq) {
                            closestDistSq = dSq;
                            bestMatchIndex = i;
                    }
                }

                if (bestMatchIndex != -1) {
                    MathUtils::Vec2 calculatedVel = MathUtils::Scale(
                        MathUtils::Sub(ping.pos, myTracks[bestMatchIndex].pos),
                        1.0f / timeDelta);
                    myTracks[bestMatchIndex].pos = ping.pos;
                    myTracks[bestMatchIndex].vel = calculatedVel;
                    myTracks[bestMatchIndex].altitude = ping.altitude;
                    myTracks[bestMatchIndex].ageSec = 0.0f;

                    myTracks[bestMatchIndex].speedKnots = MathUtils::Length(calculatedVel) * 3600.0f;
                    MathUtils::Vec2 toObserver = MathUtils::Sub(obsTransform.pos, ping.pos);
                    float dist = MathUtils::Length(toObserver);
                    if (dist > 0.001f) {
                        MathUtils::Vec2 dir = MathUtils::Scale(toObserver, 1.0f / dist);
                        myTracks[bestMatchIndex].closingSpeedKnots =
                            MathUtils::Dot(calculatedVel, dir) * 3600.0f;
                    }

                    myTracks[bestMatchIndex].timeToImpactSec = ThreatAnalysis::EstimateTimeToImpact(
                        obsTransform.pos, ping.pos, calculatedVel);

                    myTracks[bestMatchIndex].classification =
                        ThreatAnalysis::Classify(myTracks[bestMatchIndex].speedKnots, ping.altitude);

                    myTracks[bestMatchIndex].threatScore =
                        ThreatAnalysis::ComputeThreatScore(myTracks[bestMatchIndex]);
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

        // Check if missiles we fired are still alive
        auto shipView = registry.view<EngagementLog>();
        for (auto entity : shipView) {
            auto& log = shipView.get<EngagementLog>(entity);
            for (auto& eng : log.current) {
                if (eng.missileAlive) {
                    // If the missile entity is gone, our shot ended
                    eng.missileAlive = registry.valid(eng.missileEntity);
                }
            }
            // Remove finished engagements
            std::erase_if(log.current, [](const ActiveEngagement& e) {
                return !e.missileAlive;
            });
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

            // Only combat-ready states engage
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
            sortedThreats.clear(); // Wipes contents, KEEPS the allocated capacity
            for (auto& track : tracks) {
                sortedThreats.push_back(&track);
            }
            std::sort(sortedThreats.begin(), sortedThreats.end(),
                [](const RadarTrack* a, const RadarTrack* b) {
                    return a->threatScore > b->threatScore;
                });

            // --- Evaluate each threat and decide ---
            for (RadarTrack* threat : sortedThreats) {
                // DOCTRINE RULE 1: Don't engage targets moving away
                if (threat->closingSpeedKnots <= RETREAT_THRESHOLD_KTS &&
                    threat->classification != ThreatClass::MISSILE_INBOUND) continue;

                // DOCTRINE RULE 2: Select the right weapon for this target
                std::string selectedWeapon = SelectWeapon(mag, *threat);
                if (selectedWeapon.empty()) continue; // No suitable weapon

                const MissileStats& mStats = TacticalDatabase::GetMissile(selectedWeapon);
                float distToTarget = MathUtils::GetDistance(transform.pos, threat->pos);

                // DOCTRINE RULE 3: Target must be in weapon employment zone
                if (distToTarget > mStats.maxRangeNM) continue;

                // Don't fire at close range - too late for missile to arm/guide
                constexpr float MIN_ENGAGEMENT_RANGE_NM = 0.5f;
                if (distToTarget < MIN_ENGAGEMENT_RANGE_NM) continue;

                // DOCTRINE RULE 4: Shoot-Look-Shoot
                // Are we ALREADY engaging this threat with a missile in flight?
                constexpr int MAX_SHOTS_PER_THREAT = 2; // Never put more than 2 missiles on one contact

                int activeShots = CountActiveShotsAgainst(log, *threat);
                if (activeShots >= MAX_SHOTS_PER_THREAT) {
                    continue; // Already saturated this target, move on to next threat
                }

                // Only fire the SECOND shot once the first one has had a chance to work
                // (don't dump both missiles in the same instant)
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
        auto shipTargets = registry.view<Transform2D, Hull, IFF>();

        for (auto missile : missiles) {
            if (registry.all_of<DeadTag>(missile)) continue;

            auto& mTrans = missiles.get<Transform2D>(missile);
            auto& warhead = missiles.get<Warhead>(missile);
            auto& mIFF = missiles.get<IFF>(missile);

            for (auto target: shipTargets) {
                auto& tTrans = shipTargets.get<Transform2D>(target);
                auto& tIFF = shipTargets.get<IFF>(target);

                if (mIFF.isHostile == tIFF.isHostile) continue;

                // Fast-Fail Distance Check
                MathUtils::Vec2 diff = MathUtils::Sub(mTrans.pos, tTrans.pos);
                float distNmSq = MathUtils::LengthSq(diff);

                // DETONATION
                if (distNmSq <= warhead.lethalRadiusNmSq) {
                    auto& hull = shipTargets.get<Hull>(target);
                    hull.currentHP -= warhead.yieldDamage;

                    if (VERBOSE_COMBAT_LOG) {
                        std::cout << "[IMPACT] Missile hit entity " << (uint32_t)target
                                  << " for " << warhead.yieldDamage << " dmg, HP now " << hull.currentHP << "\n";
                    }

                    if (hull.currentHP <= 0.0f) {
                        registry.emplace_or_replace<DeadTag>(target);
                        if (VERBOSE_COMBAT_LOG) {
                            std::cout << "[KILL] Entity " << (uint32_t)target << " destroyed\n";
                        }
                    }

                    registry.emplace_or_replace<DeadTag>(missile);
                    break;
                }
                if (registry.all_of<DeadTag>(missile)) continue;

                for (auto otherMissile : missiles) {
                    if (otherMissile == missile) continue;
                    if (registry.all_of<DeadTag>(otherMissile)) continue;

                    auto& oIFF = missiles.get<IFF>(otherMissile);
                    if (mIFF.isHostile == oIFF.isHostile) continue;

                    auto& oTrans = missiles.get<Transform2D>(otherMissile);
                    float distNmSq = MathUtils::LengthSq(MathUtils::Sub(mTrans.pos, oTrans.pos));
                    constexpr float INTERCEPT_KILL_RADIUS_NM = 0.05f; // ~300ft
                    if (distNmSq <= INTERCEPT_KILL_RADIUS_NM * INTERCEPT_KILL_RADIUS_NM) {
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
        registry.emplace<Kinematics>(missile,
            shooterKin.velocity, shooterKin.headingVector,
            mStats.maxSpeedKnots, 600.0f, mStats.maxSpeedKnots, 50.0f);
        registry.emplace<Aerodynamics>(missile,
            mStats.massKg, 1.0f / mStats.massKg, mStats.areaM2,
            mStats.thrustNewtons, mStats.maxFuelKg, mStats.burnRateKgSec,
            mStats.cruiseAltitudeMeters);
        registry.emplace<Warhead>(missile,
            mStats.warheadYield, mStats.lethalRadiusNM,
            mStats.lethalRadiusNM * mStats.lethalRadiusNM);
        registry.emplace<RadarSignature>(missile, mStats.rcs, mStats.rcsFourthRoot);
        registry.emplace<IFF>(missile, shooterIFF.isHostile);

        return missile;
    }
};
