#include "mad/runtime/autonomy_stack.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace mad::runtime {

CsvLogger::CsvLogger(const std::string& output_path)
    : m_stream(output_path) {
    m_stream << "time,ego_x,ego_y,ego_yaw,ego_speed,decision,target_lane,task_directive,top_risk_actor,top_risk_score,min_ttc,lane_error,headway,longitudinal_accel,raw_detections,fused_objects,speed_profile_points,collided,aeb_active,mission_state,likely_cut_in_objects,safety_reason,actors\n";
}

void CsvLogger::Log(double sim_time,
                    const mad::simulation::ActorState& ego,
                    const mad::planning::BehaviorDecision& decision,
                    const mad::planning::TaskTreeDecision& task_decision,
                    const std::vector<mad::prediction::RiskObject>& top_risks,
                    const std::vector<mad::simulation::ActorState>& actors,
                    const FrameMetrics& metrics,
                    int raw_detection_count,
                    int fused_count,
                    std::size_t speed_profile_count,
                    bool collided,
                    const std::string& safety_reason,
                    bool aeb_active,
                    const std::string& mission_state,
                    int likely_cut_in_objects) {
    std::ostringstream encoded;
    for (std::size_t i = 0; i < actors.size(); ++i) {
        const auto& actor = actors[i];
        encoded << actor.id << ':' << actor.x << ':' << actor.y << ':' << actor.speed << ':'
                << actor.preferred_lane << ':' << mad::simulation::ToString(actor.behavior);
        if (i + 1 < actors.size()) {
            encoded << '|';
        }
    }

    const auto* top_risk = top_risks.empty() ? nullptr : &top_risks.front();
    m_stream << std::fixed << std::setprecision(3)
             << sim_time << ','
             << ego.x << ','
             << ego.y << ','
             << ego.yaw << ','
             << ego.speed << ','
             << decision.state << ','
             << decision.target_lane << ','
             << mad::planning::TaskTreePlanner::ToString(task_decision.directive) << ','
             << (top_risk != nullptr ? top_risk->actor_id : -1) << ','
             << (top_risk != nullptr ? top_risk->risk_score : 0.0) << ','
             << metrics.min_ttc << ','
             << metrics.lane_error << ','
             << metrics.headway << ','
             << metrics.longitudinal_accel << ','
             << raw_detection_count << ','
             << fused_count << ','
             << speed_profile_count << ','
             << (collided ? 1 : 0) << ','
             << (aeb_active ? 1 : 0) << ','
             << mission_state << ','
             << likely_cut_in_objects << ','
             << safety_reason << ','
             << encoded.str() << '\n';
}

AutonomyStack::AutonomyStack(mad::map::LaneMap lane_map)
    : m_laneMap(std::move(lane_map))
    , m_simulator(mad::simulation::CreateBuiltInAdapter(m_laneMap))
    , m_localizationFilter(m_laneMap)
    , m_routePlanner(m_laneMap) {
}

RunSummary AutonomyStack::RunScenario(const std::string& scenario_name,
                                      double sim_duration,
                                      double dt,
                                      const std::string& csv_log_path) {
    std::filesystem::create_directories(std::filesystem::path(csv_log_path).parent_path());
    CsvLogger logger(csv_log_path);
    EventLogger event_logger((std::filesystem::path("out") / "events" / (scenario_name + "_events.csv")).string());
    DecisionTraceWriter decision_trace((std::filesystem::path("out") / "decision_traces" / (scenario_name + "_decision_trace.csv")).string());
    RiskTraceWriter risk_trace((std::filesystem::path("out") / "risk_traces" / (scenario_name + "_risk_trace.csv")).string());
    HypothesisTraceWriter hypothesis_trace((std::filesystem::path("out") / "hypothesis_traces" / (scenario_name + "_hypothesis_trace.csv")).string());
    InteractionTraceWriter interaction_trace((std::filesystem::path("out") / "interaction_traces" / (scenario_name + "_interaction_trace.csv")).string());
    mad::simulation::ExternalSimBridge bridge("out/bridge/" + scenario_name + "_state.csv",
                                              "out/bridge/" + scenario_name + "_command.csv");
    mad::simulation::Ros2MessageSchemaWriter ros2_schema_writer((std::filesystem::path("out") / "interop" / "ros2_message_schema_v9.txt").string());
    ros2_schema_writer.WriteDefaultSchemas();
    mad::simulation::Ros2BridgeStub ros2_bridge("out/interop/" + scenario_name + "_ros2_topics.csv");
    mad::simulation::CarlaBridgeStub carla_bridge("out/interop/" + scenario_name + "_carla_bridge.csv");
    mad::simulation::TaskBridgeStub task_bridge("out/interop/" + scenario_name + "_task_bridge.csv");

    auto scenario = m_scenarioLoader.TryLoad(scenario_name, m_laneMap, "configs/scenarios");
    if (!scenario.has_value()) {
        scenario = mad::simulation::MakeScenario(scenario_name, m_laneMap);
    }

    m_simulator->Reset(*scenario);
    m_localizationFilter.Reset(scenario->ego);
    m_behaviorPlanner.Reset();
    m_evaluator.Reset();
    m_safetyGuardian.Reset();
    m_safetySupervisor.Reset();
    m_trackManager.Reset();
    m_missionManager.Reset();
    m_fallbackManager.Reset();

    RunSummary summary;
    summary.scenario_name = scenario_name;

    int previous_lane = m_laneMap.ClosestLane(m_simulator->Snapshot().ego.y);
    mad::simulation::ControlCommand previous_command {};
    std::vector<mad::common::TrajectoryPoint> previous_trajectory;

    for (double t = 0.0; t < sim_duration; t += dt) {
        const auto snapshot = m_simulator->Snapshot();
        m_localizationFilter.Predict(dt, previous_command);
        m_localizationFilter.CorrectWithTruth(snapshot.ego);

        const int current_lane = m_localizationFilter.state().lane_id;
        const int preferred_goal_lane = std::max(0, current_lane - 1);
        const auto route_plan = m_routePlanner.PlanRoute(current_lane, preferred_goal_lane);

        const auto detections = m_sensorModel.Observe(snapshot);
        const auto fused = m_fusionEngine.Fuse(detections, m_laneMap);
        const auto occupancy = m_occupancyGridBuilder.Build(snapshot, fused, m_laneMap);
        const auto tracks = m_trackManager.Update(detections, m_laneMap, snapshot.sim_time);
        const auto lane_flows = m_laneOccupancyAnalyzer.Analyze(snapshot, tracks, occupancy, m_laneMap);
        const auto intentions = m_intentionPredictor.Evaluate(tracks, snapshot.ego, m_laneMap);
        const auto predictions = m_predictor.Predict(tracks, intentions, m_laneMap, 3.0, 0.5);
        const auto hypotheses = m_hypothesisGenerator.Generate(snapshot.ego, predictions, m_laneMap, 3.0, 0.5);
        const auto interaction_conflicts = m_jointInteractionPredictor.Analyze(predictions, hypotheses, current_lane);
        const auto interaction_summary = m_multiAgentBehaviorPredictor.Evaluate(predictions, interaction_conflicts, current_lane);
        summary.max_interaction_conflicts = std::max(summary.max_interaction_conflicts, static_cast<int>(interaction_conflicts.size()));
        const auto scene_risk = m_sceneRiskAssessor.Evaluate(snapshot.ego, occupancy, predictions, m_laneMap, route_plan.goal_lane);
        const auto top_risks = m_riskObjectRanker.Rank(snapshot.ego, predictions, m_laneMap, route_plan.goal_lane);

        std::vector<int> blocked_lanes;
        for (const auto& flow : lane_flows) {
            if (flow.route_blocked) {
                blocked_lanes.push_back(flow.lane_id);
            }
        }
        std::vector<mad::map::LaneCorridor> corridor_candidates;
        for (int goal_lane = 0; goal_lane < m_laneMap.lane_count(); ++goal_lane) {
            corridor_candidates.push_back(m_laneCorridorBuilder.Build(m_laneMap, current_lane, goal_lane, snapshot.ego.x, 80.0, blocked_lanes));
        }
        const auto best_corridor = m_corridorCostEvaluator.SelectBest(corridor_candidates, lane_flows, top_risks);
        const auto lane_sequence_decision = m_laneSequencePlanner.Plan(route_plan, scene_risk, lane_flows, top_risks, current_lane, m_laneMap);

        int likely_cut_in_objects = 0;
        for (const auto& item : intentions) {
            if (item.likely_cut_in) {
                ++likely_cut_in_objects;
            }
        }

        const bool lane_change_in_progress = std::abs(snapshot.ego.y - m_laneMap.LaneCenterY(current_lane)) > 0.45;

        mad::planning::BehaviorTreeInput bt_input;
        bt_input.current_lane = current_lane;
        bt_input.preferred_lane = scene_risk.preferred_lane;
        bt_input.route_goal_lane = route_plan.goal_lane;
        bt_input.current_lane_risk = scene_risk.current_lane_risk;
        bt_input.preferred_lane_risk = scene_risk.preferred_lane_risk;
        bt_input.likely_cut_in_count = scene_risk.current_lane_cut_in_count;
        bt_input.lane_change_in_progress = lane_change_in_progress;
        bt_input.emergency_recommended = scene_risk.emergency_recommended;
        for (const auto& profile : scene_risk.lane_profiles) {
            if (profile.lane_id == current_lane) {
                bt_input.front_gap = profile.front_gap;
                bt_input.min_ttc = profile.min_ttc;
                break;
            }
        }
        const auto bt_decision = m_behaviorTreePlanner.Tick(bt_input);

        mad::planning::TaskTreeInput task_input;
        task_input.mission_state = m_missionManager.DebugName();
        task_input.current_lane = current_lane;
        task_input.route_goal_lane = route_plan.goal_lane;
        task_input.lane_change_in_progress = lane_change_in_progress;
        task_input.scene_risk = scene_risk;
        task_input.top_risk_objects = top_risks;
        task_input.lane_flows = lane_flows;
        task_input.interaction_conflict_count = interaction_summary.lane_change_conflict_count;
        task_input.lane_sequence_target_lane = lane_sequence_decision.immediate_target_lane;
        const auto task_decision = m_taskTreePlanner.Tick(task_input);
        if (lane_sequence_decision.immediate_target_lane != current_lane) {
            ++summary.lane_sequence_replans;
        }
        if (best_corridor.recommended_lane != current_lane) {
            ++summary.corridor_replans;
        }

        auto decision = m_behaviorPlanner.Plan(snapshot, route_plan, occupancy, predictions, m_laneMap, dt);
        decision.target_speed = std::min(decision.target_speed, scene_risk.global_speed_cap + task_decision.speed_cap_bias);
        decision.target_speed = std::max(0.0, decision.target_speed);

        if (bt_decision.directive == mad::planning::BehaviorDirective::Yield) {
            decision.state = "yield";
            decision.target_speed = std::min(decision.target_speed, std::max(5.0, snapshot.ego.speed - 2.0));
        } else if (bt_decision.directive == mad::planning::BehaviorDirective::ChangeLaneLeft) {
            decision.state = "change_left";
            decision.target_lane = std::max(0, current_lane - 1);
        } else if (bt_decision.directive == mad::planning::BehaviorDirective::ChangeLaneRight) {
            decision.state = "change_right";
            decision.target_lane = std::min(m_laneMap.lane_count() - 1, current_lane + 1);
        } else if (bt_decision.directive == mad::planning::BehaviorDirective::EmergencyBrake) {
            decision.state = "emergency_brake";
            decision.target_speed = 0.0;
            decision.emergency_brake = true;
            decision.target_lane = current_lane;
        }

        switch (task_decision.directive) {
        case mad::planning::TaskDirective::NominalDrive:
            break;
        case mad::planning::TaskDirective::OvertakeSlowerTraffic:
            if (!lane_change_in_progress && task_decision.requested_lane >= 0) {
                decision.state = "task_overtake";
                decision.target_lane = task_decision.requested_lane;
                decision.target_speed = std::max(decision.target_speed, std::min(24.0, snapshot.ego.speed + 1.0));
            }
            break;
        case mad::planning::TaskDirective::StabilizeAndYield:
            decision.state = "task_yield";
            decision.target_lane = current_lane;
            decision.target_speed = std::min(decision.target_speed, std::max(4.0, snapshot.ego.speed - 2.5));
            break;
        case mad::planning::TaskDirective::KeepLaneSafety:
            decision.state = "task_keep_lane_safety";
            decision.target_lane = current_lane;
            decision.target_speed = std::min(decision.target_speed, std::max(6.0, snapshot.ego.speed - 1.2));
            break;
        case mad::planning::TaskDirective::HardBrakeFallback:
            decision.state = "task_hard_brake";
            decision.target_lane = current_lane;
            decision.target_speed = 0.0;
            decision.emergency_brake = true;
            break;
        }

        if (task_decision.freeze_lane_change) {
            decision.target_lane = current_lane;
        }
        if (scene_risk.current_lane_cut_in_count > 0 && decision.state == "follow") {
            decision.state = "yield_follow";
        }
        if (best_corridor.recommended_lane != current_lane
            && best_corridor.cost + 0.35 < scene_risk.current_lane_risk + 1.2
            && !task_decision.freeze_lane_change) {
            decision.state = "corridor_replan";
            decision.target_lane = best_corridor.recommended_lane;
        }

        auto gap_acceptance = m_gapAcceptanceEvaluator.Evaluate(snapshot.ego,
                                                                 current_lane,
                                                                 decision.target_lane,
                                                                 occupancy,
                                                                 top_risks,
                                                                 hypotheses);
        const auto rss_lane_change = m_rssLaneChangeChecker.Evaluate(snapshot.ego,
                                                                     current_lane,
                                                                     decision.target_lane,
                                                                     occupancy,
                                                                     top_risks);
        if (decision.target_lane != current_lane && !rss_lane_change.safe) {
            gap_acceptance.accepted = false;
            gap_acceptance.reason = rss_lane_change.reason;
            ++summary.rss_lane_change_rejections;
        }
        if (!gap_acceptance.accepted && decision.target_lane != current_lane) {
            decision.state = "gap_rejected_keep_lane";
            decision.target_lane = current_lane;
            decision.target_speed = std::min(decision.target_speed, std::max(6.0, snapshot.ego.speed - 1.0));
            ++summary.gap_rejections;
            event_logger.Log(snapshot.sim_time, "gap_reject", gap_acceptance.reason, gap_acceptance.score);
        }

        const auto rss_longitudinal = m_rssLongitudinalChecker.Evaluate(snapshot.ego, snapshot.actors, current_lane);
        const auto rss_lateral = m_rssLateralChecker.Evaluate(snapshot.ego, snapshot.actors, decision.target_lane, m_laneMap);
        if (!rss_longitudinal.safe) {
            decision.state = "rss_longitudinal_brake";
            decision.target_speed = 0.0;
            decision.emergency_brake = true;
            event_logger.Log(snapshot.sim_time, "rss_longitudinal", rss_longitudinal.reason, rss_longitudinal.actual_distance);
        }
        if (decision.target_lane != current_lane && !rss_lateral.safe) {
            decision.state = "rss_lateral_block";
            decision.target_lane = current_lane;
            event_logger.Log(snapshot.sim_time, "rss_lateral", rss_lateral.reason, rss_lateral.nearest_lateral_clearance);
        }
        if (interaction_summary.label == "critical_interaction_cluster") {
            decision.target_speed = std::min(decision.target_speed, std::max(4.0, snapshot.ego.speed - 3.0));
            if (decision.state == "follow") {
                decision.state = "interaction_cluster_yield";
            }
        }

        const auto supervisor = m_safetySupervisor.Evaluate(snapshot,
                                                            decision,
                                                            gap_acceptance,
                                                            top_risks,
                                                            hypotheses);
        if (supervisor.veto_lane_change) {
            decision.target_lane = current_lane;
        }
        if (supervisor.speed_clamp_active) {
            decision.target_speed = std::min(decision.target_speed, supervisor.override_target_speed);
        }
        if (supervisor.emergency_brake) {
            decision.target_lane = current_lane;
            decision.target_speed = 0.0;
            decision.emergency_brake = true;
        }
        if (supervisor.intervention_level > 0) {
            ++summary.supervisor_interventions;
            event_logger.Log(snapshot.sim_time, "supervisor", supervisor.reason, static_cast<double>(supervisor.intervention_level));
        }

        const auto stitched = m_trajectoryStitcher.Stitch(snapshot.ego, previous_trajectory, dt);
        auto speed_profile = m_speedPlanner.Plan(snapshot.ego.speed,
                                                 decision.target_speed,
                                                 decision.front_gap,
                                                 decision.front_speed,
                                                 decision.min_ttc,
                                                 3.0,
                                                 0.2);
        auto trajectory = m_trajectoryPlanner.Plan(snapshot, decision, speed_profile, stitched, m_laneMap, 3.0, 0.2);

        const auto guardian = m_safetyGuardian.Evaluate(snapshot, decision, predictions, trajectory);
        if (guardian.hold_current_lane) {
            decision.target_lane = current_lane;
        }

        const bool final_speed_override = guardian.aeb_active
                                       || guardian.override_target_speed < decision.target_speed
                                       || supervisor.speed_clamp_active
                                       || supervisor.emergency_brake;
        if (final_speed_override) {
            decision.target_speed = std::min(decision.target_speed, guardian.override_target_speed);
            if (supervisor.speed_clamp_active || supervisor.emergency_brake) {
                decision.target_speed = std::min(decision.target_speed, supervisor.override_target_speed);
            }
            speed_profile = m_speedPlanner.Plan(snapshot.ego.speed,
                                                decision.target_speed,
                                                decision.front_gap,
                                                decision.front_speed,
                                                decision.min_ttc,
                                                2.0,
                                                0.2);
            trajectory = m_trajectoryPlanner.Plan(snapshot, decision, speed_profile, stitched, m_laneMap, 2.0, 0.2);
        }

        const bool route_blocked = !blocked_lanes.empty() && std::find(blocked_lanes.begin(), blocked_lanes.end(), current_lane) != blocked_lanes.end();
        const auto fallback = m_fallbackManager.Update({decision.min_ttc,
                                                        supervisor.intervention_level > 0,
                                                        guardian.aeb_active,
                                                        route_blocked,
                                                        !gap_acceptance.accepted && decision.target_lane == current_lane,
                                                        snapshot.ego.speed});
        if (fallback.active) {
            decision.target_lane = current_lane;
            decision.target_speed = std::min(decision.target_speed, fallback.override_target_speed);
            if (fallback.minimal_risk_stop) {
                decision.state = "fallback_stop";
                decision.emergency_brake = true;
            } else {
                decision.state = "fallback_keep_lane";
            }
            speed_profile = m_speedPlanner.Plan(snapshot.ego.speed,
                                                decision.target_speed,
                                                decision.front_gap,
                                                decision.front_speed,
                                                decision.min_ttc,
                                                2.0,
                                                0.2);
            trajectory = m_trajectoryPlanner.Plan(snapshot, decision, speed_profile, stitched, m_laneMap, 2.0, 0.2);
            event_logger.Log(snapshot.sim_time, "fallback", fallback.reason, static_cast<double>(fallback.escalation_level));
        }

        const bool yield_required = (bt_decision.directive == mad::planning::BehaviorDirective::Yield)
                                 || task_decision.directive == mad::planning::TaskDirective::StabilizeAndYield
                                 || likely_cut_in_objects > 0
                                 || decision.state == "follow"
                                 || decision.state == "yield_follow"
                                 || fallback.active;
        const auto mission = m_missionManager.Step({false,
                                                    guardian.aeb_active || supervisor.emergency_brake,
                                                    yield_required,
                                                    snapshot.ego.speed,
                                                    snapshot.ego.x,
                                                    m_laneMap.road_length()});
        if (lane_sequence_decision.immediate_target_lane != current_lane) {
            event_logger.Log(snapshot.sim_time, "lane_sequence", lane_sequence_decision.reason, static_cast<double>(lane_sequence_decision.immediate_target_lane));
        }

        if (mission.state_changed) {
            event_logger.Log(snapshot.sim_time, "mission_state", m_missionManager.DebugName(), static_cast<double>(snapshot.ego.preferred_lane));
            if (mission.state == MissionState::Yielding) {
                ++summary.yield_events;
            }
        }

        auto command = m_controller.Compute(snapshot.ego, trajectory, dt);
        if (guardian.aeb_active) {
            command.acceleration = std::min(command.acceleration, guardian.override_acceleration);
            event_logger.Log(snapshot.sim_time, "aeb", guardian.reason, guardian.override_acceleration);
        }
        if (supervisor.speed_clamp_active || supervisor.emergency_brake) {
            command.acceleration = std::min(command.acceleration, supervisor.override_acceleration);
        }
        if (fallback.active) {
            command.acceleration = std::min(command.acceleration, fallback.minimal_risk_stop ? -4.5 : -2.0);
        }

        decision_trace.Log(snapshot.sim_time,
                           decision.state,
                           bt_decision,
                           task_decision,
                           scene_risk,
                           top_risks,
                           decision.target_lane,
                           decision.target_speed);
        risk_trace.Log(snapshot.sim_time, top_risks, lane_flows);
        hypothesis_trace.Log(snapshot.sim_time, hypotheses);
        interaction_trace.Log(snapshot.sim_time, interaction_conflicts);
        task_bridge.Publish(snapshot.sim_time, task_decision);
        bridge.PublishCommand(snapshot.sim_time, command);
        ros2_bridge.PublishTopics(snapshot, bt_decision, task_decision, scene_risk, top_risks, command);

        m_simulator->Step(dt, command);
        bridge.PublishState(m_simulator->Snapshot());
        carla_bridge.PublishFrame(m_simulator->Snapshot(), bt_decision, task_decision, scene_risk, top_risks);

        const bool collided = m_simulator->HasCollision();
        if (collided) {
            event_logger.Log(m_simulator->Snapshot().sim_time, "collision", "ego_collision", 1.0);
        }

        const auto frame_metrics = m_evaluator.Update(m_simulator->Snapshot(), predictions, m_laneMap);
        logger.Log(m_simulator->Snapshot().sim_time,
                   m_simulator->Snapshot().ego,
                   decision,
                   task_decision,
                   top_risks,
                   m_simulator->Snapshot().actors,
                   frame_metrics,
                   static_cast<int>(detections.size()),
                   static_cast<int>(fused.size()),
                   speed_profile.size(),
                   collided,
                   guardian.reason.empty() ? supervisor.reason : guardian.reason,
                   guardian.aeb_active || supervisor.emergency_brake,
                   m_missionManager.DebugName(),
                   likely_cut_in_objects);

        const int new_lane = m_laneMap.ClosestLane(m_simulator->Snapshot().ego.y);
        if (new_lane != previous_lane) {
            ++summary.lane_changes;
            event_logger.Log(m_simulator->Snapshot().sim_time,
                             "lane_change",
                             std::to_string(previous_lane) + "->" + std::to_string(new_lane),
                             static_cast<double>(new_lane));
            previous_lane = new_lane;
        }

        previous_command = command;
        previous_trajectory = trajectory;
        if (collided) {
            summary.collided = true;
            break;
        }
        if (m_missionManager.state() == MissionState::Completed) {
            event_logger.Log(m_simulator->Snapshot().sim_time, "mission_complete", scenario_name, m_simulator->Snapshot().ego.x);
            break;
        }
    }

    const auto metrics = m_evaluator.Summary();
    summary.sim_duration = m_simulator->Snapshot().sim_time;
    summary.final_x = m_simulator->Snapshot().ego.x;
    summary.final_y = m_simulator->Snapshot().ego.y;
    summary.avg_speed = metrics.avg_speed;
    summary.min_ttc = metrics.min_ttc;
    summary.avg_lane_error = metrics.avg_lane_error;
    summary.avg_headway = metrics.avg_headway;
    summary.max_abs_accel = metrics.max_abs_accel;
    summary.aeb_triggers = m_safetyGuardian.aeb_trigger_count();
    summary.fallback_activations = m_fallbackManager.activation_count();
    summary.mission_result = m_missionManager.DebugName();
    const auto diagnostics = m_diagnosticsMonitor.Evaluate(summary);
    for (const auto& record : diagnostics.records) {
        if (record.level == "warn") {
            ++summary.diagnostics_warn_count;
        } else if (record.level == "error") {
            ++summary.diagnostics_error_count;
        }
    }
    return summary;
}

} // namespace mad::runtime
