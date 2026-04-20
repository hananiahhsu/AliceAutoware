#pragma once

#include "mad/control/controller.hpp"
#include "mad/localization/localization_filter.hpp"
#include "mad/map/lane_corridor_builder.hpp"
#include "mad/map/route_planner.hpp"
#include "mad/perception/fusion_engine.hpp"
#include "mad/perception/lane_occupancy_analyzer.hpp"
#include "mad/perception/occupancy_grid.hpp"
#include "mad/perception/sensor_model.hpp"
#include "mad/planning/behavior_planner.hpp"
#include "mad/planning/behavior_tree.hpp"
#include "mad/planning/corridor_cost_evaluator.hpp"
#include "mad/planning/gap_acceptance_evaluator.hpp"
#include "mad/planning/lane_sequence_planner.hpp"
#include "mad/planning/speed_planner.hpp"
#include "mad/planning/task_tree.hpp"
#include "mad/planning/trajectory_planner.hpp"
#include "mad/planning/trajectory_stitcher.hpp"
#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/prediction/intention_predictor.hpp"
#include "mad/prediction/joint_interaction_predictor.hpp"
#include "mad/prediction/multi_agent_behavior_predictor.hpp"
#include "mad/prediction/risk_object_ranker.hpp"
#include "mad/prediction/scene_risk_assessor.hpp"
#include "mad/prediction/trajectory_hypothesis_generator.hpp"
#include "mad/runtime/decision_trace_writer.hpp"
#include "mad/runtime/diagnostics_monitor.hpp"
#include "mad/runtime/fallback_manager.hpp"
#include "mad/runtime/evaluator.hpp"
#include "mad/runtime/event_logger.hpp"
#include "mad/runtime/interaction_trace_writer.hpp"
#include "mad/runtime/hypothesis_trace_writer.hpp"
#include "mad/runtime/mission_manager.hpp"
#include "mad/runtime/risk_trace_writer.hpp"
#include "mad/safety/safety_guardian.hpp"
#include "mad/safety/rss_lane_change_checker.hpp"
#include "mad/safety/rss_lateral_checker.hpp"
#include "mad/safety/rss_longitudinal_checker.hpp"
#include "mad/safety/safety_supervisor.hpp"
#include "mad/simulation/carla_bridge_stub.hpp"
#include "mad/simulation/external_sim_bridge.hpp"
#include "mad/simulation/ros2_bridge_stub.hpp"
#include "mad/simulation/ros2_message_schema.hpp"
#include "mad/simulation/scenario_loader.hpp"
#include "mad/simulation/simulator_adapter.hpp"
#include "mad/simulation/task_bridge_stub.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace mad::runtime {

struct RunSummary {
    std::string scenario_name;
    double sim_duration {0.0};
    double final_x {0.0};
    double final_y {0.0};
    bool collided {false};
    int lane_changes {0};
    double avg_speed {0.0};
    double min_ttc {1.0e9};
    double avg_lane_error {0.0};
    double avg_headway {0.0};
    double max_abs_accel {0.0};
    int aeb_triggers {0};
    int yield_events {0};
    int gap_rejections {0};
    int supervisor_interventions {0};
    int rss_lane_change_rejections {0};
    int lane_sequence_replans {0};
    int max_interaction_conflicts {0};
    int diagnostics_warn_count {0};
    int diagnostics_error_count {0};
    int fallback_activations {0};
    int corridor_replans {0};
    std::string mission_result {"unknown"};
};

class CsvLogger {
public:
    explicit CsvLogger(const std::string& output_path);
    void Log(double sim_time,
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
             int likely_cut_in_objects);

private:
    std::ofstream m_stream;
};

class AutonomyStack {
public:
    explicit AutonomyStack(mad::map::LaneMap lane_map);
    RunSummary RunScenario(const std::string& scenario_name,
                           double sim_duration,
                           double dt,
                           const std::string& csv_log_path);

private:
    mad::map::LaneMap m_laneMap;
    std::unique_ptr<mad::simulation::ISimulatorAdapter> m_simulator;
    mad::localization::LocalizationFilter m_localizationFilter;
    mad::map::RoutePlanner m_routePlanner;
    mad::map::LaneCorridorBuilder m_laneCorridorBuilder;
    mad::simulation::ScenarioLoader m_scenarioLoader;
    mad::perception::SensorModel m_sensorModel;
    mad::perception::FusionEngine m_fusionEngine;
    mad::perception::OccupancyGridBuilder m_occupancyGridBuilder;
    mad::perception::LaneOccupancyAnalyzer m_laneOccupancyAnalyzer;
    mad::perception::TrackManager m_trackManager;
    mad::prediction::IntentionPredictor m_intentionPredictor;
    mad::prediction::JointInteractionPredictor m_jointInteractionPredictor;
    mad::prediction::MultiAgentBehaviorPredictor m_multiAgentBehaviorPredictor;
    mad::prediction::ConstantVelocityPredictor m_predictor;
    mad::prediction::SceneRiskAssessor m_sceneRiskAssessor;
    mad::prediction::RiskObjectRanker m_riskObjectRanker;
    mad::prediction::TrajectoryHypothesisGenerator m_hypothesisGenerator;
    mad::planning::BehaviorPlanner m_behaviorPlanner;
    mad::planning::CorridorCostEvaluator m_corridorCostEvaluator;
    mad::planning::LaneSequencePlanner m_laneSequencePlanner;
    mad::planning::BehaviorTreePlanner m_behaviorTreePlanner;
    mad::planning::TaskTreePlanner m_taskTreePlanner;
    mad::planning::GapAcceptanceEvaluator m_gapAcceptanceEvaluator;
    mad::planning::SpeedPlanner m_speedPlanner;
    mad::planning::TrajectoryStitcher m_trajectoryStitcher;
    mad::planning::TrajectoryPlanner m_trajectoryPlanner;
    mad::control::Controller m_controller;
    mad::runtime::Evaluator m_evaluator;
    mad::runtime::MissionManager m_missionManager;
    mad::runtime::DiagnosticsMonitor m_diagnosticsMonitor;
    mad::runtime::FallbackManager m_fallbackManager;
    mad::safety::SafetyGuardian m_safetyGuardian;
    mad::safety::RssLaneChangeChecker m_rssLaneChangeChecker;
    mad::safety::RssLongitudinalChecker m_rssLongitudinalChecker;
    mad::safety::RssLateralChecker m_rssLateralChecker;
    mad::safety::SafetySupervisor m_safetySupervisor;
};

} // namespace mad::runtime
