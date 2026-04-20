#pragma once
#include "mad/map/lane_map.hpp"
#include "mad/prediction/constant_velocity_predictor.hpp"
#include "mad/prediction/interaction_predictor.hpp"
#include "mad/simulation/world.hpp"
namespace mad::runtime {
struct FrameMetrics { double min_ttc{1.0e9}; double lane_error{0.0}; double headway{1.0e9}; double longitudinal_accel{0.0}; };
struct AggregateMetrics { int samples{0}; double avg_speed{0.0}; double min_ttc{1.0e9}; double avg_lane_error{0.0}; double avg_headway{0.0}; double max_abs_accel{0.0}; };
class Evaluator {
public:
    void Reset(); FrameMetrics Update(const mad::simulation::WorldSnapshot& snapshot, const std::vector<mad::prediction::PredictedObject>& predictions, const mad::map::LaneMap& lane_map); AggregateMetrics Summary() const;
private:
    AggregateMetrics m_metrics{}; mad::prediction::InteractionPredictor m_interactionPredictor; double m_previousSpeed{0.0}; double m_previousTime{0.0}; bool m_hasPrevious{false};
};
}
