#include "test_framework.hpp"
#include "mad/control/controller.hpp"
#include "mad/control/feedforward_controller.hpp"

MAD_TEST(ControlAdvanced, FeedforwardControllerProducesSteeringBiasOnCurvedPath) {
    std::vector<mad::common::TrajectoryPoint> trajectory {
        {0.0, 0.0, 0.0, 0.00, 16.0},
        {0.2, 4.0, 0.2, 0.06, 16.0},
        {0.4, 8.0, 0.8, 0.12, 16.0},
    };
    mad::control::FeedforwardController ff;
    const auto cmd = ff.Compute(trajectory, 15.0);
    MAD_REQUIRE(cmd.steering_bias > 0.0);
}

MAD_TEST(ControlAdvanced, ControllerUsesFeedforwardWithoutBreakingLimits) {
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, 0.0, 0.02, 18.0, 4.8, 1.9, 1, true};
    std::vector<mad::common::TrajectoryPoint> trajectory {
        {0.0, 0.0, 0.0, 0.00, 18.0},
        {0.2, 5.0, 0.3, 0.08, 19.0},
        {0.4, 10.0, 1.0, 0.16, 19.0},
    };
    mad::control::Controller controller;
    const auto command = controller.Compute(ego, trajectory, 0.1);
    MAD_REQUIRE(command.steering_angle <= 0.45);
    MAD_REQUIRE(command.steering_angle >= -0.45);
}
