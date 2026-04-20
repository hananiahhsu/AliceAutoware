#include "test_framework.hpp"

#include "mad/control/controller.hpp"
#include "mad/control/gain_scheduler.hpp"

MAD_TEST(Control, GainSchedulerReducesSteeringAtHighSpeed) {
    mad::control::GainScheduler scheduler;
    const auto low = scheduler.Schedule(8.0, 0.02);
    const auto high = scheduler.Schedule(28.0, 0.02);
    MAD_REQUIRE(high.steering_limit < low.steering_limit);
    MAD_REQUIRE(high.speed_p < low.speed_p);
}

MAD_TEST(Control, GainSchedulerBoostsCurvatureResponse) {
    mad::control::GainScheduler scheduler;
    const auto straight = scheduler.Schedule(18.0, 0.01);
    const auto curved = scheduler.Schedule(18.0, 0.18);
    MAD_REQUIRE(curved.heading_gain > straight.heading_gain);
    MAD_REQUIRE(curved.steering_limit >= straight.steering_limit);
}

MAD_TEST(Control, ControllerProducesBoundedCommand) {
    mad::control::Controller controller;
    mad::simulation::ActorState ego {1, mad::common::ActorType::Ego, 0.0, 0.0, 0.0, 24.0, 4.8, 1.9, 1, true};
    std::vector<mad::common::TrajectoryPoint> trajectory {
        {0.2, 5.0, 0.5, 0.05, 22.0},
        {0.4, 10.0, 1.0, 0.10, 21.0},
        {0.6, 15.0, 1.2, 0.12, 20.0}
    };
    const auto command = controller.Compute(ego, trajectory, 0.1);
    MAD_REQUIRE(command.steering_angle <= 0.45);
    MAD_REQUIRE(command.steering_angle >= -0.45);
    MAD_REQUIRE(command.acceleration <= 3.0);
    MAD_REQUIRE(command.acceleration >= -5.5);
}
